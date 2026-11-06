using System.Numerics;
using Fuse.Enemy;
using Fuse.Physics;
using Fuse.Scene;
using JoltPhysicsSharp;

internal static class Program
{
    private const float Dt = 1f / 60f;
    private static readonly SpiderLocomotionProfile Profile = SpiderLocomotionProfile.Default;

    private static int Main()
    {
        (string Name, Action Run)[] tests =
        [
            ("Low and off-centre obstacles block the feet corridor", FeetBand),
            ("The physical capsule still detects elevated obstructions", BodyBand),
            ("Two-stage detours work on floor, wall and ceiling", SurfaceRelativeRoutes),
            ("A clear corridor without source-plane support is not a route", MissingSupport),
            ("A preferred side is not swapped for the opposite route", PreferredSide),
            ("Committed routes do not re-query, switch sides or finish at the first corner", CommittedRoute),
            ("Significant target movement invalidates without immediate re-search", TargetMovement),
            ("Support elevation invalidates active detours without queries or immediate search", ElevatedSupportInvalidation),
            ("Elevated detour waypoints advance and finish using tangent distance", ElevatedWaypointArrival),
            ("ClearPlan on an empty plan preserves the search cooldown", EmptyPlanCooldown),
            ("No full local route fails closed within the navigation allowance", BoundedFailure),
            ("A player 10 km away uses a bounded local reconnect and local support", DistantTargetLocalDetour),
            ("Navigation, motor and gait retain their shared hard caps", SharedBudget),
            ("Invalid corridor inputs fail closed without native queries", InvalidInputs)
        ];
        int failed = 0;
        foreach (var (name, run) in tests)
        {
            try { run(); Console.WriteLine($"PASS {name}"); }
            catch (Exception error) { failed++; Console.Error.WriteLine($"FAIL {name}: {error}"); }
        }
        Console.WriteLine($"RESULT {tests.Length - failed}/{tests.Length} passed; real Jolt, no rendering or synthetic motor movement.");
        return failed == 0 ? 0 : 1;
    }

    private static void FeetBand()
    {
        foreach (float x in new[] { 0f, 1.1f, -1.1f })
        {
            using var f = new Fixture();
            // All boxes are below the old bodyCenter + normal * .18 rays.
            f.Box(new(x, 0.04f, 5), new(0.07f, 0.04f, 0.25f));
            Check(!f.Clear(f.At(0, 0), f.At(0, 10)), $"missed low obstacle at x={x}");
            Check(f.Solver.FrameShapeQueries <= 2 && f.Solver.FrameRaycasts == 0, "corridor query cost");
        }
        using var empty = new Fixture();
        Check(empty.Clear(empty.At(0, 0), empty.At(0, 10)), "flat support must not block the feet band");
    }

    private static void BodyBand()
    {
        using var f = new Fixture();
        f.Box(new(0, 1.5f, 5), new(0.3f, 0.05f, 0.3f));
        Check(!f.Clear(f.At(0, 0), f.At(0, 10)), "overhead capsule collision was lost");
    }

    private static void SurfaceRelativeRoutes()
    {
        foreach (int surface in new[] { 0, 1, 2 })
        {
            using var f = new Fixture(surface);
            f.LowBox();
            Vector3 start = f.At(0, 1), target = f.At(0, 18);
            Check(f.Plan(start, target), $"no detour on surface {surface}");
            Check(f.Planner.IsSameSurfaceDetourActive && !f.Planner.PlannedTransition.IsValid,
                "same-plane route must not invent a transition contact");
            Check(f.Planner.TryGetSteeringDirection(start, f.Normal, out var direction, out float distance), "no first leg");
            Vector3 corner = start + direction * distance;
            Check(MathF.Abs(Vector3.Dot(corner - start, f.Normal)) < 0.001f, "corner left source plane");
            Check(MathF.Abs(Vector3.Dot(corner - start, f.Side)) >= 1.85f, "insufficient lateral clearance");
            Check(f.Clear(start, corner), "first leg obstructed");
            Check(!f.Planner.IsPlannedSurfaceReached(f.Normal, corner), "route ended before passing obstacle");
            Check(f.Planner.TryGetSteeringDirection(corner, f.Normal, out direction, out distance), "no exit leg");
            Vector3 exit = corner + direction * distance;
            Check(Vector3.Dot(exit, f.Forward) > 9.25f, "exit did not pass obstacle");
            Check(f.Clear(corner, exit) && f.Clear(exit, target), "exit/reconnect obstructed");
            Check(f.Planner.IsPlannedSurfaceReached(f.Normal, exit), "route did not finish at exit");
            Console.WriteLine($"  surface={surface}, corner={corner}, exit={exit}, shape={f.Solver.FrameShapeQueries}, ray={f.Solver.FrameRaycasts}");
            Check(f.Solver.FrameRaycasts <= 64 && f.Solver.FrameShapeQueries <= 170, "navigation reservation exceeded");
        }
    }

    private static void MissingSupport()
    {
        using var f = new Fixture(floor: false);
        f.Box(new(0, -0.5f, 1), new(30, 0.5f, 2));
        f.LowBox();
        Check(!f.Plan(f.At(0, 1), f.At(0, 18)), "route crossed an unsupported gap");
        Check(!f.Planner.HasPlan && f.Solver.FrameRaycasts <= 25 && f.Solver.FrameShapeQueries <= 98,
            "unsupported route was installed or search exceeded its allowance");
    }

    private static void PreferredSide()
    {
        using var f = new Fixture();
        f.LowBox();
        Vector3 start = f.At(0, 1), target = f.At(0, 18);
        Check(f.Solver.TryFindSameSurfaceDetour(start, f.Normal, f.Forward, target,
            Profile.BodyClearance, BodyID.Invalid, Vector3.UnitX, out var waypoint, out _, out var side),
            "preferred positive side has no route");
        Check(side.X > 0.99f && waypoint.X > 1.85f, "preferred side was replaced by deterministic negative side");
        f.Solver.BeginFrame();
        // Block only the committed side, leaving the negative route available.
        f.Box(new(5, 1, 1), new(0.1f, 1, 20));
        Check(!f.Solver.TryFindSameSurfaceDetour(start, f.Normal, f.Forward, target,
            Profile.BodyClearance, BodyID.Invalid, side, out _, out _, out _),
            "failed committed side silently flipped to the opposite route");
    }

    private static void CommittedRoute()
    {
        using var f = new Fixture();
        f.LowBox();
        Vector3 start = f.At(0, 1), target = f.At(0, 18);
        Check(f.Plan(start, target), "no initial plan");
        Check(f.Planner.TryGetSteeringDirection(start, f.Normal, out var initial, out float distance), "no steering");
        int shapes = f.Solver.FrameShapeQueries, rays = f.Solver.FrameRaycasts;
        for (int i = 0; i < 90; i++)
        {
            Check(f.Planner.TryGetDirection(Dt, start, f.Normal, f.Forward, target,
                Profile.BodyClearance, BodyID.Invalid, true, out var direction), "committed plan lost");
            Check(Vector3.Dot(direction, initial) > 0.999f, "steering alternated sides");
        }
        Check(f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays, "active plan re-probed physics");
        Vector3 corner = start + initial * distance;
        Check(!f.Planner.IsPlannedSurfaceReached(f.Normal, corner), "first corner discarded full route");
        Check(f.Planner.TryGetSteeringDirection(corner, f.Normal, out var advance, out _), "no forward continuation");
        Check(Vector3.Dot(advance, f.Forward) > 0.99f, "continuation cuts back toward target");
        Check(!f.Planner.TryGetSteeringDirection(corner, Vector3.UnitX, out _, out _), "stale source plane steered on a wall");
    }

    private static void TargetMovement()
    {
        using var f = new Fixture();
        f.LowBox();
        Vector3 start = f.At(0, 1), target = f.At(0, 18);
        Check(f.Plan(start, target), "no initial plan");
        int shapes = f.Solver.FrameShapeQueries, rays = f.Solver.FrameRaycasts;
        Check(f.Planner.TryGetDirection(Dt, start, f.Normal, f.Forward, target + Vector3.UnitX,
            Profile.BodyClearance, BodyID.Invalid, true, out _), "small target motion discarded route");
        Check(!f.Planner.TryGetDirection(Dt, start, f.Normal, f.Forward, target + Vector3.UnitX * 8,
            Profile.BodyClearance, BodyID.Invalid, true, out _), "teleported target kept stale route");
        Check(!f.Planner.HasPlan && f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays,
            "target invalidation performed immediate search");
        for (int i = 0; i < 5; i++)
        {
            f.Planner.ClearPlan();
            Check(!f.Plan(start, target + Vector3.UnitX * 8), "cooldown was erased after invalidation");
        }
        Check(f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays, "cooldown issued queries");
    }

    private static void ElevatedSupportInvalidation()
    {
        foreach (int surface in new[] { 0, 1, 2 })
        foreach (bool viaDetourApi in new[] { false, true })
        foreach (bool moveTarget in new[] { false, true })
        {
            using var f = new Fixture(surface);
            f.LowBox();
            Vector3 start = f.At(0, 1), target = f.At(0, 18);
            Check(f.Plan(start, target) && f.Planner.IsSameSurfaceDetourActive, "no active detour before support elevation");
            int shapes = f.Solver.FrameShapeQueries, rays = f.Solver.FrameRaycasts;
            Vector3 lift = f.Normal * 0.4f;
            f.TranslateSupport(0.4f);
            Vector3 elevatedStart = start + lift;
            Vector3 elevatedTarget = moveTarget ? target + lift : target;
            Check(Vector3.Distance(target, elevatedTarget) < MathF.Max(2f, Profile.BodyClearance * 3f),
                "target displacement must remain below the target invalidation threshold");

            Vector3 direction;
            bool hasDirection = viaDetourApi
                ? f.Planner.TryGetSameSurfaceDetour(Dt, elevatedStart, f.Normal, f.Forward, elevatedTarget,
                    Profile.BodyClearance, BodyID.Invalid, out direction)
                : f.Planner.TryGetDirection(Dt, elevatedStart, f.Normal, f.Forward, elevatedTarget,
                    Profile.BodyClearance, BodyID.Invalid, true, out direction);
            Check(!hasDirection && direction == Vector3.Zero, "elevated source plane kept stale detour steering");
            Check(!f.Planner.HasPlan && !f.Planner.IsSameSurfaceDetourActive,
                "support elevation below the target threshold did not invalidate the detour");
            Check(!f.Planner.TryGetSteeringDirection(elevatedStart, f.Normal, out _, out _),
                "invalidated world-space waypoint still provides steering");
            Check(f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays,
                "support invalidation issued queries or immediately searched for another route");

            for (int frame = 0; frame < 5; frame++)
            {
                Check(!f.Plan(elevatedStart, elevatedTarget), "detour retry ignored support invalidation cooldown");
                Check(!f.Planner.TryGetDirection(Dt, elevatedStart, f.Normal, f.Forward, elevatedTarget,
                    Profile.BodyClearance, BodyID.Invalid, true, out _), "transition search ignored support invalidation cooldown");
                Check(!f.Planner.HasPlan, "cooldown installed a replacement plan");
            }
            Check(f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays,
                "support invalidation cooldown issued queries");
        }
        Console.WriteLine("  support +0.4 m: 12 variants (3 surface frames, 2 entry points, fixed/lifted target), 0 extra shapes/rays");
    }

    private static void ElevatedWaypointArrival()
    {
        foreach (int surface in new[] { 0, 1, 2 })
        {
            using var f = new Fixture(surface);
            f.LowBox();
            Vector3 start = f.At(0, 1), target = f.At(0, 18);
            Check(f.Plan(start, target), "no initial detour");
            Check(f.Planner.TryGetSteeringDirection(start, f.Normal, out var direction, out float distance), "no first leg");
            Vector3 corner = start + direction * distance;
            int shapes = f.Solver.FrameShapeQueries, rays = f.Solver.FrameRaycasts;
            f.TranslateSupport(0.4f);
            Vector3 elevatedCorner = corner + f.Normal * 0.4f;
            Check(Vector3.Distance(corner, elevatedCorner) > 0.30f, "fixture must exceed the old 3D arrival tolerance");

            // The parent checks arrival before its target-update/invalidation
            // call. Isolate that API order so a normal-only offset cannot park
            // a plan forever with zero tangent steering and no arrival.
            Check(!f.Planner.IsPlannedSurfaceReached(f.Normal, elevatedCorner), "lateral corner prematurely finished the route");
            Check(f.Planner.TryGetSteeringDirection(elevatedCorner, f.Normal, out direction, out distance),
                "normal displacement prevented advancement to the exit leg");
            Check(Vector3.Dot(direction, f.Forward) > 0.99f, "exit steering left the tangent plane");
            Vector3 elevatedExit = elevatedCorner + direction * distance;
            Check(!f.Planner.TryGetSteeringDirection(elevatedExit, f.Normal, out var finalDirection, out float finalDistance) &&
                finalDirection == Vector3.Zero && finalDistance < 0.0001f, "exit did not have zero tangent steering distance");
            Check(f.Planner.IsPlannedSurfaceReached(f.Normal, elevatedExit),
                "zero tangent steering with a 0.4 m normal offset froze arrival at the stale 3D waypoint");
            Check(f.Planner.IsPlannedSurfaceReached(f.Normal, elevatedExit + f.Forward * 0.20f),
                "normal offset incorrectly consumed the tangent arrival tolerance");
            Check(!f.Planner.IsPlannedSurfaceReached(f.Normal, elevatedExit + f.Forward * 0.31f),
                "projected arrival accepted a point outside the unchanged 0.30 m tolerance");
            Check(f.Solver.FrameShapeQueries == shapes && f.Solver.FrameRaycasts == rays,
                "waypoint advancement or projected arrival queried physics");
        }
        Console.WriteLine("  elevated waypoints: corner advances, zero-distance exit completes, tangent tolerance remains 0.30 m, 0 extra shapes/rays");
    }

    private static void EmptyPlanCooldown()
    {
        using var f = new Fixture();
        int queryFrames = 0, totalShapes = 0;
        for (int i = 0; i < 30; i++)
        {
            f.Solver.BeginFrame();
            f.Planner.ClearPlan();
            Check(!f.Plan(f.At(0, 0), f.At(0, 18)), "invented detour on clear floor");
            if (f.Solver.FrameShapeQueries > 0) queryFrames++;
            totalShapes += f.Solver.FrameShapeQueries;
        }
        Console.WriteLine($"  empty-plan cooldown: {queryFrames}/30 search frames, {totalShapes} shapes, 0 rays");
        Check(queryFrames is >= 3 and <= 4 && totalShapes <= 8, "clear plan forced per-frame search or stopped retrying");
    }

    private static void BoundedFailure()
    {
        using var f = new Fixture();
        // The target is inside solid geometry. No route may be accepted just
        // because a lateral segment has support.
        f.Box(new(0, 3, 8), new(4, 3, 2));
        Check(!f.Plan(f.At(0, 1), f.At(0, 8)), "accepted incomplete route to solid target");
        Console.WriteLine($"  failed search: shape={f.Solver.FrameShapeQueries}, ray={f.Solver.FrameRaycasts}");
        Check(f.Solver.FrameShapeQueries <= 98 && f.Solver.FrameRaycasts <= 25,
            "local search exceeded its fixed allowance");
        Check(f.Solver.FrameShapeQueries <= 512 / 3 && f.Solver.FrameRaycasts <= 192 / 3,
            "navigation consumed motor/gait reservation");
    }

    private static void DistantTargetLocalDetour()
    {
        foreach (int surface in new[] { 0, 1, 2 })
        {
            using var f = new Fixture(surface, floor: false);
            // The entire support footprint fits inside a 50 m tangent radius
            // around the start. There is no floor anywhere near the player.
            f.Box(new(0, -0.5f, 1), new(35, 0.5f, 35));
            f.LowBox();
            Vector3 start = f.At(0, 1);
            Vector3 localTarget = start + f.Forward * 24f;
            Vector3 distantTarget = start + f.Forward * 10_000f;
            Check(f.Solver.TryFindSameSurfaceDetour(start, f.Normal, f.Forward, localTarget,
                Profile.BodyClearance, BodyID.Invalid, Vector3.Zero, out var localWaypoint,
                out float localScore, out var localSide), "local reference route was not found");
            int localShapes = f.Solver.FrameShapeQueries, localRays = f.Solver.FrameRaycasts;

            // A cast continuing toward the real player would encounter this
            // wall beyond the support patch. Local navigation must ignore it.
            f.Box(new(0, 3, 100), new(50, 3, 1));
            f.Solver.BeginFrame();
            Check(f.Solver.TryFindSameSurfaceDetour(start, f.Normal, f.Forward, distantTarget,
                Profile.BodyClearance, BodyID.Invalid, Vector3.Zero, out var waypoint,
                out float score, out var side), "distant player required support or clearance beyond the local horizon");
            Check(float.IsFinite(waypoint.LengthSquared()) && float.IsFinite(side.LengthSquared()) && float.IsFinite(score),
                "distant target produced non-finite route data");
            Check(Vector3.Distance(waypoint, localWaypoint) < 0.001f &&
                Vector3.Distance(side, localSide) < 0.001f && MathF.Abs(score - localScore) < 0.001f,
                "10 km target changed the local route or charged the full map distance to its score");
            Check(Vector3.Distance(start, waypoint) <= 12f && score is > 0f and < 50f,
                "waypoint or score escaped the bounded local route");
            int distantShapes = f.Solver.FrameShapeQueries, distantRays = f.Solver.FrameRaycasts;
            Check(distantShapes == localShapes && distantRays == localRays,
                "distant player or distant wall increased local query work");
            Check(distantShapes <= 98 && distantRays <= 25 && !f.Solver.QueryBudgetExhausted,
                "distant-target search exceeded its shape/ray allowance");

            f.Solver.BeginFrame();
            Check(f.Plan(start, distantTarget) && f.Planner.IsSameSurfaceDetourActive,
                "planner did not install the local detour for the distant player");
            Check(f.Solver.FrameShapeQueries <= 98 && f.Solver.FrameRaycasts <= 25,
                "planner exceeded the local search allowance");
            Check(f.Planner.TryGetSteeringDirection(start, f.Normal, out var direction, out float distance) &&
                float.IsFinite(direction.LengthSquared()) && float.IsFinite(distance), "non-finite first-leg steering");
            Vector3 corner = start + direction * distance;
            Check(!f.Planner.IsPlannedSurfaceReached(f.Normal, corner), "local route ended at the lateral corner");
            Check(f.Planner.TryGetSteeringDirection(corner, f.Normal, out direction, out distance) &&
                float.IsFinite(direction.LengthSquared()) && float.IsFinite(distance), "non-finite exit steering");
            Vector3 exit = corner + direction * distance;
            Check(float.IsFinite(exit.LengthSquared()) && Vector3.Distance(start, exit) < 24f &&
                Vector3.Dot(exit, f.Forward) > 9.25f, "exit was not local or did not pass the box");
            Check(f.Clear(start, corner) && f.Clear(corner, exit) && f.Clear(exit, localTarget),
                "distant-target detour has an obstructed local leg or reconnect");
            Check(f.Planner.IsPlannedSurfaceReached(f.Normal, exit), "local detour did not complete at its exit");
            Console.WriteLine($"  10 km target surface={surface}: score={score:F4}, shape={distantShapes}, ray={distantRays}, same route/cost as 24 m target; distant wall ignored");

            f.Solver.BeginFrame();
            Vector3 overflowTarget = f.At(0, float.MaxValue);
            Check(float.IsFinite(overflowTarget.Z) && !float.IsFinite((overflowTarget - start).Length()),
                "fixture must have finite target coordinates but non-finite target distance");
            Check(!f.Solver.TryFindSameSurfaceDetour(start, f.Normal, f.Forward, overflowTarget,
                Profile.BodyClearance, BodyID.Invalid, Vector3.Zero, out waypoint, out score, out side),
                "non-finite target distance was accepted");
            Check(f.Solver.FrameShapeQueries == 0 && f.Solver.FrameRaycasts == 0 &&
                float.IsFinite(waypoint.LengthSquared()) && float.IsFinite(side.LengthSquared()) && float.IsFinite(score),
                "non-finite target distance issued queries or returned non-finite data");
        }
    }

    private static void SharedBudget()
    {
        using var f = new Fixture();
        for (int i = 0; i < 300; i++) f.Clear(f.At(0, 0), f.At(0, 1));
        Check(f.Solver.FrameShapeQueries <= 512 / 3, "navigation shape reservation exceeded");
        Check(f.Solver.TryFindSupportContact(f.At(0, 0), f.Normal, f.Forward, Profile.BodyClearance,
            BodyID.Invalid, out var support), "missing support");
        for (int i = 0; i < 100; i++) f.Solver.HasSupportPatch(support, 0.2f, BodyID.Invalid);
        Check(f.Solver.FrameRaycasts == 192 / 3, "navigation ray reservation changed");
        f.Solver.BeginMotor();
        for (int i = 0; i < 100; i++) f.Solver.HasSupportPatch(support, 0.2f, BodyID.Invalid);
        Check(f.Solver.FrameRaycasts == 192 * 2 / 3, "motor ray reservation changed");
        f.Solver.BeginGait();
        for (int i = 0; i < 300; i++) f.Clear(f.At(0, 0), f.At(0, 1));
        for (int i = 0; i < 100; i++) f.Solver.HasSupportPatch(support, 0.2f, BodyID.Invalid);
        Check(f.Solver.FrameShapeQueries == 512 && f.Solver.FrameRaycasts == 192,
            "global hard caps changed or counters reset between stages");
    }

    private static void InvalidInputs()
    {
        using var f = new Fixture();
        Vector3 start = f.At(0, 0), end = f.At(0, 10);
        Check(!f.Solver.IsSameSurfacePathClear(start, end, Vector3.Zero, Profile.BodyClearance, BodyID.Invalid), "zero normal accepted");
        Check(!f.Solver.IsSameSurfacePathClear(start, end, f.Normal, float.NaN, BodyID.Invalid), "NaN clearance accepted");
        Check(!f.Clear(new(float.NaN, 0, 0), end), "NaN position accepted");
        Check(f.Solver.FrameShapeQueries == 0 && f.Solver.FrameRaycasts == 0, "invalid input used native queries");
    }

    private static void Check(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private sealed class Fixture : IDisposable
    {
        private readonly PhysicsWorld _physics = new(Vector3.Zero);
        private readonly List<RigidBody> _bodies = [];
        private readonly int _surface;
        public SpiderSurfaceSolver Solver { get; }
        public SpiderSurfacePursuitPlanner Planner { get; }
        public Vector3 Normal => Map(Vector3.UnitY);
        public Vector3 Forward => Map(Vector3.UnitZ);
        public Vector3 Side => Map(Vector3.UnitX);

        public Fixture(int surface = 0, bool floor = true)
        {
            _surface = surface;
            var scene = new SceneManager(_physics, null!);
            if (floor) Box(new(0, -0.5f, 0), new(100, 0.5f, 100));
            Solver = new SpiderSurfaceSolver(scene);
            Planner = new SpiderSurfacePursuitPlanner(Solver);
        }

        private Vector3 Map(Vector3 value) => _surface switch
        {
            1 => new(value.Y, -value.X, value.Z),
            2 => new(-value.X, -value.Y, value.Z),
            _ => value
        };
        public Vector3 At(float side, float forward) => Map(new(side, Profile.BodyClearance, forward));
        public void Box(Vector3 position, Vector3 half)
        {
            var body = new RigidBody().SetBox(Vector3.Abs(Map(half))).SetPosition(Map(position)).SetMass(0);
            body.Build(_physics);
            _bodies.Add(body);
        }
        public void LowBox() => Box(new(0, 0.35f, 8), new(1.25f, 0.35f, 1.25f));
        public void TranslateSupport(float distance)
        {
            foreach (var body in _bodies)
                _physics.SetBodyPosition(body.Native, _physics.GetBodyPosition(body.Native) + Normal * distance);
        }
        public bool Clear(Vector3 start, Vector3 end) => Solver.IsSameSurfacePathClear(start, end,
            Normal, Profile.BodyClearance, BodyID.Invalid);
        public bool Plan(Vector3 start, Vector3 target) => Planner.TryGetSameSurfaceDetour(Dt, start, Normal,
            Forward, target, Profile.BodyClearance, BodyID.Invalid, out _);
        public void Dispose()
        {
            Solver.Dispose();
            foreach (var body in _bodies) { _physics.DestroyBody(body.Native); body.Destroy(); }
            _physics.Dispose();
        }
    }
}
