using System.Numerics;
using Fuse.Enemy;
using Fuse.Physics;
using Fuse.Scene;
using JoltPhysicsSharp;
using GamePlayer = Fuse.Player.Player;

internal static class Program
{
    private static int Main()
    {
        (string Name, Action Run)[] tests =
        [
            ("Player.Position is capsule centre; feet and native body semantics", PlayerSemantics),
            ("Explicit capsule exclusion preserves the public resolver API", ExplicitCapsuleExclusion),
            ("CharacterVirtual inner capsule is excluded without actor registration", InnerCapsuleExclusion),
            ("Sensors and registered actor bodies never become support", SensorsAndActors),
            ("Floor under feet wins over nearby walls, ceiling and opposite slab", FloorPreference),
            ("Native curb contact wins over the lower floor under capsule centre", CurbContact),
            ("Footprint fallback finds a narrow ledge without lateral rays", FootprintFallback),
            ("Inside-convex hits and overhead colliders are not support", RejectInsideAndOverhead),
            ("Mesh back face is not flipped into a support surface", RejectBackFace),
            ("Airborne player cannot reuse distant native ground data", AirborneResolver),
            ("Invalid inputs fail closed; invalid preferred normal falls back to up", InvalidInputs),
            ("EnemySystem shares a throttled physical target, not capsule centre", UpdateCadence),
            ("Jump retention expires and landing refreshes immediately", JumpRetention),
            ("Noclip preserves F3, expires support and publishes current raw feet", NoclipRetention),
            ("Noclip without cached support never probes even near the floor", NoclipWithoutSupport),
            ("Teleport invalidates retention even before the next scheduled probe", Teleport),
            ("Player replacement cannot inherit the previous player's support", PlayerReplacement),
            ("Replacement by a dead player clears target flags before another live player", DeadPlayerReplacement),
            ("Removed support cannot survive in the target cache", RemovedSupport),
            ("Dead player clears both targets and stops stale pursuit", DeadPlayer),
            ("F3 off/on cannot restore an expired physical target", ToggleReset)
        ];
        int failures = 0;
        foreach (var (name, run) in tests)
        {
            try { run(); Console.WriteLine($"PASS {name}"); }
            catch (Exception exception) { failures++; Console.Error.WriteLine($"FAIL {name}: {exception}"); }
        }
        Console.WriteLine($"RESULT {tests.Length - failures}/{tests.Length} passed; real Jolt, Fuse project reference, no GL context or GPU assets.");
        return failures == 0 ? 0 : 1;
    }

    private static void PlayerSemantics()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        Near(player.Position - player.FeetPosition, new(0, 1.4f, 0), "standing capsule offset");
        Check(!player.NativeCharacter.InnerBodyID.IsValid,
            "current Player does not configure InnerBodyShape; do not invent a Character.BodyID");
        Console.WriteLine($"  Position={player.Position}; FeetPosition={player.FeetPosition}; InnerBodyID.IsValid={player.NativeCharacter.InnerBodyID.IsValid}; GroundState={player.NativeCharacter.GroundState}");
        Check(f.Resolver.TryResolvePlayer(player.FeetPosition, player.NativeCharacter, 0.6f, out var hit), "expected real player support");
        Support(hit, floor, Vector3.Zero);
    }

    private static void ExplicitCapsuleExclusion()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var capsule = f.Add(new RigidBody().SetCapsule(0.5f, 1.8f).SetPosition(new(0, 1.42f, 0)));
        Check(f.Resolver.TryResolve(new(0, 0.02f, 0), Vector3.UnitY, 0.6f, out var hit, capsule.Native), "expected floor through excluded player");
        Support(hit, floor, Vector3.Zero);
    }

    private static void InnerCapsuleExclusion()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        using var character = Character(f, new(0, 1.42f, 0));
        Check(character.InnerBodyID.IsValid, "test requires a real queryable native capsule");
        Check(!f.Scene.NonWalkableBodies.Contains(character.InnerBodyID), "exclusion must not depend on external actor registration");
        f.Refresh(character);
        Check(f.Resolver.TryResolvePlayer(new(0, 0.02f, 0), character, 0.6f, out var hit), "expected floor through native player body");
        Support(hit, floor, Vector3.Zero);
    }

    private static void SensorsAndActors()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        f.Add(new RigidBody().SetBox(new(1, 0.04f, 1)).SetPosition(new(0, 0.1f, 0)).SetTrigger(true));
        var actor = f.Add(new RigidBody().SetBox(new(1, 0.02f, 1)).SetPosition(new(0, 0.04f, 0)));
        f.Scene.NonWalkableBodies.Add(actor.Native);
        Check(f.Resolver.TryResolve(new(0, 0.02f, 0), Vector3.UnitY, 0.6f, out var hit), "expected floor beneath sensor and actor");
        Support(hit, floor, Vector3.Zero);
    }

    private static void FloorPreference()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        f.Add(new RigidBody().SetBox(new(0.1f, 3, 3)).SetPosition(new(0.15f, 1, 0)));
        f.Add(new RigidBody().SetBox(new(3, 0.1f, 3)).SetPosition(new(0, 3, 0)));
        f.Add(new RigidBody().SetBox(new(3, 0.1f, 3)).SetPosition(new(0, -0.8f, 0)));
        Check(f.Resolver.TryResolve(new(0, 0.02f, 0), Vector3.UnitY, 0.6f, out var hit), "expected floor");
        Support(hit, floor, Vector3.Zero);
    }

    private static void CurbContact()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var curb = f.Add(new RigidBody().SetBox(new(0.5f, 0.15f, 1)).SetPosition(new(0.8f, 0.15f, 0)));
        using var character = Character(f, new(0, 1.6f, 0));
        f.Refresh(character);
        Console.WriteLine($"  curb state={character.GroundState}; contact={character.GroundPosition}; normal={character.GroundNormal}");
        Check(character.GroundState == GroundState.OnGround, "capsule must actually be grounded on the curb");
        Vector3 feet = character.Position - new Vector3(0, 1.4f, 0);
        Check(f.Resolver.TryResolve(feet, Vector3.UnitY, 0.6f, out var direct, character.InnerBodyID) && direct.BodyId == floor.Native,
            "centre-only probe must see the lower floor in this regression fixture");
        Check(f.Resolver.TryResolvePlayer(feet, character, 0.6f, out var hit), "expected native capsule support");
        Check(hit.BodyId == curb.Native && hit.Normal.Y > 0.5f && hit.Point.Y > 0.2f,
            $"expected curb support, got {hit.Point} normal={hit.Normal} body={hit.BodyId}");
        Near(hit.Point, character.GroundPosition, "validated curb contact", 0.08f);
    }

    private static void FootprintFallback()
    {
        using var f = new Fixture();
        var ledge = f.Add(new RigidBody().SetBox(new(0.15f, 0.25f, 1)).SetPosition(new(0.4f, -0.25f, 0)));
        Check(f.Resolver.TryResolve(new(0, 0.02f, 0), Vector3.UnitY, 0.6f, out var hit), "expected support within capsule footprint");
        Support(hit, ledge, new(0.35f, 0, 0));
    }

    private static void RejectInsideAndOverhead()
    {
        using (var f = new Fixture())
        {
            f.Add(new RigidBody().SetBox(new(3, 1, 3)));
            Check(!f.Resolver.TryResolve(Vector3.Zero, Vector3.UnitY, 0.6f, out var hit) && !hit.IsValid,
                "fraction-zero convex hits must not become support");
        }
        using (var f = new Fixture())
        {
            f.Add(new RigidBody().SetBox(new(3, 0.1f, 3)).SetPosition(new(0, 0.5f, 0)));
            Check(!f.Resolver.TryResolve(Vector3.Zero, Vector3.UnitY, 0.6f, out _), "ceiling above probe lift is not support");
        }
    }

    private static void RejectBackFace()
    {
        using var f = new Fixture();
        Vector3[] vertices = [new(-2, 0, -2), new(-2, 0, 2), new(2, 0, -2), new(2, 0, 2)];
        f.Add(new RigidBody().SetTrimesh(vertices, [0, 2, 1, 1, 2, 3]));
        Check(!f.Resolver.TryResolve(new(0, 0.02f, 0), Vector3.UnitY, 0.6f, out _), "downward-facing mesh back face must not be flipped upwards");
    }

    private static void AirborneResolver()
    {
        using var f = new Fixture();
        f.Floor();
        using var character = Character(f, new(0, 1.42f, 0));
        f.Refresh(character);
        Check(character.GroundState == GroundState.OnGround, "initial native support required");
        character.Position = new(0, 3.42f, 0);
        // Deliberately do not RefreshContacts: NativeCharacter may expose the
        // previous ground between a jump/teleport and its next physics update.
        Check(!f.Resolver.TryResolvePlayer(new(0, 2.02f, 0), character, 0.6f, out var hit) && !hit.IsValid,
            "stale native ground two metres below the feet must be rejected");
    }

    private static void InvalidInputs()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        foreach (float distance in new[] { float.NaN, float.PositiveInfinity, 0f, -1f })
            Check(!f.Resolver.TryResolve(Vector3.Zero, Vector3.UnitY, distance, out var hit) && !hit.IsValid, "invalid search distance");
        Check(!f.Resolver.TryResolve(new(float.NaN, 0, 0), Vector3.UnitY, 1, out _), "invalid position");
        Check(f.Resolver.TryResolve(new(0, 0.02f, 0), new(float.NaN, 0, 0), 0.6f, out var fallback), "invalid normal should use world up");
        Support(fallback, floor, Vector3.Zero);
    }

    private static void UpdateCadence()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        Published(floor, Vector3.Zero);
        f.Physics.SetBodyPosition(floor.Native, new(0, -0.35f, 0));
        enemies.Update(0.01f);
        Published(floor, Vector3.Zero);
        enemies.Update(0.1f);
        Published(floor, new(0, -0.1f, 0));
    }

    private static void JumpRetention()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        player.NativeCharacter.Position = new(0.3f, 2.42f, 0);
        f.Refresh(player.NativeCharacter);
        Check(player.NativeCharacter.GroundState != GroundState.OnGround, "jump fixture must be airborne");
        enemies.Update(0.11f);
        Published(floor, Vector3.Zero);
        enemies.Update(0.25f);
        Check(!SpiderPatrol.HasPursuitTargetSurface, "retention must expire after 0.35 seconds without fresh support");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "unsupported fallback must be current, not stale");
        enemies.Update(0.11f);
        Check(!SpiderPatrol.HasPursuitTargetSurface, "a miss must not renew retention");
        player.NativeCharacter.Position = new(0, 1.42f, 0);
        f.Refresh(player.NativeCharacter);
        Check(player.IsOnGround, "landing fixture must be grounded");
        enemies.Update(0.001f);
        Published(floor, Vector3.Zero);
    }

    private static void NoclipRetention()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        Published(floor, Vector3.Zero);

        player.ToggleNoclip();
        player.NativeCharacter.Position = new(0.3f, 1.5f, 0);
        Check(player.IsNoclip && player.IsOnGround, "fixture deliberately leaves stale native grounded state in noclip");
        enemies.Update(0.11f);
        Published(floor, Vector3.Zero);
        enemies.Update(0.25f);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "noclip must preserve F3 and raw target but expire support after 0.35 seconds, even with floor in probe range");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "noclip must publish current raw feet after retention");

        player.NativeCharacter.Position = new(0.8f, 1.6f, 0);
        enemies.Update(0.11f);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "noclip must not reacquire nearby support or disable F3");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "raw noclip target must follow movement without projecting onto old support");

        player.ToggleNoclip();
        f.Refresh(player.NativeCharacter);
        enemies.Update(0);
        Published(floor, Vector3.Zero);
    }

    private static void NoclipWithoutSupport()
    {
        using var f = new Fixture();
        f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        player.ToggleNoclip();
        Check(player.IsOnGround, "native contact must be available to catch accidental noclip probes");
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "F3 enabled during noclip must publish raw feet, not resolve the available native ground contact");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "initial noclip raw target");

        player.NativeCharacter.Position = new(4, 8, 0);
        enemies.Update(0.5f);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "unsupported noclip must keep F3 active after retention duration");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "airborne noclip raw target must not retain the previous position");
    }

    private static void Teleport()
    {
        using var f = new Fixture();
        f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        player.NativeCharacter.Position = new(20, 4.42f, 0);
        enemies.Update(0);
        Check(!SpiderPatrol.HasPursuitTargetSurface, "teleport must invalidate cache immediately despite zero elapsed time");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "teleported target");
    }

    private static void PlayerReplacement()
    {
        using var f = new Fixture();
        f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        var replacement = f.PlayerAt(new(5, 3.42f, 0));
        enemies.SetPlayer(replacement);
        enemies.Update(0);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "live replacement must have its own raw target, never inherit previous support");
        Near(SpiderPatrol.PursuitTarget, replacement.FeetPosition, "replacement target");
    }

    private static void DeadPlayerReplacement()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        Published(floor, Vector3.Zero);

        var deadReplacement = f.PlayerAt(new(5, 1.42f, 0));
        deadReplacement.TakeDamage(deadReplacement.MaxHealth);
        enemies.SetPlayer(deadReplacement);
        enemies.Update(0);
        Check(!SpiderPatrol.PursuitEnabled && !SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "replacement by a dead player must clear raw/surface flags, not leave the old player's target active");
        Near(SpiderPatrol.PursuitTarget, Vector3.Zero, "dead replacement must clear the previous raw position");

        var liveReplacement = f.PlayerAt(new(5, 3.42f, 0));
        enemies.SetPlayer(liveReplacement);
        SpiderPatrol.SetPursuitEnabled(true);
        Check(!SpiderPatrol.HasPursuitTarget, "re-enabling F3 alone must not resurrect the cleared target");
        enemies.Update(0);
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "next live player must publish a fresh raw target without inheriting cleared support");
        Near(SpiderPatrol.PursuitTarget, liveReplacement.FeetPosition, "live target after dead replacement");
    }

    private static void RemovedSupport()
    {
        using var f = new Fixture();
        var floor = f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        f.Physics.RemoveBody(floor.Native);
        enemies.Update(0);
        Check(!SpiderPatrol.HasPursuitTargetSurface, "removed body must not survive in cache");
    }

    private static void DeadPlayer()
    {
        using var f = new Fixture();
        f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        player.TakeDamage(player.MaxHealth);
        enemies.Update(0);
        Check(player.IsDead && !SpiderPatrol.PursuitEnabled && !SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "dead player must clear physical target and stop raw-target pursuit");
        Near(SpiderPatrol.PursuitTarget, Vector3.Zero, "raw target must also be reset");
        SpiderPatrol.SetPursuitEnabled(true);
        enemies.Update(0);
        Check(!SpiderPatrol.PursuitEnabled && !SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "F3 cannot revive a dead player's stale target or either validity flag");
    }

    private static void ToggleReset()
    {
        using var f = new Fixture();
        f.Floor();
        var player = f.PlayerAt(new(0, 1.42f, 0));
        using var enemies = f.Pursuit(player);
        enemies.Update(0);
        SpiderPatrol.SetPursuitEnabled(false);
        enemies.Update(0);
        Check(!SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "F3 off must clear both target validity flags");
        player.NativeCharacter.Position = new(0, 2.42f, 0);
        f.Refresh(player.NativeCharacter);
        SpiderPatrol.SetPursuitEnabled(true);
        enemies.Update(0);
        Check(SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
            "re-enabling must publish a fresh raw target, not resurrect old surface cache");
        Near(SpiderPatrol.PursuitTarget, player.FeetPosition, "fresh raw target after F3 re-enable");
    }

    private static CharacterVirtual Character(Fixture f, Vector3 position)
    {
        using var shape = new CapsuleShape(0.9f, 0.5f);
        var settings = new CharacterVirtualSettings
        {
            Shape = shape, InnerBodyShape = shape, InnerBodyLayer = f.Physics.ObjectLayer,
            MaxSlopeAngle = float.DegreesToRadians(45), CharacterPadding = 0.02f,
            PredictiveContactDistance = 0.05f
        };
        Quaternion rotation = Quaternion.Identity;
        return new CharacterVirtual(settings, in position, in rotation, 0, f.Physics.Native);
    }

    private static void Support(SpiderTargetSurface hit, RigidBody body, Vector3 point)
    {
        Check(hit.IsValid && hit.BodyId == body.Native && hit.Normal.Y > 0.9f,
            $"expected upward support {body.Native}, got {hit.BodyId} at {hit.Point}, normal={hit.Normal}");
        Near(hit.Point, point, "support point");
        Near(hit.Normal, Vector3.UnitY, "support normal");
        Check(float.IsFinite(hit.Distance), "support distance must be finite");
    }

    private static void Published(RigidBody body, Vector3 point)
    {
        Check(SpiderPatrol.PursuitEnabled && SpiderPatrol.HasPursuitTarget && SpiderPatrol.HasPursuitTargetSurface &&
            SpiderPatrol.PursuitTargetSurfaceBodyId == body.Native, "expected published physical target with both validity flags");
        Near(SpiderPatrol.PursuitTargetSurfacePoint, point, "published surface");
        Near(SpiderPatrol.PursuitTarget, point, "published target must be the physical point");
        Near(SpiderPatrol.PursuitTargetSurfaceNormal, Vector3.UnitY, "published normal");
    }

    private static void Near(Vector3 actual, Vector3 expected, string message, float tolerance = 0.015f) =>
        Check(Vector3.Distance(actual, expected) <= tolerance, $"{message}: expected {expected}, got {actual}");

    private static void Check(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    private sealed class Fixture : IDisposable
    {
        public PhysicsWorld Physics { get; } = new(Vector3.Zero);
        public SceneManager Scene { get; }
        public SpiderTargetSurfaceResolver Resolver { get; }
        private readonly List<RigidBody> _bodies = [];
        private readonly List<GamePlayer> _players = [];

        public Fixture()
        {
            Scene = new SceneManager(Physics, null!);
            Resolver = new SpiderTargetSurfaceResolver(Scene);
        }

        public RigidBody Add(RigidBody body) { body.Build(Physics); _bodies.Add(body); return body; }
        public RigidBody Floor() => Add(new RigidBody().SetBox(new(10, 0.25f, 10)).SetPosition(new(0, -0.25f, 0)));
        public GamePlayer PlayerAt(Vector3 centre)
        {
            var player = new GamePlayer(Physics, centre);
            _players.Add(player);
            Refresh(player.NativeCharacter);
            return player;
        }
        public void Refresh(CharacterVirtual character)
        {
            using var bodies = new DefaultBodyFilter();
            using var shapes = new DefaultShapeFilter();
            ObjectLayer layer = Physics.ObjectLayer;
            character.RefreshContacts(in layer, Physics.Native, bodies, shapes);
            Vector3 velocity = character.LinearVelocity;
            character.LinearVelocity = -Vector3.UnitY;
            for (int frame = 0; frame < 15; frame++)
            {
                character.Update(1f / 60f, in layer, Physics.Native, bodies, shapes);
                if (character.GroundState == GroundState.OnGround)
                    break;
            }
            character.LinearVelocity = velocity;
        }
        public EnemySystem Pursuit(GamePlayer player)
        {
            var system = new EnemySystem(Physics, Scene, null!);
            system.SetPlayer(player);
            SpiderPatrol.SetPursuitEnabled(true);
            return system;
        }
        public void Dispose()
        {
            SpiderPatrol.SetPursuitEnabled(false);
            SpiderPatrol.ClearPursuitTarget();
            foreach (var player in _players) player.Dispose();
            foreach (var body in _bodies) { Physics.DestroyBody(body.Native); body.Destroy(); }
            Physics.Dispose();
        }
    }
}
