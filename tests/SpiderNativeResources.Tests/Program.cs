using System.Numerics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Fuse.Animation;
using Fuse.Core;
using Fuse.Enemy;
using Fuse.Physics;
using Fuse.Renderer;
using Fuse.Scene;
using JoltPhysicsSharp;
using AnimationSkeleton = Fuse.Animation.Skeleton;

internal static class Program
{
    private const BindingFlags PrivateInstance = BindingFlags.Instance | BindingFlags.NonPublic;
    private static readonly PropertyInfo Ownership = typeof(NativeObject).GetProperty("OwnsHandle", PrivateInstance)!;

    public static int Main()
    {
        (string Name, Action Run)[] tests =
        [
            ("Filter ownership and callback userdata release (512 disposals)", FilterOwnership),
            ("Owned body settings and borrowed BodyLock/callback wrappers", SettingsAndBorrowedBodies),
            ("Solver cache ownership, idempotent disposal and post-disposal query guards", SolverCacheDisposal),
            ("Navigation/motor/gait ray reserves share one capped counter", RaycastPhaseReserves),
            ("Disabled patrol resets exhausted budgets and maintains floor support", DisabledPatrolBudgets),
            ("Exhausted capacity, safe SpawnSpider/proxy failure and retry", ExhaustedCapacity),
            ("EnemySystem.Clear unregisters movement/damage bodies", ClearRegistrations),
            ("Death unregisters live bodies without removing the ragdoll entity", DeathRegistrations),
            ("Scene.Remove unregisters an already-invalidated body", RemoveInvalidatedBody)
        ];

        int failures = 0;
        foreach (var (name, run) in tests)
        {
            try
            {
                run();
                Console.WriteLine($"PASS {name}");
            }
            catch (Exception exception)
            {
                failures++;
                Console.Error.WriteLine($"FAIL {name}: {exception}");
            }
        }

        Console.WriteLine($"RESULT {tests.Length - failures}/{tests.Length} passed; no GL context or GPU assets created.");
        return failures == 0 ? 0 : 1;
    }

    private static void FilterOwnership()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var ignored = new HashSet<BodyID>();
        Func<NativeObject>[] factories =
        [
            () => new DefaultBroadPhaseLayerFilter(),
            () => new DefaultObjectLayerFilter(),
            () => new DefaultBodyFilter(),
            () => new DefaultShapeFilter(),
            () => new EnemyBodyFilter(BodyID.Invalid),
            () => new EnemyBodyFilter(BodyID.Invalid, ignored),
            () => new EnemyBodyFilter(ignored),
            () => new SurfaceBodyFilter(BodyID.Invalid, ignored, ignored)
        ];
        var disposedFilters = new List<NativeObject>();
        var userdata = new List<WeakReference>();
        try
        {
            for (int iteration = 0; iteration < 64; iteration++)
            {
                foreach (var create in factories)
                {
                    NativeObject filter = create();
                    disposedFilters.Add(filter);
                    Check(Owns(filter), $"{filter.GetType().Name} must own its native handle");
                    Check(filter.Handle != 0, "filter must have a native handle before disposal");
                    userdata.Add(TrackUserdata(filter));
                    filter.Dispose();
                    filter.Dispose();
                    Check(filter.IsDisposed && filter.Handle == 0, "filter disposal must be idempotent");
                }
            }

            Collect();
            Check(userdata.All(reference => !reference.IsAlive),
                "disposed filters must free the GCHandles rooting their userdata delegates");
            GC.KeepAlive(disposedFilters);
            Console.WriteLine($"  Released callback userdata: {userdata.Count}/{userdata.Count}");
        }
        finally
        {
            foreach (NativeObject filter in disposedFilters)
                filter.Dispose();
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference TrackUserdata(NativeObject filter)
    {
        Type bindingType = filter.GetType().BaseType!;
        FieldInfo field = bindingType.GetField("_listenerUserData", PrivateInstance)!;
        nint context = (nint)field.GetValue(filter)!;
        object target = GCHandle.FromIntPtr(context).Target!;
        Check(target != null, "live filter userdata must be rooted");
        return new WeakReference(target);
    }

    private static void SettingsAndBorrowedBodies()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        using (var shape = new SphereShape(0.25f))
        {
            MethodInfo factory = typeof(RigidBody).GetMethod("CreateOwnedBodySettings", BindingFlags.Static | BindingFlags.NonPublic)!;
            using var settings = (BodyCreationSettings)factory.Invoke(null,
                [shape, Vector3.Zero, Quaternion.Identity, MotionType.Static, (ObjectLayer)0])!;
            Check(Owns(settings) && settings.Handle != 0, "production body settings must own their native allocation");
            settings.Dispose();
            settings.Dispose();
            Check(settings.IsDisposed && settings.Handle == 0, "settings disposal must be idempotent");
            Check(!shape.IsDisposed && shape.Radius == 0.25f, "settings must not dispose the caller's shape wrapper");
            Console.WriteLine($"  Settings workaround: owns={Owns(settings)}; disposed={settings.IsDisposed}; handle={settings.Handle}; caller shape radius={shape.Radius}.");
        }

        var body = new RigidBody().SetBox(Vector3.One).SetMass(0);
        body.Build(physics);
        try
        {
            Check(body.IsBuilt && body.Native.IsValid, "body must survive disposal of creation settings");
            var scene = new SceneManager(physics, null!);
            physics.BodyLockInterface.LockRead(body.Native, out BodyLockRead readLock);
            Check(readLock.Succeeded, "body lock must succeed");
            Body borrowed;
            try
            {
                borrowed = readLock.Body!;
                Check(!Owns(borrowed), "BodyLock.Body must remain borrowed, not owned");
            }
            finally
            {
                physics.BodyLockInterface.UnlockRead(readLock);
            }

            for (int i = 0; i < 64; i++)
            {
                Check(scene.Raycast(new Vector3(0, 3, 0), -Vector3.UnitY, 6f, out var hit,
                    collideWithBackFaces: true, walkableOnly: true) && hit.BodyID == body.Native,
                    "raycast callbacks must survive temporary filter/settings disposal");
            }
            Check(!borrowed.IsDisposed, "filter callbacks must not dispose borrowed Body wrappers");
            Check(body.Position(physics) == Vector3.Zero, "native body must remain usable");
            GC.KeepAlive(borrowed);
        }
        finally
        {
            Destroy(physics, body);
        }
    }

    private static void SolverCacheDisposal()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var scene = new SceneManager(physics, null!);
        using var solver = new SpiderSurfaceSolver(scene);
        var cache = GetField<Dictionary<(int Length, int Radius), Shape>>(solver, "_segmentShapes");
        var order = GetField<Queue<(int Length, int Radius)>>(solver, "_shapeOrder");
        Check(cache.Count == 0 && order.Count == 0, "solver segment-shape cache must start empty");

        Vector3 a = new(0, 3, 0), b = new(0, 4, 0);
        solver.BeginGait();
        Check(solver.IsSegmentClear(a, b, 0.05f, BodyID.Invalid), "segment query in an empty world must succeed");
        Check(cache.Count == 1 && cache.Values.Single() is CapsuleShape,
            "the first segment query must populate the capsule cache");
        Shape capsule = cache.Values.Single();
        var filters = new NativeObject[]
        {
            GetField<DefaultBroadPhaseLayerFilter>(solver, "_queryBroadPhase"),
            GetField<DefaultObjectLayerFilter>(solver, "_queryObjectLayer"),
            GetField<DefaultShapeFilter>(solver, "_queryShapeFilter"),
            GetField<SurfaceBodyFilter>(solver, "_queryBodyFilter")
        };
        Check(filters.All(filter => Owns(filter) && filter.Handle != 0 && !filter.IsDisposed),
            "all cached query filters must own live native handles");
        WeakReference[] userdata = filters.Select(TrackUserdata).ToArray();

        solver.BeginFrame();
        solver.BeginGait();
        Check(solver.IsSegmentClear(a, b, 0.05f, BodyID.Invalid), "the cached capsule must remain usable across update phases");
        Check(cache.Count == 1 && ReferenceEquals(cache.Values.Single(), capsule),
            "an identical segment query must reuse its cached shape");
        Check(ReferenceEquals(GetField<SurfaceBodyFilter>(solver, "_queryBodyFilter"), filters[3]),
            "queries for the same self body must reuse the cached body filter");
        Check(solver.IsSegmentClear(a, a, 0.05f, BodyID.Invalid), "a zero-length segment must support a sphere query");
        Shape[] shapes = cache.Values.ToArray();
        Check(shapes.Length == 2 && shapes.Any(shape => shape is SphereShape) && order.Count == 2,
            "capsule and sphere queries must populate both cache entries and the eviction queue");
        Check(shapes.All(shape => Owns(shape) && shape.Handle != 0 && !shape.IsDisposed),
            "cached segment shapes must own live native handles");

        solver.Dispose();
        solver.Dispose();
        Check(cache.Count == 0 && order.Count == 0, "Dispose must empty the shape cache and eviction queue");
        Check(shapes.Concat(filters).All(resource => resource.IsDisposed && resource.Handle == 0),
            "Dispose must release every cached shape/filter native handle exactly once");
        Collect();
        Check(userdata.All(reference => !reference.IsAlive), "solver disposal must release cached-filter callback userdata");

        // A null physics dependency makes accidental world access fail managed,
        // rather than risking a native call through an already-disposed world.
        SetField(scene, "_physics", null!);
        Check(scene.Physics == null, "post-disposal query sentinel must have no native physics world");
        solver.BeginFrame();
        solver.BeginGait();
        Check(!solver.HasShapeQueryBudget && !solver.HasRaycastBudget,
            "update phase resets must not reactivate a disposed solver");
        Check(!solver.IsSegmentClear(a, b, 0.05f, BodyID.Invalid), "disposed overlap queries must fail closed");
        Check(!solver.IsSegmentMotionClear(a, b, a + Vector3.UnitX, b + Vector3.UnitX, 0.05f, BodyID.Invalid),
            "disposed segment motion queries must fail closed");
        Check(solver.ShapeTravelFraction(capsule, Matrix4x4.Identity, Vector3.UnitX, BodyID.Invalid) == 0f,
            "disposed shape casts must reject even an already-disposed cached shape without accessing native code");
        Check(!solver.TryFindSupportContact(a, Vector3.UnitY, Vector3.UnitZ, 1f, BodyID.Invalid, out var contact) && !contact.IsValid,
            "disposed support probes must fail without accessing the physics world");
        Check(solver.FrameShapeQueries == 0 && solver.FrameRaycasts == 0 && solver.QueryBudgetExhausted,
            "disposed queries must not consume native-query counters");
        Check(cache.Count == 0 && order.Count == 0 &&
            ReferenceEquals(GetField<SurfaceBodyFilter>(solver, "_queryBodyFilter"), filters[3]),
            "post-disposal queries must not recreate cached resources");
        solver.Dispose();
        GC.KeepAlive(shapes);
        GC.KeepAlive(filters);
        Console.WriteLine($"  Solver cache: {shapes.Length} shapes + {filters.Length} filters disposed; {userdata.Length}/{userdata.Length} userdatas released; 4 query paths rejected with no physics world.");
    }

    private static void RaycastPhaseReserves()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        using var solver = new SpiderSurfaceSolver(new SceneManager(physics, null!));
        Func<bool> takeRaycast = typeof(SpiderSurfaceSolver).GetMethod("TakeRaycast", PrivateInstance)!
            .CreateDelegate<Func<bool>>(solver);
        int cap = solver.Profile.MaximumRaycastsPerUpdate;
        int navigationLimit = Math.Max(1, cap / 3);
        int motorLimit = Math.Max(1, cap * 2 / 3);
        Check(navigationLimit < motorLimit && motorLimit < cap, "default ray budget must have three distinct phase reserves");

        void ExhaustTo(int expectedLimit, string phase)
        {
            int before = solver.FrameRaycasts;
            for (int count = before; count < expectedLimit; count++)
                Check(takeRaycast(), $"{phase} must allow ray {count + 1} up to its cumulative reserve");
            Check(solver.FrameRaycasts == expectedLimit, $"{phase} must stop at its exact cumulative limit");
            Check(!takeRaycast() && !solver.HasRaycastBudget && solver.QueryBudgetExhausted,
                $"{phase} must reject the first ray beyond its reserve");
            Check(solver.FrameRaycasts == expectedLimit && solver.FrameRaycasts <= cap,
                $"{phase} rejection must not increment the shared counter or exceed the update cap");
        }

        solver.BeginFrame();
        Check(solver.FrameRaycasts == 0 && !solver.QueryBudgetExhausted, "BeginFrame must start a fresh navigation budget");
        ExhaustTo(navigationLimit, "navigation");
        solver.BeginMotor();
        Check(solver.FrameRaycasts == navigationLimit && solver.HasRaycastBudget,
            "BeginMotor must reopen the motor reserve without resetting navigation's consumed rays");
        ExhaustTo(motorLimit, "motor");
        solver.BeginGait();
        Check(solver.FrameRaycasts == motorLimit && solver.HasRaycastBudget,
            "BeginGait must expose the remaining full-update reserve without resetting the counter");
        ExhaustTo(cap, "gait");
        solver.BeginGait();
        Check(!takeRaycast() && solver.FrameRaycasts == cap,
            "re-entering gait must not bypass the full update cap");
        Console.WriteLine($"  Ray reserves: navigation={navigationLimit}, motor={motorLimit}, gait={cap}; transitions preserve the counter; cap={cap} enforced.");
    }

    private static void DisabledPatrolBudgets()
    {
        bool previousEnabled = SpiderPatrol.Enabled;
        using var physics = new PhysicsWorld(new Vector3(0, -9.81f, 0));
        var scene = new SceneManager(physics, null!);
        var floor = new RigidBody().SetBox(new Vector3(50, 0.5f, 50)).SetPosition(new Vector3(0, -0.5f, 0)).SetMass(0);
        using var enemy = new SpiderEnemy("disabled-patrol-probe");
        try
        {
            floor.Build(physics);
            Check(floor.IsBuilt, "headless floor must have a native body");
            SpiderLocomotionProfile profile = SpiderLocomotionProfile.Default;
            Vector3 spawn = Vector3.UnitY * profile.BodyClearance;
            enemy.Body.SetCapsule(profile.BodyRadius, profile.BodyCylinderHeight)
                .SetPosition(spawn).SetKinematic(true).Build(physics);
            Check(enemy.Body.IsBuilt && enemy.Entity == null, "synthetic spider must have a body but no rendered entity or gait");
            scene.NonWalkableBodies.Add(enemy.Body.Native);
            using var solver = new SpiderSurfaceSolver(scene, profile);
            using var motor = new SpiderSurfaceMotor(physics, enemy.Body, solver, spawn, Vector3.UnitY, Vector3.UnitZ);
            var patrol = new SpiderPatrol(enemy, physics, motor, solver);
            Check(motor.HasSurface && motor.SurfaceContact.BodyId == floor.Native,
                "motor must acquire the floor before disabling patrol");
            Func<bool> takeRaycast = typeof(SpiderSurfaceSolver).GetMethod("TakeRaycast", PrivateInstance)!
                .CreateDelegate<Func<bool>>(solver);
            Func<bool> takeShapeQuery = typeof(SpiderSurfaceSolver).GetMethod("TakeShapeQuery", PrivateInstance)!
                .CreateDelegate<Func<bool>>(solver);
            int rayCap = profile.MaximumRaycastsPerUpdate;
            int shapeCap = profile.MaximumShapeQueriesPerUpdate;
            int motorRayLimit = Math.Max(1, rayCap * 2 / 3);
            int motorShapeLimit = Math.Max(1, shapeCap / 3);
            int minRays = int.MaxValue, maxRays = 0, minShapes = int.MaxValue, maxShapes = 0;
            const int updates = 120;
            const float dt = 1f / 60f;
            SpiderPatrol.Enabled = false;

            for (int frame = 0; frame < updates; frame++)
            {
                // Seed prior work without a fake gait. Only patrol.Update is
                // allowed to prepare/reset this frame and run the actual motor.
                solver.BeginGait();
                for (int count = solver.FrameRaycasts; count < rayCap; count++)
                    Check(takeRaycast(), $"frame {frame}: ray preconditioning must reach the full cap");
                for (int count = solver.FrameShapeQueries; count < shapeCap; count++)
                    Check(takeShapeQuery(), $"frame {frame}: shape preconditioning must reach the full cap");
                Check(!takeRaycast() && !takeShapeQuery() && solver.QueryBudgetExhausted,
                    $"frame {frame}: both budgets must be exhausted before the disabled update");
                SetField(patrol, "_commandSpeed", 3f);

                physics.Step(dt);
                patrol.Update(dt);

                Check(solver.FrameRaycasts > 0 && solver.FrameRaycasts <= motorRayLimit && solver.FrameRaycasts < rayCap,
                    $"frame {frame}: disabled patrol must reset rays and run fresh motor support queries, got {solver.FrameRaycasts}");
                Check(solver.FrameShapeQueries <= motorShapeLimit && solver.FrameShapeQueries < shapeCap,
                    $"frame {frame}: disabled patrol must reset shape queries, got {solver.FrameShapeQueries}");
                Check(!solver.QueryBudgetExhausted && solver.HasRaycastBudget && solver.HasShapeQueryBudget,
                    $"frame {frame}: previous exhaustion must not remain latched after disabled patrol.Update");
                Check(GetField<int>(solver, "_raycastLimit") == motorRayLimit && !GetField<bool>(motor, "_updatePrepared"),
                    $"frame {frame}: the motor must consume preparation and enable its two-thirds ray reserve");
                Check(motor.HasSurface && patrol.HasSurface && motor.SurfaceContact.IsValid &&
                    motor.SurfaceContact.BodyId == floor.Native && GetField<float>(motor, "_lostSurfaceTime") == 0f,
                    $"frame {frame}: disabled patrol must renew real floor support, not merely retain grace-period state");
                Check(GetField<float>(patrol, "_commandSpeed") == 0f && motor.DesiredRelativeVelocity == Vector3.Zero,
                    $"frame {frame}: disabled patrol must request zero movement speed");
                Check(patrol.CurrentSpeed == motor.CurrentSpeed && patrol.CurrentVelocity == motor.CurrentVelocity,
                    $"frame {frame}: disabled patrol must publish the freshly updated motor state");
                minRays = Math.Min(minRays, solver.FrameRaycasts);
                maxRays = Math.Max(maxRays, solver.FrameRaycasts);
                minShapes = Math.Min(minShapes, solver.FrameShapeQueries);
                maxShapes = Math.Max(maxShapes, solver.FrameShapeQueries);
            }
            Console.WriteLine($"  Disabled patrol: {updates} updates; exhausted {rayCap}-ray/{shapeCap}-shape budgets reset each frame; fresh rays={minRays}..{maxRays}, shapes={minShapes}..{maxShapes}; floor support maintained with zero grace time.");
        }
        finally
        {
            SpiderPatrol.Enabled = previousEnabled;
            scene.NonWalkableBodies.Remove(enemy.Body.Native);
            Destroy(physics, enemy.Body);
            Destroy(physics, floor);
        }
    }

    private static void ExhaustedCapacity()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var scene = new SceneManager(physics, null!);
        using var enemies = new EnemySystem(physics, scene, null!);
        var bodies = new List<RigidBody>();
        var overflow = new RigidBody().SetSphere(0.1f).SetMass(0);
        using var spider = new SpiderEnemy("capacity-probe");
        try
        {
            Check(!overflow.IsBuilt && !overflow.Native.IsValid, "new wrappers must start with an invalid ID");
            for (int i = 0; i < 1024; i++)
            {
                var body = new RigidBody().SetSphere(0.1f).SetMass(0).SetPosition(new Vector3(i * 2f, 0, 0));
                bodies.Add(body);
                body.Build(physics);
                Check(body.IsBuilt && body.Native.IsValid, $"pool body {i + 1} must be created");
            }

            overflow.Build(physics);
            Check(!overflow.IsBuilt && !overflow.Native.IsValid, "the 1025th body must fail without publishing a valid wrapper");
            Check(typeof(RigidBody).GetField("_shape", PrivateInstance)!.GetValue(overflow) == null,
                "failed creation must release its owned shape");
            Check(Logger.GetRecentLogs(1).Any(entry => entry.Level == LogLevel.Error && entry.Message.Contains("capacity")),
                "failed creation must report a useful error");
            overflow.Destroy();
            overflow.Destroy();

            try
            {
                spider.Initialize(physics, scene, Vector3.Zero, Vector3.UnitY);
                throw new Exception("expected a managed movement-body creation failure");
            }
            catch (InvalidOperationException exception)
            {
                Check(exception.Message.Contains("capacity-probe") && exception.Message.Contains("movement body"),
                    "spider must fail at the new guard, before assets/GL are accessed");
            }
            Check(!spider.Body.IsBuilt && !spider.Body.Native.IsValid, "failed spider must remain unbuilt");
            Check(scene.ActiveScene.Entities.Count == 0 && scene.NonWalkableBodies.Count == 0,
                "failed initialization must not register an entity or movement body");

            uint bodyCountBeforeSpawn = physics.Native.BodiesCount;
            Check(bodyCountBeforeSpawn == 1024, "the native pool must be full before SpawnSpider");
            for (int attempt = 0; attempt < 2; attempt++)
            {
                Check(enemies.SpawnSpider(Vector3.Zero, surfaceNormal: Vector3.UnitY) == null,
                    "SpawnSpider must return null instead of throwing when movement-body creation fails");
                Check(physics.Native.BodiesCount == bodyCountBeforeSpawn,
                    "a failed SpawnSpider must not add or remove native bodies");
                Check(enemies.GetSpiders().Count == 0 && scene.ActiveScene.Entities.Count == 0 && scene.NonWalkableBodies.Count == 0,
                    "a failed SpawnSpider must not register a spider, entity or movement body");
                Check(GetField<Dictionary<BodyID, SpiderEnemy>>(enemies, "_spiderDamageBodies").Count == 0 &&
                    enemies.GetSpiderMovementBodiesForDamageRaycast().Count == 0,
                    "a failed SpawnSpider must not leave sensor registrations or raycast exclusions");
            }
            Console.WriteLine($"  SpawnSpider: 2 exhausted-pool attempts returned null; native bodies unchanged at {bodyCountBeforeSpawn}; no entity/spider registrations.");

            var skeleton = Skeleton();
            using (var damage = new SpiderDamageBody(physics, skeleton, Vector3.Zero, Quaternion.Identity, Vector3.One, Definition()))
            {
                Check(!damage.IsBuilt && damage.PartCount == 0 && damage.BodyIds.Count == 0,
                    "exhausted damage proxy must not register invalid IDs");
                damage.SyncFromSkeleton(skeleton, Vector3.One, Quaternion.Identity, Vector3.One);
                damage.Dispose();
                damage.Dispose();
            }

            Destroy(physics, bodies[0]);
            overflow.Build(physics);
            Check(overflow.IsBuilt && overflow.Native.IsValid, "failed build must be retryable after freeing a slot");
            Console.WriteLine("  Pool: 1024 successful bodies; overflow/proxy/spider rejected safely; retry succeeded.");
        }
        finally
        {
            Destroy(physics, overflow);
            foreach (RigidBody body in bodies)
                Destroy(physics, body);
        }
    }

    private static void ClearRegistrations()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var scene = new SceneManager(physics, null!);
        using var enemies = new EnemySystem(physics, scene, null!);
        SpiderEnemy spider = AddSpider(physics, scene, enemies);
        BodyID movement = spider.Body.Native;
        SpiderDamageBody damage = spider.DamageBody!;
        BodyID sensor = damage.BodyIds[0];
        IReadOnlySet<BodyID> excluded = enemies.GetSpiderMovementBodiesForDamageRaycast();
        Check(excluded.Contains(movement) && enemies.TryGetEnemy(sensor, out _), "live bodies must be registered first");

        enemies.Clear();
        enemies.Clear();
        Check(!spider.Body.IsBuilt && !damage.IsBuilt, "clear must destroy movement and damage bodies");
        Check(!scene.NonWalkableBodies.Contains(movement), "clear must remove NonWalkableBodies before invalidating the wrapper");
        Check(scene.ActiveScene.GetEntityByBody(movement) == null, "clear must remove the body/entity mapping");
        Check(scene.ActiveScene.Entities.Count == 0 && enemies.GetSpiders().Count == 0, "clear must remove the entity and spider");
        Check(!enemies.TryGetEnemy(sensor, out _) && excluded.Count == 0, "clear must remove sensor lookup and cached exclusions");
    }

    private static void DeathRegistrations()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var scene = new SceneManager(physics, null!);
        using var enemies = new EnemySystem(physics, scene, null!);
        SpiderEnemy spider = AddSpider(physics, scene, enemies);
        BodyID movement = spider.Body.Native;
        BodyID sensor = spider.DamageBody!.BodyIds[0];
        spider.DeathRagdollDefinition.LifetimeSeconds = 0.1f;
        spider.Health = 0;

        enemies.Update(0.01f);
        Check(spider.IsDeathRagdollActive && spider.Entity.Visible, "death must preserve the visible ragdoll entity");
        Check(scene.ActiveScene.Entities.Contains(spider.Entity), "death must not prematurely remove the entity");
        Check(!spider.Body.IsBuilt && spider.DamageBody == null, "death must destroy the live movement/damage bodies");
        Check(!scene.NonWalkableBodies.Contains(movement) && scene.ActiveScene.GetEntityByBody(movement) == null,
            "death must unregister the old movement ID immediately");
        Check(!enemies.TryGetEnemy(sensor, out _), "death must unregister live sensor lookup");

        physics.Step(1f / 60f);
        enemies.Update(0.2f);
        Check(enemies.GetSpiders().Count == 0 && scene.ActiveScene.Entities.Count == 0,
            "ragdoll lifetime expiry must complete cleanup");
    }

    private static void RemoveInvalidatedBody()
    {
        using var physics = new PhysicsWorld(Vector3.Zero);
        var scene = new SceneManager(physics, null!);
        var body = new RigidBody().SetSphere(0.1f).SetMass(0);
        body.Build(physics);
        Entity entity = scene.ActiveScene.Add(null, "invalidated-probe", body);
        scene.ActiveScene.RegisterBody(entity);
        BodyID id = body.Native;
        Destroy(physics, body);
        scene.ActiveScene.Remove(entity);
        scene.ActiveScene.Remove(entity);
        Check(scene.ActiveScene.GetEntityByBody(id) == null && scene.ActiveScene.Entities.Count == 0,
            "Remove must clear stale registration even after Body.Destroy");
    }

    private static SpiderEnemy AddSpider(PhysicsWorld physics, SceneManager scene, EnemySystem enemies)
    {
        var spider = new SpiderEnemy("lifecycle-probe");
        spider.Body.SetSphere(0.25f).SetKinematic(true).Build(physics);
        Check(spider.Body.IsBuilt, "synthetic movement body must be created");
        Entity entity = scene.ActiveScene.Add(null, spider.Id, spider.Body);
        typeof(SpiderEnemy).GetProperty(nameof(SpiderEnemy.Entity))!.SetValue(spider, entity);
        SetField(spider, "_sceneManager", scene);
        SetField(spider, "_physics", physics);
        SetField(spider, "_initialized", true);
        var damage = new SpiderDamageBody(physics, Skeleton(), Vector3.Zero, Quaternion.Identity, Vector3.One, Definition());
        SetField(spider, "_damageBody", damage);
        scene.ActiveScene.RegisterBody(entity);
        scene.NonWalkableBodies.Add(spider.Body.Native);
        ((List<SpiderEnemy>)typeof(EnemySystem).GetField("_spiders", PrivateInstance)!.GetValue(enemies)!).Add(spider);
        typeof(EnemySystem).GetMethod("RegisterSpiderDamageBodies", PrivateInstance)!.Invoke(enemies, [spider]);
        return spider;
    }

    private static AnimationSkeleton Skeleton() => new(
        [new AnimationNode { Name = "root", RestLocal = Matrix4x4.Identity }],
        new Dictionary<string, int> { ["root"] = 0 }, [], Matrix4x4.Identity);

    private static SpiderRagdollDefinition Definition()
    {
        var definition = new SpiderRagdollDefinition { Name = "synthetic-proxy" };
        definition.Parts.Add(new SpiderRagdollPartDefinition { Id = "Body", BoneName = "root", Radius = 0.1f, Height = 0.25f });
        return definition;
    }

    private static bool Owns(NativeObject value) => (bool)Ownership.GetValue(value)!;
    private static T GetField<T>(object target, string name) =>
        (T)target.GetType().GetField(name, PrivateInstance)!.GetValue(target)!;
    private static void SetField(object target, string name, object value) =>
        target.GetType().GetField(name, PrivateInstance)!.SetValue(target, value);

    private static void Destroy(PhysicsWorld physics, RigidBody body)
    {
        if (body.IsBuilt)
        {
            Check(body.Native.IsValid, "never destroy BodyID.Invalid through the native interface");
            physics.DestroyBody(body.Native);
        }
        body.Destroy();
    }

    private static void Collect()
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
    }

    private static void Check(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}
