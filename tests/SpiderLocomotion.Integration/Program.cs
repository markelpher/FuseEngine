using System.Collections;
using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Reflection;
using System.Security.Cryptography;
using System.Text.Json;
using Fuse.Animation;
using Fuse.Enemy;
using Fuse.Physics;
using Fuse.Renderer;
using Fuse.Scene;

internal static class Program
{
    private const float Dt = 1f / 60f;
    private const BindingFlags Fields = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };

    public static int Main(string[] args)
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        try
        {
            string root = FindRoot();
            string selected = Option(args, "--case", "floor");
            float speed = float.Parse(Option(args, "--speed", "9.5"), CultureInfo.InvariantCulture);
            int frames = int.Parse(Option(args, "--frames", "240"), CultureInfo.InvariantCulture);
            int maximumFrames = selected == "blocked" ? 3600 : 300;
            if (!float.IsFinite(speed) || speed <= 0 || frames < 180 || frames > maximumFrames)
                throw new ArgumentException("Use positive finite --speed and --frames 180..300, or 180..3600 only with --case blocked.");
            double maximumUpdateMs = double.Parse(Option(args, "--max-update-ms", "250"), CultureInfo.InvariantCulture);
            string p95Option = Option(args, "--max-p95-ms", "");
            double? maximumP95Ms = p95Option.Length == 0 ? null : double.Parse(p95Option, CultureInfo.InvariantCulture);
            if (!double.IsFinite(maximumUpdateMs) || maximumUpdateMs <= 0 ||
                (maximumP95Ms.HasValue && (!double.IsFinite(maximumP95Ms.Value) || maximumP95Ms.Value <= 0)))
                throw new ArgumentException("Timing limits must be positive finite milliseconds.");
            bool privateMemory = args.Contains("--private-memory");
            string output = Path.GetFullPath(Option(args, "--output", Path.Combine(root, "tests", "SpiderLocomotion.Integration", "results")));
            Directory.CreateDirectory(output);
            string rigPath = Path.Combine(root, "Fuse", "res", "skinned_models", "spider.fbx");
            Console.WriteLine($"HEADLESS rig={rigPath} sha256={Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(rigPath)))}");
            string assembly = typeof(SpiderSurfaceMotor).Assembly.Location;
            Console.WriteLine($"assembly={assembly} utc={DateTime.UtcNow:O}");
            string[] sourcePaths = ["Fuse/src/Animation/ProceduralSpiderWalk.cs", "Fuse/src/Animation/SpiderLegKinematics.cs",
                "Fuse/src/Enemy/SpiderSurfaceMotor.cs", "Fuse/src/Enemy/SpiderSurfaceSolver.cs", "Fuse/src/Enemy/SpiderLocomotionProfile.cs",
                "Fuse/src/Enemy/SpiderLocomotionPose.cs", "Fuse/src/Renderer/SkinnedModelLoader.cs",
                "tests/SpiderLocomotion.Integration/Program.cs"];
            var sourceHashes = sourcePaths.ToDictionary(p => p, p => Hash(Path.Combine(root, p)));
            File.WriteAllText(Path.Combine(output, "environment.json"), JsonSerializer.Serialize(new
            {
                Utc = DateTime.UtcNow, AssemblySha256 = Hash(assembly), RigSha256 = Hash(rigPath),
                AssemblyMvid = typeof(SpiderSurfaceMotor).Module.ModuleVersionId, SourceHashes = sourceHashes,
                HarnessAssemblySha256 = Hash(typeof(Program).Assembly.Location),
                HarnessAssemblyMvid = typeof(Program).Module.ModuleVersionId,
                Profile = SpiderLocomotionProfile.Default, Speed = speed, Frames = frames,
                FixedDeltaSeconds = Dt, SettlingFrames = 30, MaximumUpdateMilliseconds = maximumUpdateMs,
                MaximumP95Milliseconds = maximumP95Ms, PrivateMemoryEnabled = privateMemory,
                Runtime = System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription,
                OS = System.Runtime.InteropServices.RuntimeInformation.OSDescription,
                Architecture = System.Runtime.InteropServices.RuntimeInformation.ProcessArchitecture.ToString(),
                ProcessorCount = Environment.ProcessorCount, StopwatchFrequency = Stopwatch.Frequency,
#if DEBUG
                Configuration = "Debug"
#else
                Configuration = "Release"
#endif
            }, JsonOptions));
            var model = SkinnedModelLoader.LoadRig(rigPath) ?? throw new InvalidOperationException("LoadRig returned null.");
            Console.WriteLine($"RIG nodes={model.Skeleton.Nodes.Length} bones={model.Skeleton.Bones.Length}");
            var cases = new[]
            {
                new Scenario("floor", Vector3.UnitY, Vector3.UnitZ),
                new Scenario("wall", Vector3.UnitX, Vector3.UnitZ),
                new Scenario("ceiling", -Vector3.UnitY, Vector3.UnitZ),
                new Scenario("turn", Vector3.UnitY, Vector3.UnitZ, Motion.Turn),
                new Scenario("stop", Vector3.UnitY, Vector3.UnitZ, Motion.Stop),
                new Scenario("reverse", Vector3.UnitY, Vector3.UnitZ, Motion.Reverse),
                new Scenario("slow", Vector3.UnitY, Vector3.UnitZ, Speed: 2f),
                new Scenario("blocked", Vector3.UnitY, Vector3.UnitZ, Motion.Blocked)
            };
            var chosen = cases.Where(c => selected == "all" || selected == c.Name).ToArray();
            if (chosen.Length == 0) throw new ArgumentException("--case floor|wall|ceiling|turn|stop|reverse|slow|blocked|all");
            var results = new List<Result>();
            foreach (var scenario in chosen)
            {
                Result result = Run(scenario, model.Skeleton.CreateInstance(), scenario.Speed ?? speed, frames, output,
                    maximumUpdateMs, maximumP95Ms, privateMemory);
                results.Add(result);
                File.WriteAllText(Path.Combine(output, "summary.json"), JsonSerializer.Serialize(results, JsonOptions));
            }
            string[] changedSources = sourcePaths.Where(p => sourceHashes[p] != Hash(Path.Combine(root, p))).ToArray();
            File.WriteAllText(Path.Combine(output, "sources-changed-during-run.json"), JsonSerializer.Serialize(changedSources, JsonOptions));
            if (changedSources.Length > 0) Console.WriteLine($"WARNING sources changed during run: {string.Join(", ", changedSources)}; results describe the loaded assembly.");
            Console.WriteLine($"RESULT {(results.All(r => r.Failures.Count == 0) ? "PASS" : "FAIL")} reports={output}");
            return results.All(r => r.Failures.Count == 0) ? 0 : 1;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"HARNESS ERROR: {ex}");
            return 2;
        }
    }

    private static Result Run(Scenario scenario, Skeleton skeleton, float speed, int frames, string output,
        double maximumUpdateMs, double? maximumP95Ms, bool privateMemory)
    {
        var watch = Stopwatch.StartNew();
        var profile = SpiderLocomotionProfile.Default;
        using var physics = new PhysicsWorld(new Vector3(0, -9.81f, 0));
        var scene = new SceneManager(physics, null!);
        Vector3 halfExtents = scenario.Normal.X != 0 ? new(0.5f, 100, 100) : new(100, 0.5f, 100);
        var ground = new RigidBody().SetBox(halfExtents).SetPosition(-scenario.Normal * 0.5f).SetMass(0);
        ground.Build(physics);
        Vector3 spawn = scenario.Normal * profile.BodyClearance;
        var body = new RigidBody().SetCapsule(profile.BodyRadius, profile.BodyCylinderHeight)
            .SetPosition(spawn).SetKinematic(true).SetFriction(0.5f).SetRestitution(0.1f);
        body.Build(physics);
        scene.NonWalkableBodies.Add(body.Native);
        using var solver = new SpiderSurfaceSolver(scene, profile);
        using var motor = new SpiderSurfaceMotor(physics, body, solver, spawn, scenario.Normal, scenario.Forward);
        var gait = new ProceduralSpiderWalk(scene, solver, profile);
        Initialize(gait, skeleton, profile.VisualScale);
        var rejectionDiagnostics = new Dictionary<string, string>();
        var pendingDiagnostics = new Dictionary<string, (Vector3 Position, Quaternion Rotation)>();
        void FlushDiagnostics()
        {
            foreach (var entry in pendingDiagnostics)
                rejectionDiagnostics[entry.Key] = DescribeIkRejection(gait, entry.Value.Position, entry.Value.Rotation, entry.Key);
            pendingDiagnostics.Clear();
        }
        motor.BodyPoseValidator = (position, rotation) =>
        {
            bool accepted = gait.CanOccupyPose(position, rotation);
            string reason = gait.LastBodyBlockReason;
            if (!accepted && reason.Contains("reach/static clearance") && !rejectionDiagnostics.ContainsKey(reason))
                pendingDiagnostics.TryAdd(reason, (position, rotation));
            return accepted;
        };
        motor.BodyPoseFailureDescription = () => gait.LastBodyBlockReason;
        motor.PoseReset += gait.ResetContacts;
        int[] landings = new int[8];
        gait.FootLanded += (leg, _, _) => landings[leg]++;
        SpiderLocomotionPose Pose() => new(body.Position(physics), body.Rotation(physics),
            motor.CurrentVelocity, motor.CurrentAngularVelocity, profile.VisualScale, body.Native,
            motor.SurfaceContact, motor.SupportVelocity, motor.SupportAngularVelocity, motor.DesiredRelativeVelocity);

        var result = new Result { Case = scenario.Name, Frames = frames, Speed = speed };
        // Settle contacts before measuring; every frame still uses the production validator.
        for (int i = 0; i < 30; i++)
        {
            motor.Update(Dt, scenario.Forward, 0);
            FlushDiagnostics();
            gait.Update(Dt, Pose());
            physics.Step(Dt);
            CheckTetrads(result, Legs(gait), i - 30);
        }
        object[] legs = Legs(gait);
        result.Start = body.Position(physics);
        result.InitialPlanted = gait.PlantedLegCount;
        result.InitialValid = legs.Count(l => Field<bool>(l, "PoseValid"));
        Console.WriteLine($"BEGIN {scenario.Name} settled={V(result.Start)} planted={result.InitialPlanted} valid={result.InitialValid}");
        DumpLegs(gait, skeleton, Pose(), scenario.Normal, "initial");
        Array.Clear(landings);
        int[][] legNodes = legs.Select(l => new[] { Field<int>(l, "Hip"), Field<int>(l, "Knee"),
            Field<int>(l, "Ankle"), Field<int>(l, "Tip") }).ToArray();
        RigidBody? obstacle = null;
        float obstacleFront = 0;
        if (scenario.Motion == Motion.Blocked)
        {
            // Insert only after settling, beyond every current limb capsule. The broad, tall
            // box obstructs predicted landings without invalidating the initial stance.
            var initialPose = Pose();
            obstacleFront = result.Start.Z + profile.BodyRadius;
            for (int i = 0; i < legs.Length; i++)
                foreach (int node in legNodes[i])
                    obstacleFront = MathF.Max(obstacleFront,
                        initialPose.ToWorld(Point(skeleton.Nodes[node].Global)).Z + Field<float>(legs[i], "Radius"));
            obstacleFront += 0.05f;
            obstacle = new RigidBody().SetBox(new Vector3(20, 5, 5))
                .SetPosition(new Vector3(result.Start.X, 5, obstacleFront + 5)).SetMass(0);
            obstacle.Build(physics);
            // This is a blocked floor request, not a floor-to-wall transition test.
            motor.SetTransitionConstraint(scenario.Normal);
            result.ObstacleFrontZ = obstacleFront;
            Console.WriteLine($"OBSTACLE blocked frontZ={obstacleFront:F6} halfExtents=(20,5,5) insertedAfterSettle=true");
        }
        var shapeQueries = OptionalReader<int>(solver, "FrameShapeQueries");
        var raycasts = OptionalReader<int>(solver, "FrameRaycasts");
        var exhausted = OptionalReader<bool>(solver, "QueryBudgetExhausted");
        result.MaximumShapeQueriesPerUpdate = OptionalReader<int>(profile, "MaximumShapeQueriesPerUpdate")?.Invoke();
        result.MaximumRaycastsPerUpdate = OptionalReader<int>(profile, "MaximumRaycastsPerUpdate")?.Invoke();
        var samples = new UpdateSample[frames];
        int[]? shapeSamples = shapeQueries == null ? null : new int[frames];
        int[]? raySamples = raycasts == null ? null : new int[frames];
        if (exhausted != null) result.QueryBudgetExhaustedFrames = 0;
        result.MeasuredThreadId = Environment.CurrentManagedThreadId;
        float requestedDistance = 0;
        float pathLength = 0;
        float commandedPathLength = 0;
        float turnAngle = 0;
        Vector3 settledPosition = default;
        Vector3[] settledFeet = [];
        const int stopFrame = 90;
        const int settledFrame = stopFrame + 30;
        Vector3 previous = result.Start;
        using var csv = new StreamWriter(Path.Combine(output, scenario.Name + ".csv"));
        csv.WriteLine("frame,time,x,y,z,commandSpeed,actualSpeed,acceptedFraction,planted,swing,valid,blockedLegs,hasSurface,maxSkinResidual,maxPlantedResidual,bodyBlockReason,phase,swingMask,swingGroup,headingX,headingY,headingZ,motorMs,gaitMs,fixedUpdateMs,allocatedBytesCurrentThread,gcGen0,gcGen1,gcGen2,shapeQueries,raycasts,queryBudgetExhausted");
        if (privateMemory) result.PrivateMemoryBeforeBytes = PrivateMemoryBytes();
        for (int frame = 0; frame < frames; frame++)
        {
            float command = MathF.Min(speed, 8f * (frame + 1) * Dt);
            Vector3 direction = scenario.Forward;
            string phase = scenario.Motion == Motion.Blocked ? "blocked" : "walk";
            if (scenario.Motion == Motion.Turn && frame >= 60)
                direction = Vector3.Transform(scenario.Forward, Quaternion.CreateFromAxisAngle(scenario.Normal, MathF.PI / 2));
            if (scenario.Motion == Motion.Stop && frame >= stopFrame)
            {
                command = 0f;
                phase = frame < settledFrame ? "settling" : "stopped";
                if (frame == stopFrame) result.StopPosition = previous;
                if (frame == settledFrame)
                {
                    settledPosition = previous;
                    settledFeet = legs.Select(l => Pose().ToWorld(Point(skeleton.Nodes[Field<int>(l, "Tip")].Global))).ToArray();
                }
            }
            if (scenario.Motion == Motion.Reverse)
            {
                command = 0f;
                direction = -scenario.Forward;
                phase = "turn-in-place";
            }
            requestedDistance += command * Dt;
            var motorStart = Measurement.Start();
            motor.Update(Dt, direction, command);
            var motorCost = motorStart.Finish();
            // No reflection/IK diagnostics, Pose acquisition, physics or report work in
            // either measured window. Keep the diagnostic on the pre-gait state.
            FlushDiagnostics();
            var pose = Pose();
            var gaitStart = Measurement.Start();
            gait.Update(Dt, pose);
            var gaitCost = gaitStart.Finish();
            var sample = new UpdateSample(motorCost, gaitCost);
            samples[frame] = sample;
            if (shapeSamples != null) shapeSamples[frame] = shapeQueries!();
            if (raySamples != null) raySamples[frame] = raycasts!();
            bool? budgetExhausted = exhausted?.Invoke();
            if (budgetExhausted == true) result.QueryBudgetExhaustedFrames++;
            physics.Step(Dt);
            Vector3 current = body.Position(physics);
            float travel = Vector3.Distance(previous, current);
            pathLength += travel;
            if (command > 0f) commandedPathLength += travel;
            if (scenario.Motion == Motion.Reverse)
                result.MaximumStationaryDisplacement = MathF.Max(result.MaximumStationaryDisplacement, Vector3.Distance(result.Start, current));
            if (scenario.Motion == Motion.Stop && frame >= settledFrame)
            {
                result.StoppedObservationFrames++;
                result.MaximumStoppedBodyDrift = MathF.Max(result.MaximumStoppedBodyDrift, Vector3.Distance(settledPosition, current));
                result.StoppedPathLength += travel;
                result.MaximumStoppedSpeed = MathF.Max(result.MaximumStoppedSpeed, motor.CurrentVelocity.Length());
                if (gait.SwingLegCount > 0 || gait.PlantedLegCount != 8) result.UnsettledStopFrames++;
                for (int i = 0; i < legs.Length; i++)
                {
                    Vector3 actual = pose.ToWorld(Point(skeleton.Nodes[Field<int>(legs[i], "Tip")].Global));
                    result.MaximumStoppedFootDrift = MathF.Max(result.MaximumStoppedFootDrift, Vector3.Distance(settledFeet[i], actual));
                }
            }
            previous = current;
            var (swingMask, swingGroup) = CheckTetrads(result, legs, frame);
            result.MinimumPlanted = Math.Min(result.MinimumPlanted, gait.PlantedLegCount);
            int valid = legs.Count(l => Field<bool>(l, "PoseValid"));
            result.MinimumValid = Math.Min(result.MinimumValid, valid);
            result.MaximumSwing = Math.Max(result.MaximumSwing, gait.SwingLegCount);
            if (motor.IsBlocked) result.BlockedFrames++;
            if (!motor.HasSurface) result.SurfaceLostFrames++;
            result.MinimumAcceptedFraction = MathF.Min(result.MinimumAcceptedFraction, motor.AcceptedMotionFraction);
            result.AcceptedFractionSum += motor.AcceptedMotionFraction;
            result.MaximumClearanceError = MathF.Max(result.MaximumClearanceError,
                MathF.Abs(Vector3.Dot(current, scenario.Normal) - profile.BodyClearance));
            float skinResidual = 0, plantedResidual = 0;
            foreach (object leg in legs)
            {
                Vector3 actual = pose.ToWorld(Point(skeleton.Nodes[Field<int>(leg, "Tip")].Global));
                float residual = Vector3.Distance(actual, Field<Vector3>(leg, "Foot"));
                skinResidual = MathF.Max(skinResidual, residual);
                bool planted = Field<SpiderSurfaceContact>(leg, "Planted").IsValid && !Field<bool>(leg, "Stepping");
                if (planted) plantedResidual = MathF.Max(plantedResidual, residual);
                if (!Finite(actual) || !float.IsFinite(residual)) result.NonFiniteSamples++;
                if (planted)
                {
                    var contact = Field<SpiderSurfaceContact>(leg, "Planted");
                    bool onGround = contact.BodyId == ground.Native &&
                        MathF.Abs(Vector3.Dot(contact.Point, scenario.Normal)) <= profile.ContactToleranceWorld &&
                        Vector3.Dot(contact.Normal, scenario.Normal) >= 0.98f;
                    bool onObstacle = obstacle != null && contact.BodyId == obstacle.Native &&
                        MathF.Abs(contact.Point.Z - obstacleFront) <= profile.ContactToleranceWorld &&
                        Vector3.Dot(contact.Normal, -Vector3.UnitZ) >= 0.98f;
                    if (onObstacle) result.ObstacleContactSamples++;
                    if (!Finite(contact.Point) || !Finite(contact.Normal) || (!onGround && !onObstacle)) result.InvalidContactSamples++;
                }
                string failure = Field<string>(leg, "Failure");
                if (failure.Length > 0) Increment(result.LegFailures, $"{Field<int>(leg, "Index")}: {failure}");
                if (failure == "no collision-free step to a reachable support") result.RejectedLandingSamples++;
            }
            result.MaximumSkinResidual = MathF.Max(result.MaximumSkinResidual, skinResidual);
            result.MaximumPlantedResidual = MathF.Max(result.MaximumPlantedResidual, plantedResidual);
            if (!string.IsNullOrEmpty(motor.LastBodyBlockReason)) Increment(result.BodyFailures, motor.LastBodyBlockReason);
            if (!Finite(current) || !Finite(pose.Up) || !Finite(pose.Forward) || !Finite(motor.CurrentVelocity) ||
                !Finite(motor.CurrentAngularVelocity) || !float.IsFinite(motor.CurrentSpeed) ||
                !float.IsFinite(motor.AcceptedMotionFraction)) result.NonFiniteSamples++;
            if (obstacle != null)
            {
                // All capsule endpoints must remain behind the front plane, so no limb
                // segment (or body) can tunnel through the box between its joints.
                var physicalPose = Pose();
                float penetration = current.Z + profile.BodyRadius +
                    MathF.Abs(physicalPose.Up.Z) * profile.BodyCylinderHeight * 0.5f - obstacleFront;
                for (int i = 0; i < legs.Length; i++)
                    foreach (int node in legNodes[i])
                    {
                        Vector3 joint = physicalPose.ToWorld(Point(skeleton.Nodes[node].Global));
                        if (!Finite(joint)) result.NonFiniteSamples++;
                        float radius = Field<float>(legs[i], "Radius") * profile.LegRadiusFractionOfFoot;
                        if (node == legNodes[i][3]) radius = Field<float>(legs[i], "Radius");
                        penetration = MathF.Max(penetration, joint.Z + radius - obstacleFront);
                    }
                result.MaximumObstaclePenetration = MathF.Max(result.MaximumObstaclePenetration, penetration);
                if (penetration > profile.ContactToleranceWorld) result.ObstaclePenetrationFrames++;
            }
            turnAngle = float.RadiansToDegrees(MathF.Acos(Math.Clamp(Vector3.Dot(scenario.Forward, pose.Forward), -1, 1)));
            csv.WriteLine($"{frame},{(frame + 1) * Dt:F4},{current.X:R},{current.Y:R},{current.Z:R},{command:R},{motor.CurrentSpeed:R},{motor.AcceptedMotionFraction:R},{gait.PlantedLegCount},{gait.SwingLegCount},{valid},{gait.BlockedLegCount},{motor.HasSurface},{skinResidual:R},{plantedResidual:R},\"{motor.LastBodyBlockReason.Replace("\"", "\"\"")}\",{phase},{swingMask:X2},{swingGroup},{pose.Forward.X:R},{pose.Forward.Y:R},{pose.Forward.Z:R},{sample.Motor.Milliseconds:R},{sample.Gait.Milliseconds:R},{sample.Milliseconds:R},{sample.AllocatedBytes},{sample.Gen0},{sample.Gen1},{sample.Gen2},{shapeSamples?[frame]},{raySamples?[frame]},{budgetExhausted}");
            if ((frame + 1) % 60 == 0)
            {
                csv.Flush();
                Console.WriteLine($"SAMPLE {scenario.Name} t={(frame + 1) * Dt:F1}s pos={V(current)} forward={V(motor.Forward)} planted={gait.PlantedLegCount} swing={gait.SwingLegCount} valid={valid} blocked={result.BlockedFrames} residual={skinResidual:G5} reason={motor.LastBodyBlockReason}");
            }
        }
        if (privateMemory) result.PrivateMemoryAfterBytes = PrivateMemoryBytes();
        result.FixedUpdateMilliseconds = Statistics.Of(samples.Select(s => s.Milliseconds));
        result.MotorMilliseconds = Statistics.Of(samples.Select(s => s.Motor.Milliseconds));
        result.GaitMilliseconds = Statistics.Of(samples.Select(s => s.Gait.Milliseconds));
        result.AllocatedBytesCurrentThreadPerUpdate = Statistics.Of(samples.Select(s => (double)s.AllocatedBytes));
        result.GCCollections = [samples.Sum(s => s.Gen0), samples.Sum(s => s.Gen1), samples.Sum(s => s.Gen2)];
        result.UpdatesOverFixedDelta = samples.Count(s => s.Milliseconds > Dt * 1000d);
        if (shapeSamples != null) result.FrameShapeQueries = Statistics.Of(shapeSamples.Select(s => (double)s));
        if (raySamples != null) result.FrameRaycasts = Statistics.Of(raySamples.Select(s => (double)s));
        Check(result, result.FixedUpdateMilliseconds.Count == frames, $"Only {result.FixedUpdateMilliseconds.Count}/{frames} measured updates.");
        Check(result, result.FixedUpdateMilliseconds.Maximum <= maximumUpdateMs,
            $"Fixed update max {result.FixedUpdateMilliseconds.Maximum:F3} ms > responsiveness limit {maximumUpdateMs:F3} ms.");
        if (maximumP95Ms.HasValue)
            Check(result, result.FixedUpdateMilliseconds.P95 <= maximumP95Ms.Value,
                $"Fixed update p95 {result.FixedUpdateMilliseconds.P95:F3} ms > {maximumP95Ms.Value:F3} ms.");
        if (result.MaximumShapeQueriesPerUpdate.HasValue)
        {
            Check(result, result.MaximumShapeQueriesPerUpdate is > 0 and <= 512,
                $"Profile shape-query budget {result.MaximumShapeQueriesPerUpdate} outside conservative cap 1..512.");
            if (shapeSamples != null)
                Check(result, shapeSamples.All(n => n >= 0 && n <= result.MaximumShapeQueriesPerUpdate.Value),
                    $"Shape queries outside 0..{result.MaximumShapeQueriesPerUpdate}; max={result.FrameShapeQueries!.Maximum}.");
        }
        if (result.MaximumRaycastsPerUpdate.HasValue)
        {
            Check(result, result.MaximumRaycastsPerUpdate is > 0 and <= 192,
                $"Profile raycast budget {result.MaximumRaycastsPerUpdate} outside conservative cap 1..192.");
            if (raySamples != null)
                Check(result, raySamples.All(n => n >= 0 && n <= result.MaximumRaycastsPerUpdate.Value),
                    $"Raycasts outside 0..{result.MaximumRaycastsPerUpdate}; max={result.FrameRaycasts!.Maximum}.");
        }
        result.End = body.Position(physics);
        result.Displacement = Vector3.Distance(result.Start, result.End);
        result.ForwardDisplacement = Vector3.Dot(result.End - result.Start, scenario.Forward);
        result.PathLength = pathLength;
        result.RequestedDistance = requestedDistance;
        result.CommandedPathLength = commandedPathLength;
        result.MotionRatio = requestedDistance > 0 ? commandedPathLength / requestedDistance : null;
        result.TurnDegrees = turnAngle;
        Vector3 finalDirection = scenario.Motion switch
        {
            Motion.Turn => Vector3.Transform(scenario.Forward, Quaternion.CreateFromAxisAngle(scenario.Normal, MathF.PI / 2)),
            Motion.Reverse => -scenario.Forward,
            _ => scenario.Forward
        };
        result.FinalHeadingAlignment = Vector3.Dot(Pose().Forward, finalDirection);
        result.Landings = landings;
        result.RejectionDiagnostics = rejectionDiagnostics;
        result.MeanAcceptedFraction = result.AcceptedFractionSum / frames;
        result.ElapsedSeconds = watch.Elapsed.TotalSeconds;
        Check(result, result.InitialValid == 8, $"Only {result.InitialValid}/8 valid legs after settling.");
        Check(result, result.InitialPlanted >= profile.MinimumSupportLegs, $"Only {result.InitialPlanted} initial supports.");
        if (scenario.Motion == Motion.Reverse)
        {
            Check(result, result.MaximumStationaryDisplacement <= profile.ContactToleranceWorld,
                $"Turn-in-place body drift {result.MaximumStationaryDisplacement:G6} > {profile.ContactToleranceWorld}.");
            Check(result, result.PathLength <= profile.ContactToleranceWorld,
                $"Turn-in-place body path {result.PathLength:G6} > {profile.ContactToleranceWorld}.");
            Check(result, result.TurnDegrees >= 170f, $"Stationary reverse achieved only {result.TurnDegrees:F3} degrees (expected >=170).");
        }
        else if (scenario.Motion == Motion.Blocked)
        {
            Check(result, result.RequestedDistance >= 1f, "Blocked scenario did not request meaningful movement.");
            Check(result, result.RejectedLandingSamples > 0, "Obstacle did not force any infeasible landing attempts.");
            Check(result, result.BlockedFrames > 0, "Obstacle did not block the motor.");
            Check(result, result.ObstaclePenetrationFrames == 0,
                $"Crossed obstacle on {result.ObstaclePenetrationFrames} frames; max penetration={result.MaximumObstaclePenetration:G6}.");
        }
        else
        {
            Check(result, result.Displacement >= 1f, $"Net displacement {result.Displacement:F6} < 1 world unit.");
            Check(result, result.MotionRatio >= 0.5f, $"Travel/requested ratio {result.MotionRatio:P2} < 50%.");
        }
        if (scenario.Motion == Motion.Stop)
        {
            Check(result, result.StoppedObservationFrames >= 60, $"Only {result.StoppedObservationFrames} stopped observation frames (expected >=60).");
            Check(result, result.UnsettledStopFrames == 0, $"Feet still swinging or not all planted on {result.UnsettledStopFrames} frames after 0.5s settling.");
            Check(result, result.MaximumStoppedBodyDrift <= profile.ContactToleranceWorld,
                $"Stopped body drift {result.MaximumStoppedBodyDrift:G6} > {profile.ContactToleranceWorld}.");
            Check(result, result.StoppedPathLength <= profile.ContactToleranceWorld,
                $"Stopped body path {result.StoppedPathLength:G6} > {profile.ContactToleranceWorld}.");
            Check(result, result.MaximumStoppedFootDrift <= profile.ContactToleranceWorld,
                $"Stopped foot drift {result.MaximumStoppedFootDrift:G6} > {profile.ContactToleranceWorld}.");
            Check(result, result.MaximumStoppedSpeed <= profile.ContactToleranceWorld,
                $"Stopped speed {result.MaximumStoppedSpeed:G6} > {profile.ContactToleranceWorld} units/s.");
        }
        Check(result, result.MinimumPlanted >= profile.MinimumSupportLegs, $"Minimum supports {result.MinimumPlanted} < {profile.MinimumSupportLegs}.");
        Check(result, result.MinimumValid == 8, $"Minimum valid feet {result.MinimumValid}/8.");
        Check(result, result.MaximumSwing <= profile.MaximumSwingLegs, $"Maximum simultaneous swing legs {result.MaximumSwing} > {profile.MaximumSwingLegs}.");
        Check(result, result.MixedTetradFrames == 0 && result.OversizedTetradFrames == 0 && result.GroupMismatchSamples == 0,
            $"Tetrad violations: mixed={result.MixedTetradFrames}, more than four={result.OversizedTetradFrames}, wrong group={result.GroupMismatchSamples}; first={result.FirstTetradFailure}.");
        Check(result, result.FinalHeadingAlignment >= 0.98f, $"Final physical body heading alignment {result.FinalHeadingAlignment:G6} < 0.98.");
        if (scenario.Motion != Motion.Blocked)
            Check(result, landings.All(n => n > 0), $"Not every leg completed a step: [{string.Join(",", landings)}].");
        Check(result, result.MaximumSkinResidual <= profile.ContactToleranceWorld, $"Skin residual {result.MaximumSkinResidual:G6} > {profile.ContactToleranceWorld}.");
        Check(result, result.MaximumPlantedResidual <= profile.ContactToleranceWorld, $"Planted residual {result.MaximumPlantedResidual:G6} > {profile.ContactToleranceWorld}.");
        Check(result, result.MaximumClearanceError <= profile.ContactToleranceWorld, $"Body clearance error {result.MaximumClearanceError:G6} > {profile.ContactToleranceWorld}.");
        Check(result, result.SurfaceLostFrames == 0, $"Lost surface on {result.SurfaceLostFrames} frames.");
        Check(result, result.NonFiniteSamples == 0 && result.InvalidContactSamples == 0, $"Nonfinite={result.NonFiniteSamples}; invalid contacts={result.InvalidContactSamples}.");
        if (scenario.Motion == Motion.Turn) Check(result, result.TurnDegrees >= 80f, $"Turn achieved only {result.TurnDegrees:F3} degrees (expected >=80).");
        Console.WriteLine($"END {scenario.Name} {(result.Failures.Count == 0 ? "PASS" : "FAIL")} displacement={result.Displacement:F6} path={pathLength:F6} requested={requestedDistance:F3} ratio={(result.MotionRatio.HasValue ? result.MotionRatio.Value.ToString("P2") : "n/a")} supportsMin={result.MinimumPlanted} validMin={result.MinimumValid} landed=[{string.Join(",", landings)}] blocked={result.BlockedFrames}/{frames} elapsed={result.ElapsedSeconds:F2}s");
        Console.WriteLine($"COST {scenario.Name} updates={frames} fixedMs[p95={result.FixedUpdateMilliseconds.P95:F3},max={result.FixedUpdateMilliseconds.Maximum:F3},mean={result.FixedUpdateMilliseconds.Mean:F3}] motorMs[p95={result.MotorMilliseconds.P95:F3},max={result.MotorMilliseconds.Maximum:F3}] gaitMs[p95={result.GaitMilliseconds.P95:F3},max={result.GaitMilliseconds.Maximum:F3}] overDt={result.UpdatesOverFixedDelta}/{frames} allocBytes[total={result.AllocatedBytesCurrentThreadPerUpdate.Total:F0},p95={result.AllocatedBytesCurrentThreadPerUpdate.P95:F0},max={result.AllocatedBytesCurrentThreadPerUpdate.Maximum:F0}] GC=[{string.Join(",", result.GCCollections)}]");
        Console.WriteLine($"SOLVER {scenario.Name} shapeQueries[p95={result.FrameShapeQueries?.P95.ToString("F0") ?? "n/a"},max={result.FrameShapeQueries?.Maximum.ToString("F0") ?? "n/a"}] raycasts[p95={result.FrameRaycasts?.P95.ToString("F0") ?? "n/a"},max={result.FrameRaycasts?.Maximum.ToString("F0") ?? "n/a"}] shapeBudget={result.MaximumShapeQueriesPerUpdate?.ToString() ?? "n/a"} raycastBudget={result.MaximumRaycastsPerUpdate?.ToString() ?? "n/a"} exhaustedFrames={result.QueryBudgetExhaustedFrames?.ToString() ?? "n/a"}");
        if (privateMemory) Console.WriteLine($"MEMORY {scenario.Name} privateBefore={result.PrivateMemoryBeforeBytes} privateAfter={result.PrivateMemoryAfterBytes} delta={result.PrivateMemoryAfterBytes - result.PrivateMemoryBeforeBytes}");
        if (obstacle != null) Console.WriteLine($"BLOCKED rejectedLandings={result.RejectedLandingSamples} obstacleContacts={result.ObstacleContactSamples} penetrationFrames={result.ObstaclePenetrationFrames} maxPenetration={result.MaximumObstaclePenetration:G6}");
        Console.WriteLine($"TETRADS {scenario.Name} group0Frames={result.SwingGroupFrames[0]} group1Frames={result.SwingGroupFrames[1]} mixed={result.MixedTetradFrames} oversized={result.OversizedTetradFrames} groupMismatch={result.GroupMismatchSamples}");
        if (scenario.Motion == Motion.Stop) Console.WriteLine($"STOP observed={result.StoppedObservationFrames} frames bodyDrift={result.MaximumStoppedBodyDrift:G6} footDrift={result.MaximumStoppedFootDrift:G6} path={result.StoppedPathLength:G6} speed={result.MaximumStoppedSpeed:G6} unsettled={result.UnsettledStopFrames}");
        if (scenario.Motion == Motion.Reverse) Console.WriteLine($"REVERSE angle={result.TurnDegrees:F6} headingDot={result.FinalHeadingAlignment:G6} bodyDrift={result.MaximumStationaryDisplacement:G6}");
        foreach (string failure in result.Failures) Console.WriteLine($"ASSERT FAIL: {failure}");
        foreach (var entry in result.BodyFailures.OrderByDescending(e => e.Value)) Console.WriteLine($"BODY FAILURE {entry.Value}: {entry.Key}");
        foreach (var entry in rejectionDiagnostics) Console.WriteLine($"IK DIAGNOSTIC {entry.Key}: {entry.Value}");
        DumpLegs(gait, skeleton, Pose(), scenario.Normal, "final");
        // RigidBody.Destroy releases shapes only; remove native bodies first.
        physics.DestroyBody(body.Native);
        body.Destroy();
        if (obstacle != null)
        {
            physics.DestroyBody(obstacle.Native);
            obstacle.Destroy();
        }
        physics.DestroyBody(ground.Native);
        ground.Destroy();
        return result;
    }

    private static void Initialize(ProceduralSpiderWalk gait, Skeleton skeleton, float scale)
    {
        Type dataType = typeof(SpiderEnemy).GetNestedType("LegData", BindingFlags.NonPublic)
            ?? throw new MissingMemberException("SpiderEnemy.LegData");
        Array data = Array.CreateInstance(dataType, 8);
        for (int i = 0; i < 8; i++)
        {
            string side = i < 4 ? "L" : "R";
            int n = i % 4;
            string[] names = [$"{side}.thigh.{n}", $"{side}.Leg.{n}", $"{side}.foot.{n}", $"{side}.toes.{n}"];
            int[] indices = names.Select(name => skeleton.TryGetNodeIndex(name, out int index) ? index :
                throw new InvalidOperationException($"Missing rig node {name}")).ToArray();
            object leg = Activator.CreateInstance(dataType)!;
            dataType.GetField("ThighNodeIndex", Fields)!.SetValue(leg, indices[0]);
            dataType.GetField("SegmentNodeIndices", Fields)!.SetValue(leg, indices[1..]);
            data.SetValue(leg, i);
            Vector3[] points = indices.Select(index => Point(skeleton.Nodes[index].RestGlobal)).ToArray();
            Console.WriteLine($"LEG {i} {names[0]} nodes=[{string.Join(",", indices)}] restWorldLengths=[{string.Join(",", Enumerable.Range(0, 3).Select(j => (Vector3.Distance(points[j], points[j + 1]) * scale).ToString("F6")))}] restHip={V(points[0] * scale)} restTip={V(points[3] * scale)}");
        }
        typeof(ProceduralSpiderWalk).GetMethod("Initialize", Fields)!.Invoke(gait, [skeleton, data]);
        if (Legs(gait).Length != 8) throw new InvalidOperationException("Initialize did not accept all eight real rig legs.");
    }

    private static string DescribeIkRejection(ProceduralSpiderWalk gait, Vector3 position, Quaternion rotation, string reason)
    {
        object leg = Legs(gait).First(l => reason.StartsWith($"leg {Field<int>(l, "Index")}:"));
        var oldBody = Field<SpiderLocomotionPose>(gait, "_body");
        var nextBody = oldBody with { Position = position, Rotation = rotation };
        var contact = Field<SpiderSurfaceContact>(leg, "Planted");
        bool supported = contact.IsValid && !Field<bool>(leg, "Stepping");
        Vector3 normal = supported ? contact.Normal : Field<Vector3>(leg, "Normal");
        Vector3 foot = supported ? (Vector3)typeof(ProceduralSpiderWalk)
            .GetMethod("ContactPoint", Fields)!.Invoke(gait, [leg, contact, nextBody])!
            : Field<Vector3>(leg, "Foot");
        object rig = leg.GetType().GetField("Rig", Fields)!.GetValue(leg)!;
        object previous = leg.GetType().GetField("Pose", Fields)!.GetValue(leg)!;
        Vector3 target = nextBody.ToModel(foot);
        // Diagnostic only: no collision callback. Never feed this result back into the validator.
        object?[] arguments = [target, Vector3.Normalize(nextBody.DirectionToModel(normal)), previous, null, null];
        bool rawIk = (bool)rig.GetType().GetMethod("TrySolve")!.Invoke(rig, arguments)!;
        float reach = (float)rig.GetType().GetProperty("Reach")!.GetValue(rig)!;
        Vector3 hip = (Vector3)previous.GetType().GetProperty("Hip")!.GetValue(previous)!;
        return $"rawIkWithoutCollisions={rawIk}; targetDistanceWorld={Vector3.Distance(hip, target) * nextBody.Scale:F6}; " +
            $"reachWorld={reach * nextBody.Scale:F6}; stepping={Field<bool>(leg, "Stepping")}; " +
            $"bodyDelta={V(position - oldBody.Position)}; " +
            (rawIk ? "IK has a solution; clearance rejection remains to isolate." : "IK sampling/joint constraints fail even without collision checks.");
    }

    private static void DumpLegs(ProceduralSpiderWalk gait, Skeleton skeleton, SpiderLocomotionPose pose, Vector3 normal, string stage)
    {
        foreach (object leg in Legs(gait))
        {
            var foot = Field<Vector3>(leg, "Foot");
            var actual = pose.ToWorld(Point(skeleton.Nodes[Field<int>(leg, "Tip")].Global));
            var contact = Field<SpiderSurfaceContact>(leg, "Planted");
            Console.WriteLine($"LEGSTATE {stage} leg={Field<int>(leg, "Index")} valid={Field<bool>(leg, "PoseValid")} planted={contact.IsValid} swing={Field<bool>(leg, "Stepping")} recovery={Field<bool>(leg, "NeedsRecovery")} foot={V(foot)} actual={V(actual)} residual={Vector3.Distance(foot, actual):G6} planeHeight={Vector3.Dot(actual, normal):G6} failure={Field<string>(leg, "Failure")}");
        }
    }

    private static (int Mask, int Group) CheckTetrads(Result result, object[] legs, int frame)
    {
        int mask = 0, groups = 0, mismatches = 0;
        foreach (object leg in legs)
        {
            int index = Field<int>(leg, "Index");
            // Left 0..3, right 4..7: diagonally alternating tetrads, independent of production's Group field.
            int expectedGroup = ((index % 4) + index / 4) % 2;
            if (Field<int>(leg, "Group") != expectedGroup) mismatches++;
            if (!Field<bool>(leg, "Stepping")) continue;
            mask |= 1 << index;
            groups |= 1 << expectedGroup;
        }
        bool mixed = groups == 3;
        bool oversized = BitOperations.PopCount((uint)mask) > 4;
        if (mixed) result.MixedTetradFrames++;
        if (oversized) result.OversizedTetradFrames++;
        result.GroupMismatchSamples += mismatches;
        if ((mixed || oversized || mismatches > 0) && result.FirstTetradFailure.Length == 0)
            result.FirstTetradFailure = $"frame={frame}, swingMask=0x{mask:X2}, mixed={mixed}, oversized={oversized}, wrongGroup={mismatches}";
        int group = groups switch { 0 => -1, 1 => 0, 2 => 1, _ => -2 };
        if (group >= 0) result.SwingGroupFrames[group]++;
        return (mask, group);
    }

    private static object[] Legs(ProceduralSpiderWalk gait) => ((IEnumerable)typeof(ProceduralSpiderWalk).GetField("_legs", Fields)!.GetValue(gait)!).Cast<object>().ToArray();
    private static T Field<T>(object obj, string name) => (T)obj.GetType().GetField(name, Fields)!.GetValue(obj)!;
    private static string Hash(string path) => Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path)));
    private static Vector3 Point(Matrix4x4 m) => new(m.M14, m.M24, m.M34);
    private static bool Finite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
    private static string V(Vector3 v) => $"({v.X:F6},{v.Y:F6},{v.Z:F6})";
    private static void Check(Result result, bool condition, string failure) { if (!condition) result.Failures.Add(failure); }
    private static void Increment(Dictionary<string, int> counts, string key) => counts[key] = counts.GetValueOrDefault(key) + 1;
    private static string Option(string[] args, string name, string fallback)
    {
        int index = Array.IndexOf(args, name);
        return index < 0 ? fallback : index + 1 < args.Length ? args[index + 1] : throw new ArgumentException($"Missing value for {name}");
    }
    private static string FindRoot()
    {
        for (DirectoryInfo? directory = new(AppContext.BaseDirectory); directory != null; directory = directory.Parent)
            if (File.Exists(Path.Combine(directory.FullName, "Fuse", "Fuse.csproj"))) return directory.FullName;
        throw new DirectoryNotFoundException("Cannot locate Fuse/Fuse.csproj from harness output directory.");
    }

    private static Func<T>? OptionalReader<T>(object instance, string name)
    {
        // Resolve once before measurement; missing future APIs stay null, never fake zero.
        var property = instance.GetType().GetProperty(name, Fields);
        if (property != null)
        {
            if (property.PropertyType != typeof(T) || property.GetMethod == null)
                throw new InvalidOperationException($"Unexpected metric contract: {instance.GetType().Name}.{name}");
            return property.GetMethod.CreateDelegate<Func<T>>(instance);
        }
        var field = instance.GetType().GetField(name, Fields);
        if (field == null) return null;
        if (field.FieldType != typeof(T))
            throw new InvalidOperationException($"Unexpected metric contract: {instance.GetType().Name}.{name}");
        return () => (T)field.GetValue(instance)!;
    }

    private static long PrivateMemoryBytes()
    {
        using var process = Process.GetCurrentProcess();
        return process.PrivateMemorySize64;
    }

    private readonly record struct Cost(long Ticks, long AllocatedBytes, int Gen0, int Gen1, int Gen2)
    {
        public double Milliseconds => Ticks * (1000d / Stopwatch.Frequency);
    }

    private readonly record struct Measurement(long Bytes, int Gen0, int Gen1, int Gen2, long Timestamp)
    {
        public static Measurement Start() => new(GC.GetAllocatedBytesForCurrentThread(),
            GC.CollectionCount(0), GC.CollectionCount(1), GC.CollectionCount(2), Stopwatch.GetTimestamp());

        public Cost Finish()
        {
            long ticks = Stopwatch.GetTimestamp() - Timestamp;
            return new(ticks, GC.GetAllocatedBytesForCurrentThread() - Bytes,
                GC.CollectionCount(0) - Gen0, GC.CollectionCount(1) - Gen1, GC.CollectionCount(2) - Gen2);
        }
    }

    private readonly record struct UpdateSample(Cost Motor, Cost Gait)
    {
        public double Milliseconds => Motor.Milliseconds + Gait.Milliseconds;
        public long AllocatedBytes => Motor.AllocatedBytes + Gait.AllocatedBytes;
        public int Gen0 => Motor.Gen0 + Gait.Gen0;
        public int Gen1 => Motor.Gen1 + Gait.Gen1;
        public int Gen2 => Motor.Gen2 + Gait.Gen2;
    }

    private sealed class Statistics
    {
        public int Count;
        public double Total, Mean, P50, P95, Maximum;

        public static Statistics Of(IEnumerable<double> samples)
        {
            double[] sorted = samples.Order().ToArray();
            if (sorted.Length == 0) throw new ArgumentException("No benchmark samples.");
            double total = sorted.Sum();
            return new Statistics
            {
                Count = sorted.Length, Total = total, Mean = total / sorted.Length,
                P50 = sorted[(int)Math.Ceiling(sorted.Length * 0.5) - 1],
                P95 = sorted[(int)Math.Ceiling(sorted.Length * 0.95) - 1], Maximum = sorted[^1]
            };
        }
    }

    private enum Motion { Walk, Turn, Stop, Reverse, Blocked }
    private sealed record Scenario(string Name, Vector3 Normal, Vector3 Forward, Motion Motion = Motion.Walk, float? Speed = null);
    private sealed class Result
    {
        public string Case = "";
        public int Frames, InitialPlanted, InitialValid, MinimumPlanted = 8, MinimumValid = 8, MaximumSwing;
        public int BlockedFrames, SurfaceLostFrames, NonFiniteSamples, InvalidContactSamples;
        public float Speed, Displacement, ForwardDisplacement, PathLength, RequestedDistance, TurnDegrees, FinalHeadingAlignment;
        public float? MotionRatio;
        public float CommandedPathLength, MaximumStationaryDisplacement, MaximumStoppedBodyDrift, MaximumStoppedFootDrift;
        public float StoppedPathLength, MaximumStoppedSpeed;
        public int StoppedObservationFrames, UnsettledStopFrames, MixedTetradFrames, OversizedTetradFrames, GroupMismatchSamples;
        public int[] SwingGroupFrames = new int[2];
        public string FirstTetradFailure = "";
        public Vector3 StopPosition;
        public float MinimumAcceptedFraction = 1, AcceptedFractionSum, MeanAcceptedFraction, MaximumClearanceError;
        public float MaximumSkinResidual, MaximumPlantedResidual;
        public double ElapsedSeconds;
        public Statistics FixedUpdateMilliseconds = new(), MotorMilliseconds = new(), GaitMilliseconds = new();
        public Statistics AllocatedBytesCurrentThreadPerUpdate = new();
        public Statistics? FrameShapeQueries, FrameRaycasts;
        public int[] GCCollections = new int[3];
        public int MeasuredThreadId, UpdatesOverFixedDelta, RejectedLandingSamples, ObstaclePenetrationFrames, ObstacleContactSamples;
        public int? MaximumShapeQueriesPerUpdate, MaximumRaycastsPerUpdate, QueryBudgetExhaustedFrames;
        public long? PrivateMemoryBeforeBytes, PrivateMemoryAfterBytes;
        public float? ObstacleFrontZ;
        public float MaximumObstaclePenetration;
        public Vector3 Start, End;
        public int[] Landings = [];
        public Dictionary<string, int> BodyFailures = [], LegFailures = [];
        public Dictionary<string, string> RejectionDiagnostics = [];
        public List<string> Failures = [];
    }
}
