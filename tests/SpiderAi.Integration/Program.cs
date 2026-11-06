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
    private const float ArrivalRadius = 1.25f;
    private const int ObservationFrames = 120;
    private const BindingFlags Members = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private static readonly JsonSerializerOptions Json = new() { WriteIndented = true, IncludeFields = true };
    private static readonly string[] Cases = ["stationary", "moving", "airborne", "near-arrival", "f3-off", "f6-off-f3-on", "low-box", "blocked",
            "hysteresis", "target-lost-reacquire", "target-lost-reacquire-boundary", "detour-retarget",
                        "arrived-clear-near", "elevated-near-state"];

    public static int Main(string[] args)
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        try
        {
            string root = FindRoot();
            string selected = Option(args, "--case", "all");
            string f3Override = Option(args, "--f3-override", "resume");
            if (f3Override is not ("observe" or "resume" or "stay-disabled"))
                throw new ArgumentException("--f3-override observe|resume|stay-disabled");
            string[] chosen = Cases.Where(c => selected == "all" || c == selected).ToArray();
            if (chosen.Length == 0) throw new ArgumentException($"--case all|{string.Join('|', Cases)}");
            double maxUpdateMs = double.Parse(Option(args, "--max-update-ms", "250"));
            double maxP95Ms = double.Parse(Option(args, "--max-p95-ms", "16.666667"));
            if (!double.IsFinite(maxUpdateMs) || maxUpdateMs <= 0 || !double.IsFinite(maxP95Ms) || maxP95Ms <= 0)
                throw new ArgumentException("Timing limits must be positive finite milliseconds.");
            string output = Path.GetFullPath(Option(args, "--output", Path.Combine(root, "tests", "SpiderAi.Integration", "results-expanded")));
            Directory.CreateDirectory(output);
            string rig = Path.Combine(root, "Fuse", "res", "skinned_models", "spider.fbx");
            string[] sources = ["Fuse/src/Enemy/SpiderPatrol.cs", "Fuse/src/Enemy/SpiderSurfacePursuitPlanner.cs",
                "Fuse/src/Core/DevShortcuts.cs", "Fuse/src/Enemy/SpiderSurfaceMotor.cs", "Fuse/src/Enemy/SpiderSurfaceSolver.cs",
                "Fuse/src/Enemy/SpiderLocomotionProfile.cs", "Fuse/src/Enemy/SpiderEnemy.cs", "Fuse/src/Enemy/SpiderLocomotionPose.cs",
                "Fuse/src/Animation/ProceduralSpiderWalk.cs", "Fuse/src/Animation/SpiderLegKinematics.cs",
                "Fuse/src/Renderer/SkinnedModelLoader.cs", "tests/SpiderAi.Integration/Program.cs"];
            var hashes = sources.ToDictionary(p => p, p => Hash(Path.Combine(root, p)));
            File.WriteAllText(Path.Combine(output, "environment.json"), JsonSerializer.Serialize(new
            {
                Utc = DateTime.UtcNow, Cases = chosen, F3Override = f3Override, FixedDeltaSeconds = Dt,
                ArrivalRadius, ObservationFrames, SettlingFrames = 30, MaximumUpdateMilliseconds = maxUpdateMs,
                MaximumP95Milliseconds = maxP95Ms, Profile = SpiderLocomotionProfile.Default,
                RigSha256 = Hash(rig), FuseAssemblySha256 = Hash(typeof(SpiderPatrol).Assembly.Location),
                FuseAssemblyMvid = typeof(SpiderPatrol).Module.ModuleVersionId,
                HarnessAssemblySha256 = Hash(typeof(Program).Assembly.Location), SourceHashesAtRunStart = hashes,
                Runtime = System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription,
                OS = System.Runtime.InteropServices.RuntimeInformation.OSDescription,
                StopwatchFrequency = Stopwatch.Frequency
            }, Json));
            var model = SkinnedModelLoader.LoadRig(rig) ?? throw new InvalidOperationException("LoadRig returned null.");
            Console.WriteLine($"HEADLESS FBX={Hash(rig)} nodes={model.Skeleton.Nodes.Length} bones={model.Skeleton.Bones.Length} F3/F6={f3Override}");
            var results = new List<Result>();
            foreach (string name in chosen)
            {
                try { results.Add(Run(name, model.Skeleton.CreateInstance(), output, f3Override, maxUpdateMs, maxP95Ms)); }
                catch (Exception ex)
                {
                    Console.WriteLine($"HARNESS ERROR {name}: {ex}");
                    results.Add(new Result { Case = name, Failures = [$"HARNESS ERROR: {ex}"] });
                }
                File.WriteAllText(Path.Combine(output, "summary.json"), JsonSerializer.Serialize(results, Json));
            }
            string[] changed = sources.Where(p => hashes[p] != Hash(Path.Combine(root, p))).ToArray();
            File.WriteAllText(Path.Combine(output, "sources-changed-during-run.json"), JsonSerializer.Serialize(changed, Json));
            if (changed.Length > 0) Console.WriteLine($"WARNING sources changed during run: {string.Join(", ", changed)}; baseline describes the loaded assembly.");
            Console.WriteLine($"RESULT {(results.All(r => r.Failures.Count == 0) ? "PASS" : "FAIL")} reports={output}");
            return results.All(r => r.Failures.Count == 0) ? 0 : 1;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"HARNESS ERROR: {ex}");
            return 2;
        }
    }

    private static Result Run(string name, Skeleton skeleton, string output, string f3Override, double maxUpdateMs, double maxP95Ms)
    {
        bool oldEnabled = SpiderPatrol.Enabled;
        bool oldPursuit = SpiderPatrol.PursuitEnabled;
        bool oldHasTarget = SpiderPatrol.HasPursuitTarget;
        Vector3 oldTarget = SpiderPatrol.PursuitTarget;
        bool oldHasSurface = SpiderPatrol.HasPursuitTargetSurface;
        Vector3 oldPoint = SpiderPatrol.PursuitTargetSurfacePoint;
        Vector3 oldNormal = SpiderPatrol.PursuitTargetSurfaceNormal;
        var oldBody = SpiderPatrol.PursuitTargetSurfaceBodyId;
        try
        {
            using var world = new Fixture(skeleton, name);
            var profile = SpiderLocomotionProfile.Default;
            int frames = name switch
                        {
                            "elevated-near-state" => 1,
                                                        "near-arrival" => 360, "f3-off" or "f6-off-f3-on" or "arrived-clear-near" => 600,
                            "moving" or "hysteresis" or "target-lost-reacquire" or "target-lost-reacquire-boundary" or "detour-retarget" => 1080, _ => 900
                        };
            var result = new Result { Case = name, Frames = frames, F3OverrideExpectation = f3Override };
            SpiderPatrol.Enabled = false;
            SpiderPatrol.SetPursuitEnabled(false);
            SpiderPatrol.ClearPursuitTarget();
            for (int i = 0; i < 30; i++) world.Tick();
            Check(result, world.Patrol.State == SpiderAiState.Disabled, "SETUP: settling with F6 OFF must report Disabled.");
            Check(result, world.Patrol.PursuitStopDistance == 1f && world.Patrol.PursuitResumeDistance == 1.5f,
                "CONTRACT: default pursuit stop/resume distances must be 1.0/1.5m.");
            result.Start = world.Position;
            object[] legs = ((IEnumerable)Field(world.Gait, "_legs")).Cast<object>().ToArray();
            Check(result, legs.Length == 8 && legs.All(l => Read<bool>(l, "PoseValid")), "SETUP: all eight FBX legs must be valid after settling.");
            Check(result, world.Gait.PlantedLegCount >= profile.MinimumSupportLegs, "SETUP: insufficient settled supporting legs.");
            SceneRaycastHit? elevatedSupport = name == "elevated-near-state" ? world.ResolveElevatedTarget() : null;
                        Vector3 target = elevatedSupport?.Position ?? new(0, name == "airborne" ? 4 : 0, name == "near-arrival" ? 1 : 18);
                        if (elevatedSupport.HasValue)
                        {
                            Check(result, MathF.Abs(target.Y - 1.2f) <= profile.ContactToleranceWorld &&
                                MathF.Abs(TangentDistance(world.Position, target) - 0.9f) <= profile.ContactToleranceWorld,
                                "ELEVATED SETUP: resolved target must be 1.2m above floor zero and 0.9m away in XZ.");
                            Check(result, world.Motor.SurfaceContact.BodyId == world.Ground.Native &&
                                MathF.Abs(world.Motor.SurfaceContact.Point.Y) <= profile.ContactToleranceWorld,
                                "ELEVATED SETUP: spider must still be supported by the physical lower floor.");
                            result.LifecycleMetrics["ResolvedTargetHeight"] = target.Y;
                            result.LifecycleMetrics["InitialTangentDistance"] = TangentDistance(world.Position, target);
                        }
            SpiderPatrol.Enabled = name != "f6-off-f3-on";
            SpiderPatrol.SetPursuitEnabled(name != "f6-off-f3-on");
            LifecycleScenario? lifecycle = name is "hysteresis" or "target-lost-reacquire" or "target-lost-reacquire-boundary" or "detour-retarget" or "arrived-clear-near" ? new(name) : null;
            if (lifecycle != null) target = lifecycle.Target;
            var states = new List<SpiderAiState>(frames);
            var positions = new List<Vector3>(frames);
            var speeds = new List<float>(frames);
            var distances = new List<float>(frames);
            var costs = new List<double>(frames);
            var patrolCosts = new List<double>(frames);
            var gaitCosts = new List<double>(frames);
            var shapes = new List<double>(frames);
            var rays = new List<double>(frames);
            Vector3 previous = world.Position;
            int offPlans = 0, offPursuitTargets = 0, resumedPatrolFrames = 0, disabledMotionFrames = 0;
            int disabledStateViolations = 0, disabledIntentFrames = 0;
            float postSwitchPath = 0, pausedPath = 0, pausedDrift = 0;
            bool f3Reactivated = false;
            Vector3 switchPosition = default, pausePosition = default;
            using var csv = new StreamWriter(Path.Combine(output, name + ".csv"));
            csv.WriteLine("frame,time,x,y,z,targetX,targetY,targetZ,tangentDistance,pursuit,enabled,hasTarget,hasTargetSurface,state,phase,remainingTravel,commandSpeed,desiredSpeed,actualSpeed,planted,swing,hasSurface,blocked,hasPlan,detour,bodyReason,patrolMs,gaitMs,updateMs,shapeQueries,raycasts,budgetExhausted");
            Console.WriteLine($"BEGIN {name} frames={frames} start={V(previous)} target={V(target)}");
            for (int frame = 0; frame < frames; frame++)
            {
                if (name == "moving") target = new(0, 0, 18 + 0.8f * MathF.Min(frame * Dt, 8f));
                if (name == "f3-off" && frame == 180)
                {
                    SpiderPatrol.SetPursuitEnabled(false);
                    switchPosition = previous;
                }
                // Continue changing the target after OFF; stale pursuit intent must not follow it.
                if (name == "f3-off" && frame >= 180) target = new(30 + frame * Dt, 0, -20);
                if (name == "f6-off-f3-on" && frame == 120)
                {
                    SpiderPatrol.SetPursuitEnabled(true);
                    f3Reactivated = SpiderPatrol.Enabled;
                    switchPosition = previous;
                }
                if (name == "f6-off-f3-on" && frame == 240)
                {
                    SpiderPatrol.Enabled = false;
                    pausePosition = previous;
                    Check(result, SpiderPatrol.PursuitEnabled, "F6 PAUSE: test must pause an active pursuit without disabling F3.");
                }
                if (name == "f6-off-f3-on" && frame == 300) SpiderPatrol.Enabled = true;
                lifecycle?.Before(frame, world, result);
                if (lifecycle != null) target = lifecycle.Target;
                if (lifecycle?.HasTarget != false)
                {
                    SpiderPatrol.SetPursuitTarget(target);
                    if (name == "airborne") SpiderPatrol.ClearPursuitTargetSurface();
                    else if (elevatedSupport is { } support) SpiderPatrol.SetPursuitTargetSurface(support.Position, support.Normal, support.BodyID);
                                        else SpiderPatrol.SetPursuitTargetSurface(target, Vector3.UnitY, world.Ground.Native);
                }
                var cost = world.Tick();
                Vector3 current = world.Position;
                float travel = Vector3.Distance(previous, current);
                float distance = TangentDistance(current, target);
                positions.Add(current);
                speeds.Add(world.Motor.CurrentSpeed);
                distances.Add(distance);
                costs.Add(cost.PatrolMs + cost.GaitMs);
                patrolCosts.Add(cost.PatrolMs);
                gaitCosts.Add(cost.GaitMs);
                shapes.Add(world.Solver.FrameShapeQueries);
                rays.Add(world.Solver.FrameRaycasts);
                result.PathLength += travel;
                result.MinimumDistance = MathF.Min(result.MinimumDistance, distance);
                result.MaximumFrameDisplacement = MathF.Max(result.MaximumFrameDisplacement, travel);
                result.MaximumSpeed = MathF.Max(result.MaximumSpeed, world.Motor.CurrentVelocity.Length());
                result.MaximumHeightError = MathF.Max(result.MaximumHeightError, MathF.Abs(current.Y - profile.BodyClearance));
                result.MinimumPlanted = Math.Min(result.MinimumPlanted, world.Gait.PlantedLegCount);
                if (world.Motor.IsBlocked) result.BlockedFrames++;
                if (!world.Motor.HasSurface) result.SurfaceLostFrames++;
                if (world.Patrol.PursuitPlanner.IsSameSurfaceDetourActive) result.DetourFrames++;
                if (world.Solver.QueryBudgetExhausted) result.BudgetExhaustedFrames++;
                if (!Finite(current) || !Finite(world.Motor.CurrentVelocity) || !Finite(world.Motor.SurfaceNormal) ||
                    !Finite(world.Motor.Forward) || !Finite(world.Motor.DesiredRelativeVelocity) || !float.IsFinite(world.Motor.CurrentSpeed) ||
                    !Finite(world.Body.Rotation(world.Physics))) result.NonFiniteFrames++;
                if (world.Obstacle != null)
                    result.MaximumBodyPenetration = MathF.Max(result.MaximumBodyPenetration,
                        CapsulePenetration(current, world.Motor.SurfaceNormal, profile, world.ObstacleCenter, world.ObstacleHalfExtents));
                if (world.TargetSupport != null)
                    result.MaximumBodyPenetration = MathF.Max(result.MaximumBodyPenetration,
                        CapsulePenetration(current, world.Motor.SurfaceNormal, profile, world.TargetSupportCenter, world.TargetSupportHalfExtents));
                string reason = world.Motor.LastBodyBlockReason;
                if (reason.Length > 0) result.BodyReasons[reason] = result.BodyReasons.GetValueOrDefault(reason) + 1;
                SpiderAiState state = world.Patrol.State;
                                if (states.Count == 0 || states[^1] != state) result.StateTransitions.Add($"{frame}: {state}");
                                states.Add(state);
                                result.StateFrames[state.ToString()] = result.StateFrames.GetValueOrDefault(state.ToString()) + 1;
                                lifecycle?.After(frame, world, result, previous);
                float remaining = Read<float>(world.Patrol, "_remainingTravel");
                if (name == "f3-off" && frame >= 180)
                {
                    postSwitchPath += travel;
                    if (world.Patrol.PursuitPlanner.HasPlan) offPlans++;
                    if (TangentDistance(world.Patrol.NavigationTarget, target) < 0.01f) offPursuitTargets++;
                    if (remaining > 0.8f && state == SpiderAiState.Patrolling && world.Motor.CurrentSpeed > 0.05f) resumedPatrolFrames++;
                }
                if (name == "f6-off-f3-on")
                {
                    if (frame < 120 || frame is >= 240 and < 300)
                    {
                        if (travel > profile.ContactToleranceWorld) disabledMotionFrames++;
                        if (state != SpiderAiState.Disabled) disabledStateViolations++;
                        if (world.Motor.DesiredRelativeVelocity.Length() > 0.05f) disabledIntentFrames++;
                    }
                    if (frame is >= 240 and < 300)
                    {
                        pausedPath += travel;
                        pausedDrift = MathF.Max(pausedDrift, Vector3.Distance(current, pausePosition));
                    }
                    if (frame >= 120) postSwitchPath += travel;
                }
                csv.WriteLine($"{frame},{(frame + 1) * Dt:R},{current.X:R},{current.Y:R},{current.Z:R},{target.X:R},{target.Y:R},{target.Z:R},{distance:R},{SpiderPatrol.PursuitEnabled},{SpiderPatrol.Enabled},{SpiderPatrol.HasPursuitTarget},{SpiderPatrol.HasPursuitTargetSurface},{state},{lifecycle?.Phase ?? "baseline"},{remaining:R},{Read<float>(world.Patrol, "_commandSpeed"):R},{world.Motor.DesiredRelativeVelocity.Length():R},{world.Motor.CurrentSpeed:R},{world.Gait.PlantedLegCount},{world.Gait.SwingLegCount},{world.Motor.HasSurface},{world.Motor.IsBlocked},{world.Patrol.PursuitPlanner.HasPlan},{world.Patrol.PursuitPlanner.IsSameSurfaceDetourActive},\"{reason.Replace("\"", "\"\"")}\",{cost.PatrolMs:R},{cost.GaitMs:R},{cost.PatrolMs + cost.GaitMs:R},{world.Solver.FrameShapeQueries},{world.Solver.FrameRaycasts},{world.Solver.QueryBudgetExhausted}");
                if ((frame + 1) % 180 == 0)
                {
                    csv.Flush();
                    Console.WriteLine($"SAMPLE {name} t={(frame + 1) * Dt:F1} pos={V(current)} distance={distance:F3} speed={world.Motor.CurrentSpeed:F3} blocked={result.BlockedFrames} reason={reason}");
                }
                previous = current;
            }
            result.End = world.Position;
            result.FinalDistance = distances[^1];
            result.UpdateMs = Statistics.Of(costs);
            result.PatrolMs = Statistics.Of(patrolCosts);
            result.GaitMs = Statistics.Of(gaitCosts);
            result.ShapeQueries = Statistics.Of(shapes);
            result.Raycasts = Statistics.Of(rays);
            result.PostSwitchPath = postSwitchPath;
            Check(result, result.NonFiniteFrames == 0, $"SAFETY: non-finite pose/velocity on {result.NonFiniteFrames} frames.");
            Check(result, result.MaximumFrameDisplacement <= world.Patrol.MoveSpeed * Dt + profile.ContactToleranceWorld,
                $"SAFETY: frame displacement {result.MaximumFrameDisplacement:F6} exceeds speed*dt + contact tolerance.");
            Check(result, result.MaximumSpeed <= world.Patrol.MoveSpeed + 0.05f, $"SAFETY: speed exploded to {result.MaximumSpeed:F6}.");
            Check(result, result.SurfaceLostFrames == 0, $"SAFETY: lost physical support on {result.SurfaceLostFrames} frames.");
            Check(result, result.MinimumPlanted >= profile.MinimumSupportLegs, $"SAFETY: minimum supports {result.MinimumPlanted} < {profile.MinimumSupportLegs}.");
            Check(result, result.MaximumBodyPenetration <= profile.ContactToleranceWorld, $"SAFETY: body penetrated obstacle by {result.MaximumBodyPenetration:F6}.");
            if (name != "blocked")
                Check(result, result.MaximumHeightError <= profile.ContactToleranceWorld,
                    $"FLOOR: body left support clearance by {result.MaximumHeightError:F6}; raw airborne target must not make it fly, low box must be detoured on floor.");
            Check(result, profile.MaximumShapeQueriesPerUpdate is > 0 and <= 512 && result.ShapeQueries.Maximum <= profile.MaximumShapeQueriesPerUpdate && result.ShapeQueries.P95 <= 512,
                $"BUDGET: shape queries p95={result.ShapeQueries.P95}, max={result.ShapeQueries.Maximum}, profile={profile.MaximumShapeQueriesPerUpdate}; hard cap=512/update.");
            Check(result, profile.MaximumRaycastsPerUpdate is > 0 and <= 192 && result.Raycasts.Maximum <= profile.MaximumRaycastsPerUpdate && result.Raycasts.P95 <= 192,
                $"BUDGET: raycasts p95={result.Raycasts.P95}, max={result.Raycasts.Maximum}, profile={profile.MaximumRaycastsPerUpdate}; hard cap=192/update.");
            Check(result, result.UpdateMs.Maximum <= maxUpdateMs, $"CPU: update max {result.UpdateMs.Maximum:F3}ms > {maxUpdateMs:F3}ms.");
            Check(result, result.UpdateMs.P95 <= maxP95Ms, $"CPU: update p95 {result.UpdateMs.P95:F3}ms > {maxP95Ms:F3}ms.");
            if (name is "stationary" or "moving" or "airborne" or "near-arrival" or "low-box" || lifecycle != null)
            {
                Check(result, result.FinalDistance <= ArrivalRadius, $"ARRIVAL: final tangent distance {result.FinalDistance:F6} > {ArrivalRadius}m.");
                int begin = frames - ObservationFrames;
                Vector3 anchor = positions[begin];
                result.StoppedDrift = positions.Skip(begin).Max(p => Vector3.Distance(p, anchor));
                result.StoppedPath = Enumerable.Range(begin + 1, ObservationFrames - 1).Sum(i => Vector3.Distance(positions[i - 1], positions[i]));
                result.StoppedMaximumSpeed = speeds.Skip(begin).Max();
                Check(result, distances.Skip(begin).All(d => d <= ArrivalRadius), "ARRIVAL: must remain inside arrival radius for the final 120 frames, not repeatedly enter/leave it.");
                                Check(result, states.Skip(begin).All(s => s == SpiderAiState.Arrived), "STATE: final 120 frames must consistently report Arrived.");
                Check(result, result.StoppedMaximumSpeed <= 0.05f, $"STOP: final-window speed {result.StoppedMaximumSpeed:F6} > 0.05m/s.");
                Check(result, result.StoppedDrift <= profile.ContactToleranceWorld, $"STOP: final-window body drift {result.StoppedDrift:F6} > {profile.ContactToleranceWorld}m.");
                Check(result, result.StoppedPath <= profile.ContactToleranceWorld, $"STOP: final-window path {result.StoppedPath:F6} > {profile.ContactToleranceWorld}m (oscillation counts even if net drift is small).");
            }
            if (name == "moving")
            {
                Check(result, result.End.Z - result.Start.Z >= 20f, $"MOVING: advance {result.End.Z - result.Start.Z:F6} < 20m toward target ending at z=24.4m.");
                result.MovingTailMeanDistance = distances.Skip(360).Take(120).Average();
                Check(result, result.MovingTailMeanDistance <= 8f, $"MOVING: mean chase distance during t=6..8s {result.MovingTailMeanDistance:F6} > 8m.");
            }
            if (name == "f3-off")
            {
                Check(result, offPlans == 0, $"F3 OFF: pursuit plan remained active on {offPlans} frames.");
                Check(result, offPursuitTargets == 0, $"F3 OFF: patrol copied the changing pursuit target on {offPursuitTargets} frames.");
                Check(result, postSwitchPath >= 1f && resumedPatrolFrames >= 30,
                    $"F3 OFF: must resume genuine patrol, path={postSwitchPath:F6}m, moving patrol frames={resumedPatrolFrames}.");
                Check(result, !SpiderPatrol.PursuitEnabled, "F3 OFF: pursuit was re-enabled unexpectedly.");
                                Check(result, states.Skip(180).All(s => s is SpiderAiState.Resting or SpiderAiState.Patrolling),
                                    "F3 OFF: public state must return to Resting/Patrolling, not remain in pursuit.");
            }
            if (name == "f6-off-f3-on")
            {
                result.F3AfterF6Observation = $"Enabled={SpiderPatrol.Enabled}; PursuitEnabled={SpiderPatrol.PursuitEnabled}; pathAfterF3={postSwitchPath:F6}; displacementAfterF3={Vector3.Distance(switchPosition, world.Position):F6}; targetProgress={18 - result.FinalDistance:F6}";
                Check(result, disabledMotionFrames == 0, $"F6 OFF: moved while paused on {disabledMotionFrames} frames.");
                Check(result, disabledStateViolations == 0 && disabledIntentFrames == 0,
                    $"F6 OFF: expected Disabled + zero movement intent; wrong-state={disabledStateViolations}, moving-intent={disabledIntentFrames} frames.");
                Check(result, pausedPath <= profile.ContactToleranceWorld && pausedDrift <= profile.ContactToleranceWorld,
                    $"F6 PAUSE: active pursuit must stop, path={pausedPath:F6}, drift={pausedDrift:F6}m.");
                Check(result, states.Skip(300).Any(s => s is SpiderAiState.Pursuing or SpiderAiState.Navigating),
                    "F6 RESUME: re-enabling Enabled without a new F3 must resume pursuit.");
                if (f3Override == "resume")
                {
                    Check(result, f3Reactivated, "F3 OVERRIDE: SetPursuitEnabled(true) must reactivate Enabled immediately.");
                    Check(result, 18 - result.FinalDistance >= 3f, $"F3 OVERRIDE: expected pursuit despite F6 OFF; {result.F3AfterF6Observation}");
                }
                else if (f3Override == "stay-disabled")
                    Check(result, postSwitchPath <= profile.ContactToleranceWorld, $"F6 PRECEDENCE: expected no movement; {result.F3AfterF6Observation}");
                Console.WriteLine($"F3/F6 OBSERVATION ({f3Override}): {result.F3AfterF6Observation}");
            }
            if (name == "low-box")
            {
                result.MaximumLateralExcursion = positions.Max(p => MathF.Abs(p.X));
                Check(result, result.DetourFrames > 0, "DETOUR: planner never activated a same-surface detour around low box.");
                Check(result, result.MaximumLateralExcursion >= world.ObstacleHalfExtents.X + profile.BodyRadius,
                    $"DETOUR: lateral excursion {result.MaximumLateralExcursion:F6} never cleared obstacle + body radius.");
                Check(result, result.End.Z > world.ObstacleCenter.Z + world.ObstacleHalfExtents.Z, "DETOUR: never passed the low box.");
            }
            if (name == "blocked")
                Check(result, result.MinimumDistance <= 8f, $"BLOCKED: never approached obstruction; minimum target distance {result.MinimumDistance:F6} > 8m.");
            lifecycle?.Verify(world, result);
            if (elevatedSupport.HasValue)
            {
                bool stillOnLowerFloor = world.Motor.SurfaceContact.BodyId == world.Ground.Native &&
                    MathF.Abs(world.Motor.SurfaceContact.Point.Y) <= profile.ContactToleranceWorld &&
                    MathF.Abs(world.Position.Y - profile.BodyClearance) <= profile.ContactToleranceWorld;
                result.LifecycleMetrics["LowerFloorObservationFrames"] = stillOnLowerFloor ? frames : 0;
                result.LifecycleMetrics["ArrivedOnLowerFloorFrames"] = states.Count(s => s == SpiderAiState.Arrived);
                result.LifecycleMetrics["FinalBodyHeight"] = world.Position.Y;
                Check(result, stillOnLowerFloor, "ELEVATED: short state regression must actually observe the spider on floor zero (non-vacuous).");
                Check(result, states.All(s => s != SpiderAiState.Arrived),
                    "ELEVATED: target support 1.2m above lower floor cannot report Arrived merely because XZ distance is 0.9m.");
                Check(result, SpiderPatrol.HasPursuitTargetSurface && SpiderPatrol.PursuitTargetSurfaceBodyId == world.TargetSupport!.Native,
                    "ELEVATED: target must retain its raycast-resolved upper support identity, not be flattened onto the lower floor.");
            }
            Console.WriteLine($"END {name} {(result.Failures.Count == 0 ? "PASS" : "FAIL")} finalDistance={result.FinalDistance:F6} minDistance={result.MinimumDistance:F6} path={result.PathLength:F6} stopPath={result.StoppedPath:F6}");
            Console.WriteLine($"COST {name} updateMs[p95={result.UpdateMs.P95:F3},max={result.UpdateMs.Maximum:F3}] shape[p95={result.ShapeQueries.P95:F0},max={result.ShapeQueries.Maximum:F0}] ray[p95={result.Raycasts.P95:F0},max={result.Raycasts.Maximum:F0}]");
            foreach (string failure in result.Failures) Console.WriteLine($"ASSERT FAIL {name}: {failure}");
            return result;
        }
        finally
        {
            SpiderPatrol.SetPursuitEnabled(oldPursuit);
            // SetPursuitEnabled(true) reactivates AI; restore Enabled after that side effect.
            SpiderPatrol.Enabled = oldEnabled;
            SpiderPatrol.ClearPursuitTarget();
            if (oldHasTarget) SpiderPatrol.SetPursuitTarget(oldTarget);
            if (oldHasSurface) SpiderPatrol.SetPursuitTargetSurface(oldPoint, oldNormal, oldBody);
            if (SpiderPatrol.HasPursuitTarget != oldHasTarget || SpiderPatrol.Enabled != oldEnabled ||
                SpiderPatrol.PursuitEnabled != oldPursuit || SpiderPatrol.HasPursuitTargetSurface != oldHasSurface)
                throw new InvalidOperationException("Teardown did not restore target presence/AI flags.");
        }
    }

    private sealed class LifecycleScenario
    {
        private readonly string _name;
        private int _firstArrival = -1, _nearMove = -1, _loss = -1, _resume = -1, _newArrival = -1, _retarget = -1;
        private int _holdSamples, _holdStateViolations, _targetPresenceViolations, _initialStateViolations;
        private int _detourFramesBefore, _sideChanges, _stalePlanFrames;
        private float _holdPath, _holdDrift, _holdMaximumSpeed, _postResumePath;
        private float _holdMinimumDistance = float.MaxValue, _holdMaximumDistance;
        private Vector3 _holdAnchor, _heldTarget, _oldSide, _newSide;
        private bool _lostWhileMoving, _lostDuringPursuit, _hadTargetBeforeLoss, _invalidatedOnRetarget, _chasingAfterResume;
                private float _lossActualSpeed, _lossDesiredSpeed;
                private bool IsTargetLoss => _name is "target-lost-reacquire" or "target-lost-reacquire-boundary";
        private SpiderAiState? _resumeState;

        public Vector3 Target { get; private set; }
        public bool HasTarget { get; private set; } = true;
        public string Phase { get; private set; } = "approach";

        public LifecycleScenario(string name)
        {
            _name = name;
            Target = new(0, 0, name is "hysteresis" or "arrived-clear-near" ? 6 : 18);
        }

        public void Before(int frame, Fixture world, Result result)
        {
            if (_name == "hysteresis")
            {
                if (_firstArrival >= 0 && frame == _firstArrival + ObservationFrames)
                {
                    _nearMove = frame;
                    _holdAnchor = world.Position;
                    Vector3 away = Target - new Vector3(_holdAnchor.X, 0, _holdAnchor.Z);
                    _heldTarget = Target + Vector3.Normalize(away) * 0.3f;
                    Target = _heldTarget;
                    Phase = "within-resume-band";
                    Event(result, frame, "move target 0.3m outward after 120 Arrived frames", world);
                }
                if (_nearMove >= 0 && frame < _nearMove + 180)
                    Target = _heldTarget + Vector3.UnitX * (0.1f * MathF.Sin((frame - _nearMove) * 0.2f));
                if (_nearMove >= 0 && frame == _nearMove + 180)
                {
                    Target = new Vector3(_holdAnchor.X, 0, _holdAnchor.Z) + Vector3.UnitZ * 4f;
                    _resume = frame;
                    Phase = "far-retarget";
                    Check(result, TangentDistance(world.Position, Target) > 3f, "HYSTERESIS SETUP: far retarget must exceed 3m.");
                    Event(result, frame, "move target >3m; require pursuit and a second arrival", world);
                }
            }
            else if (_name == "arrived-clear-near")
            {
                if (_firstArrival >= 0 && frame == _firstArrival + ObservationFrames)
                {
                    Check(result, world.Patrol.State == SpiderAiState.Arrived && SpiderPatrol.HasPursuitTarget,
                        "ARRIVED CLEAR SETUP: clear must interrupt a genuinely latched Arrived state.");
                    _loss = frame;
                    _holdAnchor = world.Position;
                    SpiderPatrol.ClearPursuitTarget();
                    HasTarget = false;
                    Phase = "arrived-target-cleared";
                    Event(result, frame, "clear after stable Arrived; observe 60 targetless updates", world);
                }
                if (_loss >= 0 && frame == _loss + 60)
                {
                    Target = new Vector3(world.Position.X, 0, world.Position.Z) + Vector3.UnitZ * 1.3f;
                    HasTarget = true;
                    _resume = frame;
                    Phase = "near-target-republished";
                    result.LifecycleMetrics["ReacquireInitialDistance"] = TangentDistance(world.Position, Target);
                    Check(result, MathF.Abs(TangentDistance(world.Position, Target) - 1.3f) <= 0.001f,
                        "ARRIVED CLEAR SETUP: replacement target must start at 1.3m, inside old resume band but outside stop radius.");
                    Event(result, frame, "publish target at 1.3m without toggling F3; stale arrival latch must not persist", world);
                }
            }
            else if (IsTargetLoss)
            {
                // Keep the exact interrupted-stance reproduction as well as the moving-body variant.
                bool requestLoss = _name == "target-lost-reacquire-boundary" ? frame == 60 :
                    frame >= 60 && IsChasing(world.Patrol.State) && world.Motor.CurrentVelocity.Length() > 0.05f;
                if (_loss < 0 && requestLoss)
                {
                    _loss = frame;
                    _lossActualSpeed = world.Motor.CurrentVelocity.Length();
                    _lossDesiredSpeed = world.Motor.DesiredRelativeVelocity.Length();
                    _lostWhileMoving = _lossActualSpeed > 0.05f;
                    _lostDuringPursuit = IsChasing(world.Patrol.State) && _lossDesiredSpeed > 0.05f &&
                        TangentDistance(world.Position, result.Start) >= 1f;
                    _hadTargetBeforeLoss = SpiderPatrol.HasPursuitTarget && SpiderPatrol.HasPursuitTargetSurface;
                    SpiderPatrol.ClearPursuitTarget();
                    HasTarget = false;
                    _holdAnchor = world.Position;
                    Phase = "target-lost";
                    Check(result, !SpiderPatrol.HasPursuitTarget && !SpiderPatrol.HasPursuitTargetSurface,
                        "TARGET LOST: ClearPursuitTarget must immediately clear target presence and surface.");
                    Event(result, frame, "clear a moving pursuit target once; do not republish for 180 frames", world);
                }
                if (_loss >= 0 && frame == _loss + 180)
                {
                    HasTarget = true;
                    Target = new(6, 0, 18);
                    _resume = frame;
                    Phase = "target-reacquired";
                    Check(result, TangentDistance(world.Position, Target) > 3f, "REACQUIRE SETUP: replacement target must be >3m away.");
                    Event(result, frame, "reacquire target without toggling F3 or Enabled", world);
                }
            }
            else if (_retarget < 0 && world.Patrol.PursuitPlanner.IsSameSurfaceDetourActive &&
                _detourFramesBefore >= 15 && MathF.Abs(world.Position.X) >= 0.25f)
            {
                _retarget = _resume = frame;
                Vector3 oldSide = Read<Vector3>(world.Patrol.PursuitPlanner, "_sameSurfaceDetourSide");
                Target = new(-MathF.CopySign(8f, oldSide.X), 0, 18);
                Phase = "detour-retargeted";
                Check(result, world.Patrol.State == SpiderAiState.Navigating, "RETARGET SETUP: active detour must report Navigating.");
                result.LifecycleMetrics["OldDetourSideX"] = oldSide.X;
                result.LifecycleMetrics["OldWaypointX"] = Read<Vector3>(world.Patrol.PursuitPlanner, "_plannedWaypointPosition").X;
                Event(result, frame, "replace active detour target with target on opposite lateral side", world);
            }
        }

        public void After(int frame, Fixture world, Result result, Vector3 previous)
        {
            SpiderAiState state = world.Patrol.State;
            if (_name == "arrived-clear-near")
            {
                if (_firstArrival < 0 && state == SpiderAiState.Arrived)
                {
                    _firstArrival = frame;
                    Event(result, frame, "first Arrived before clear", world);
                }
                if (_firstArrival >= 0 && _loss < 0 && state != SpiderAiState.Arrived) _initialStateViolations++;
                if (_loss >= 0 && !HasTarget)
                {
                    ObserveHold(world, previous);
                    if (state != SpiderAiState.WaitingForSurface) _holdStateViolations++;
                    if (SpiderPatrol.HasPursuitTarget || SpiderPatrol.HasPursuitTargetSurface ||
                        !SpiderPatrol.PursuitEnabled || !SpiderPatrol.Enabled || world.Patrol.PursuitPlanner.HasPlan)
                        _targetPresenceViolations++;
                }
            }
            if (_name == "hysteresis")
            {
                if (_firstArrival < 0 && state == SpiderAiState.Arrived)
                {
                    _firstArrival = frame;
                    Event(result, frame, "first Arrived", world);
                }
                if (_firstArrival >= 0 && _nearMove < 0 && state != SpiderAiState.Arrived) _initialStateViolations++;
                if (_nearMove >= 0 && _resume < 0)
                {
                    ObserveHold(world, previous);
                    if (state != SpiderAiState.Arrived) _holdStateViolations++;
                    float distance = TangentDistance(world.Position, Target);
                    _holdMinimumDistance = MathF.Min(_holdMinimumDistance, distance);
                    _holdMaximumDistance = MathF.Max(_holdMaximumDistance, distance);
                }
            }
            if (IsTargetLoss && !HasTarget)
            {
                ObserveHold(world, previous);
                if (state != SpiderAiState.WaitingForSurface) _holdStateViolations++;
                if (SpiderPatrol.HasPursuitTarget || SpiderPatrol.HasPursuitTargetSurface ||
                    !SpiderPatrol.PursuitEnabled || !SpiderPatrol.Enabled || world.Patrol.PursuitPlanner.HasPlan)
                    _targetPresenceViolations++;
            }
            if (_resume >= 0)
            {
                if (frame == _resume) _resumeState = state;
                if (IsChasing(state)) _chasingAfterResume = true;
                _postResumePath += Vector3.Distance(previous, world.Position);
                if (_newArrival < 0 && state == SpiderAiState.Arrived)
                {
                    _newArrival = frame;
                    Event(result, frame, "Arrived at replacement target", world);
                }
            }
            if (_name == "detour-retarget")
            {
                var planner = world.Patrol.PursuitPlanner;
                bool currentTargetPlan = !planner.HasPlan || TangentDistance(Read<Vector3>(planner, "_plannedTargetPosition"), Target) <= 0.01f;
                if (frame == _retarget)
                    _invalidatedOnRetarget = currentTargetPlan && (planner.HasPlan || !Read<bool>(planner, "_hasSameSurfaceDetourSide"));
                if (_retarget >= 0 && !currentTargetPlan) _stalePlanFrames++;
                if (planner.IsSameSurfaceDetourActive)
                {
                    if (_retarget < 0) _detourFramesBefore++;
                    Vector3 side = Read<Vector3>(planner, "_sameSurfaceDetourSide");
                    if (!Read<bool>(planner, "_hasSameSurfaceDetourSide") || !Finite(side) || side.LengthSquared() < 0.5f)
                        _sideChanges++;
                    else
                    {
                        side = Vector3.Normalize(side);
                        ref Vector3 reference = ref (_retarget < 0 ? ref _oldSide : ref _newSide);
                        if (reference.LengthSquared() > 0.5f)
                        {
                            if (Vector3.Dot(reference, side) < 0.98f) _sideChanges++;
                        }
                        else reference = side;
                    }
                }
            }
        }

        private void ObserveHold(Fixture world, Vector3 previous)
        {
            _holdSamples++;
            _holdPath += Vector3.Distance(previous, world.Position);
            _holdDrift = MathF.Max(_holdDrift, Vector3.Distance(_holdAnchor, world.Position));
            _holdMaximumSpeed = MathF.Max(_holdMaximumSpeed,
                MathF.Max(world.Motor.CurrentVelocity.Length(), world.Motor.DesiredRelativeVelocity.Length()));
        }

        public void Verify(Fixture world, Result result)
        {
            result.LifecycleMetrics["FirstArrivalFrame"] = _firstArrival;
            result.LifecycleMetrics["TargetLostFrame"] = _loss;
                        result.LifecycleMetrics["ResumeFrame"] = _resume;
            result.LifecycleMetrics["NewArrivalFrame"] = _newArrival;
            result.LifecycleMetrics["HoldFrames"] = _holdSamples;
            result.LifecycleMetrics["HoldPath"] = _holdPath;
            result.LifecycleMetrics["HoldDrift"] = _holdDrift;
            result.LifecycleMetrics["HoldMaximumActualOrDesiredSpeed"] = _holdMaximumSpeed;
            result.LifecycleMetrics["PostResumePath"] = _postResumePath;
            Check(result, _resume >= 0 && _chasingAfterResume && _resumeState.HasValue && IsChasing(_resumeState.Value),
                $"LIFECYCLE: must resume as Pursuing/Navigating on replacement-target update; frame={_resume}, state={_resumeState}.");
            if (_name == "arrived-clear-near")
            {
                float maximumArrivalDistance = 1.01f + SpiderLocomotionProfile.Default.ContactToleranceWorld;
                Check(result, _firstArrival >= 0 && _loss == _firstArrival + ObservationFrames && _initialStateViolations == 0,
                    "ARRIVED CLEAR: must first sustain Arrived for 120 frames before clearing the target.");
                Check(result, _holdSamples == 60 && _holdStateViolations == 0 && _targetPresenceViolations == 0 &&
                    _holdPath <= SpiderLocomotionProfile.Default.ContactToleranceWorld &&
                    _holdDrift <= SpiderLocomotionProfile.Default.ContactToleranceWorld && _holdMaximumSpeed <= 0.05f,
                    "ARRIVED CLEAR: targetless updates must report WaitingForSurface and remain stopped with no target/surface/plan.");
                Check(result, _newArrival > _resume && world.Patrol.State == SpiderAiState.Arrived &&
                    TangentDistance(world.Position, Target) <= maximumArrivalDistance &&
                    _postResumePath >= 1.3f - maximumArrivalDistance,
                    $"ARRIVED CLEAR: republishing at 1.3m must approach <=1.01+tol and newly Arrive, not reuse resume hysteresis; path={_postResumePath:F6}, arrival={_newArrival}.");
            }
            else
                Check(result, _newArrival > _resume && world.Patrol.State == SpiderAiState.Arrived && _postResumePath >= 1f,
                    $"LIFECYCLE: replacement target must cause movement and a new Arrived; resume={_resume}, arrival={_newArrival}, path={_postResumePath:F6}.");
            Check(result, TangentDistance(world.Position, Target) <= world.Patrol.PursuitStopDistance + SpiderLocomotionProfile.Default.ContactToleranceWorld,
                "LIFECYCLE: final replacement target must reach the 1.0m stop distance (plus contact tolerance), not merely the legacy 1.25m gate.");
            if (_name == "hysteresis" || IsTargetLoss)
            {
                Check(result, _holdSamples == 180 && _holdStateViolations == 0,
                    $"HOLD: expected 180 stable {_name} state frames; observed={_holdSamples}, violations={_holdStateViolations}.");
                Check(result, _holdPath <= SpiderLocomotionProfile.Default.ContactToleranceWorld &&
                    _holdDrift <= SpiderLocomotionProfile.Default.ContactToleranceWorld && _holdMaximumSpeed <= 0.05f,
                    $"HOLD: must stop without oscillation/intent; path={_holdPath:F6}, drift={_holdDrift:F6}, speed/intent={_holdMaximumSpeed:F6}.");
            }
            if (_name == "hysteresis")
            {
                result.LifecycleMetrics["NearTargetMoveFrame"] = _nearMove;
                result.LifecycleMetrics["HoldMinimumTargetDistance"] = _holdMinimumDistance;
                result.LifecycleMetrics["HoldMaximumTargetDistance"] = _holdMaximumDistance;
                Check(result, _firstArrival >= 0 && _nearMove == _firstArrival + ObservationFrames && _initialStateViolations == 0,
                    "HYSTERESIS: first Arrived must persist for 120 frames before moving the target.");
                Check(result, _holdMinimumDistance > 1f && _holdMaximumDistance < 1.5f,
                    $"HYSTERESIS: small target motion must test the band (1.0,1.5), observed [{_holdMinimumDistance:F6},{_holdMaximumDistance:F6}].");
            }
            else if (IsTargetLoss)
            {
                result.LifecycleMetrics["ActualSpeedBeforeLoss"] = _lossActualSpeed;
                result.LifecycleMetrics["DesiredSpeedBeforeLoss"] = _lossDesiredSpeed;
                Check(result, _lostDuringPursuit && _hadTargetBeforeLoss && _targetPresenceViolations == 0,
                    $"TARGET LOST: clear must interrupt active pursuit after meaningful travel, keep target/surface absent and F3/Enabled ON until reacquire; flag/plan violations={_targetPresenceViolations}.");
                Check(result, _name == "target-lost-reacquire-boundary" ? !_lostWhileMoving : _lostWhileMoving,
                    "TARGET LOST SETUP: moving variant must interrupt body movement; boundary variant must interrupt the gait's transient physical stop.");
            }
            else if (_name == "detour-retarget")
            {
                result.LifecycleMetrics["RetargetFrame"] = _retarget;
                result.LifecycleMetrics["DetourFramesBeforeRetarget"] = _detourFramesBefore;
                result.LifecycleMetrics["StalePlanFramesAfterRetarget"] = _stalePlanFrames;
                result.LifecycleMetrics["DetourSideChangesWithinTarget"] = _sideChanges;
                Check(result, _retarget >= 0 && _detourFramesBefore >= 15, "RETARGET: must replace target during a real committed detour after lateral movement.");
                Check(result, _invalidatedOnRetarget && _stalePlanFrames == 0,
                    $"RETARGET: old route/side cache must be invalidated on the first update, never steer a stale plan; stale frames={_stalePlanFrames}.");
                Check(result, _sideChanges == 0, $"RETARGET: detour mixed lateral sides within a fixed target on {_sideChanges} frames (one reset is allowed at retarget).");
            }
        }

        private static bool IsChasing(SpiderAiState state) => state is SpiderAiState.Pursuing or SpiderAiState.Navigating;
        private void Event(Result result, int frame, string message, Fixture world)
        {
            string entry = $"{frame}: {message}; phase={Phase}; state={world.Patrol.State}; target={V(Target)}; position={V(world.Position)}";
            result.LifecycleEvents.Add(entry);
            Console.WriteLine($"EVENT {_name} {entry}");
        }
    }

    private sealed class Fixture : IDisposable
    {
        public readonly PhysicsWorld Physics = new(new Vector3(0, -9.81f, 0));
        public readonly SceneManager Scene;
        public RigidBody? TargetSupport { get; private set; }
        public readonly Vector3 TargetSupportCenter = new(0, 1.15f, 0.9f);
        public readonly Vector3 TargetSupportHalfExtents = new(0.18f, 0.05f, 0.18f);
        public readonly SpiderEnemy Enemy;
        public readonly RigidBody Ground;
        public readonly RigidBody? Obstacle;
        public readonly Vector3 ObstacleCenter;
        public readonly Vector3 ObstacleHalfExtents;
        public readonly SpiderSurfaceSolver Solver;
        public readonly SpiderSurfaceMotor Motor;
        public readonly ProceduralSpiderWalk Gait;
        public readonly SpiderPatrol Patrol;
        public RigidBody Body => Enemy.Body;
        public Vector3 Position => Body.Position(Physics);

        public Fixture(Skeleton skeleton, string name)
        {
            var profile = SpiderLocomotionProfile.Default;
            var scene = Scene = new SceneManager(Physics, null!);
            Ground = new RigidBody().SetBox(new Vector3(100, 0.5f, 100)).SetPosition(new Vector3(0, -0.5f, 0)).SetMass(0);
            Ground.Build(Physics);
            Enemy = new SpiderEnemy($"ai-headless-{name}", assets: null);
            Vector3 spawn = Vector3.UnitY * profile.BodyClearance;
            Body.SetCapsule(profile.BodyRadius, profile.BodyCylinderHeight).SetPosition(spawn)
                .SetKinematic(true).SetFriction(0.5f).SetRestitution(0.1f).Build(Physics);
            if (!Body.IsBuilt || !Ground.IsBuilt) throw new InvalidOperationException("Cannot create Jolt test bodies.");
            typeof(SpiderEnemy).GetProperty(nameof(SpiderEnemy.Entity))!.GetSetMethod(true)!
                .Invoke(Enemy, [scene.ActiveScene.Add(null, Enemy.Id, Body)]);
            scene.NonWalkableBodies.Add(Body.Native);
            if (name is "low-box" or "blocked" or "detour-retarget")
            {
                bool lowBox = name != "blocked";
                ObstacleHalfExtents = lowBox ? new(1.25f, 0.35f, 1.25f) : new(4, 3, 3);
                ObstacleCenter = lowBox ? new(0, 0.35f, 8) : new(0, 3, 18);
                Obstacle = new RigidBody().SetBox(ObstacleHalfExtents).SetPosition(ObstacleCenter).SetMass(0);
                Obstacle.Build(Physics);
            }
            Solver = new SpiderSurfaceSolver(scene, profile);
            Motor = new SpiderSurfaceMotor(Physics, Body, Solver, spawn, Vector3.UnitY, Vector3.UnitZ);
            Gait = new ProceduralSpiderWalk(scene, Solver, profile);
            InitializeLegs(Gait, skeleton);
            Motor.BodyPoseValidator = Gait.CanOccupyPose;
            Motor.BodyPoseFailureDescription = () => Gait.LastBodyBlockReason;
            Motor.PoseReset += Gait.ResetContacts;
            Patrol = new SpiderPatrol(Enemy, Physics, Motor, Solver)
            {
                MinWaitTime = 0.1f, MaxWaitTime = 0.1f, MinTravelDistance = 6f, PatrolRadius = 12f
            };
            SetField(Enemy, "_surfaceSolver", Solver);
            SetField(Enemy, "_surfaceMotor", Motor);
            SetField(Enemy, "_proceduralWalk", Gait);
            SetField(Enemy, "_spiderPatrol", Patrol);
        }

        public SceneRaycastHit ResolveElevatedTarget()
        {
            // Insert only after settling, so this regression starts with a valid lower-floor stance.
            TargetSupport = new RigidBody().SetBox(TargetSupportHalfExtents).SetPosition(TargetSupportCenter).SetMass(0);
            TargetSupport.Build(Physics);
            if (!TargetSupport.IsBuilt || !Scene.Raycast(new Vector3(0, 2.2f, 0.9f), -Vector3.UnitY, 2f,
                out SceneRaycastHit hit, Body.Native, walkableOnly: true) || hit.BodyID != TargetSupport.Native ||
                MathF.Abs(hit.Position.Y - 1.2f) > SpiderLocomotionProfile.Default.ContactToleranceWorld ||
                Vector3.Dot(hit.Normal, Vector3.UnitY) < 0.98f)
                throw new InvalidOperationException("Cannot resolve the real elevated support at y=1.2, z=0.9.");
            return hit;
        }

        public (double PatrolMs, double GaitMs) Tick()
        {
            long start = Stopwatch.GetTimestamp();
            Patrol.Update(Dt);
            double patrolMs = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
            var pose = new SpiderLocomotionPose(Position, Body.Rotation(Physics), Motor.CurrentVelocity,
                Motor.CurrentAngularVelocity, SpiderLocomotionProfile.Default.VisualScale, Body.Native,
                Motor.SurfaceContact, Motor.SupportVelocity, Motor.SupportAngularVelocity, Motor.DesiredRelativeVelocity);
            start = Stopwatch.GetTimestamp();
            Gait.Update(Dt, pose);
            double gaitMs = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
            Physics.Step(Dt);
            return (patrolMs, gaitMs);
        }

        public void Dispose()
        {
            Motor.PoseReset -= Gait.ResetContacts;
            Motor.Dispose();
            Solver.Dispose();
            foreach (RigidBody body in Obstacle == null ? new[] { Body, Ground } : new[] { Body, Obstacle, Ground })
            {
                Physics.DestroyBody(body.Native);
                body.Destroy();
            }
            if (TargetSupport != null)
            {
                Physics.DestroyBody(TargetSupport.Native);
                TargetSupport.Destroy();
            }
            Physics.Dispose();
        }
    }

    // Minimum of the real-rig initializer in SpiderLocomotion.Integration; no synthetic feet or bypassed IK.
    private static void InitializeLegs(ProceduralSpiderWalk gait, Skeleton skeleton)
    {
        Type type = typeof(SpiderEnemy).GetNestedType("LegData", BindingFlags.NonPublic)
            ?? throw new MissingMemberException("SpiderEnemy.LegData");
        Array data = Array.CreateInstance(type, 8);
        for (int i = 0; i < 8; i++)
        {
            string side = i < 4 ? "L" : "R";
            int n = i % 4;
            string[] names = [$"{side}.thigh.{n}", $"{side}.Leg.{n}", $"{side}.foot.{n}", $"{side}.toes.{n}"];
            int[] indices = names.Select(name => skeleton.TryGetNodeIndex(name, out int index) ? index :
                throw new InvalidOperationException($"Missing rig node {name}")).ToArray();
            object leg = Activator.CreateInstance(type)!;
            type.GetField("ThighNodeIndex", Members)!.SetValue(leg, indices[0]);
            type.GetField("SegmentNodeIndices", Members)!.SetValue(leg, indices[1..]);
            data.SetValue(leg, i);
        }
        typeof(ProceduralSpiderWalk).GetMethod("Initialize", Members)!.Invoke(gait, [skeleton, data]);
    }

    private static float CapsulePenetration(Vector3 center, Vector3 up, SpiderLocomotionProfile profile, Vector3 boxCenter, Vector3 half)
    {
        Vector3 a = center - up * (profile.BodyCylinderHeight * 0.5f);
        Vector3 b = center + up * (profile.BodyCylinderHeight * 0.5f);
        float DistanceSquared(float t)
        {
            Vector3 outside = Vector3.Max(Vector3.Abs(Vector3.Lerp(a, b, t) - boxCenter) - half, Vector3.Zero);
            return outside.LengthSquared();
        }
        // Squared segment-to-AABB distance is convex, including a zero plateau inside the box.
        float lo = 0, hi = 1;
        for (int i = 0; i < 32; i++)
        {
            float left = lo + (hi - lo) / 3, right = hi - (hi - lo) / 3;
            if (DistanceSquared(left) <= DistanceSquared(right)) hi = right; else lo = left;
        }
        return MathF.Max(0, profile.BodyRadius - MathF.Sqrt(MathF.Min(DistanceSquared(0), MathF.Min(DistanceSquared(1), DistanceSquared((lo + hi) * 0.5f)))));
    }

    private static object Field(object value, string name) => value.GetType().GetField(name, Members)?.GetValue(value)
        ?? throw new MissingFieldException(value.GetType().FullName, name);
    private static T Read<T>(object value, string name) => (T)Field(value, name);
    private static void SetField(object value, string name, object data) => (value.GetType().GetField(name, Members)
        ?? throw new MissingFieldException(value.GetType().FullName, name)).SetValue(value, data);
    private static float TangentDistance(Vector3 a, Vector3 b) => new Vector2(a.X - b.X, a.Z - b.Z).Length();
    private static bool Finite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
    private static bool Finite(Quaternion q) => float.IsFinite(q.X) && float.IsFinite(q.Y) && float.IsFinite(q.Z) && float.IsFinite(q.W);
    private static string V(Vector3 v) => $"({v.X:F4},{v.Y:F4},{v.Z:F4})";
    private static void Check(Result result, bool condition, string message) { if (!condition) result.Failures.Add(message); }
    private static string Hash(string path) => Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path)));
    private static string Option(string[] args, string name, string fallback)
    {
        int index = Array.IndexOf(args, name);
        return index < 0 ? fallback : index + 1 < args.Length ? args[index + 1] : throw new ArgumentException($"Missing value for {name}");
    }
    private static string FindRoot()
    {
        for (DirectoryInfo? directory = new(AppContext.BaseDirectory); directory != null; directory = directory.Parent)
            if (File.Exists(Path.Combine(directory.FullName, "Fuse", "Fuse.csproj"))) return directory.FullName;
        throw new DirectoryNotFoundException("Cannot locate Fuse/Fuse.csproj.");
    }

    private sealed record Statistics(int Count, double P95, double Maximum, double Mean)
    {
        public static Statistics Of(IEnumerable<double> source)
        {
            double[] values = source.Order().ToArray();
            return new(values.Length, values[(int)Math.Ceiling(values.Length * 0.95) - 1], values[^1], values.Average());
        }
    }

    private sealed class Result
    {
        public string Case { get; set; } = "";
        public int Frames { get; set; }
        public Vector3 Start { get; set; }
        public Vector3 End { get; set; }
        public float PathLength { get; set; }
        public float MinimumDistance { get; set; } = float.MaxValue;
        public float FinalDistance { get; set; }
        public float MaximumFrameDisplacement { get; set; }
        public float MaximumSpeed { get; set; }
        public float MaximumHeightError { get; set; }
        public float MaximumBodyPenetration { get; set; }
        public int MinimumPlanted { get; set; } = 8;
        public int NonFiniteFrames { get; set; }
        public int BlockedFrames { get; set; }
        public int SurfaceLostFrames { get; set; }
        public int DetourFrames { get; set; }
        public int BudgetExhaustedFrames { get; set; }
        public float StoppedDrift { get; set; }
        public float StoppedPath { get; set; }
        public float StoppedMaximumSpeed { get; set; }
        public double? MovingTailMeanDistance { get; set; }
        public float MaximumLateralExcursion { get; set; }
        public float PostSwitchPath { get; set; }
        public string F3OverrideExpectation { get; set; } = "resume";
                public Dictionary<string, int> StateFrames { get; set; } = new();
                public List<string> StateTransitions { get; set; } = new();
                public List<string> LifecycleEvents { get; set; } = new();
                public Dictionary<string, double> LifecycleMetrics { get; set; } = new();
        public string? F3AfterF6Observation { get; set; }
        public Statistics? UpdateMs { get; set; }
        public Statistics? PatrolMs { get; set; }
        public Statistics? GaitMs { get; set; }
        public Statistics? ShapeQueries { get; set; }
        public Statistics? Raycasts { get; set; }
        public Dictionary<string, int> BodyReasons { get; set; } = new();
        public List<string> Failures { get; set; } = new();
    }
}
