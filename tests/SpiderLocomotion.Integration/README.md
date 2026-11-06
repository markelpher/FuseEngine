# Spider locomotion integration checks

Headless `net10.0-windows` console referencing the real `Fuse` project. It imports `Fuse/res/skinned_models/spider.fbx` with `SkinnedModelLoader.LoadRig`, initializes all eight procedural legs by reflection, and runs the production motor, IK, collision solver and Jolt physics. No window, OpenGL, audio or GPU assets are created. Existing Assimp/Jolt native dependencies must be available; missing dependencies fail explicitly.

## Run

From the repository root, with an external timeout of 120 seconds recommended:

```sh
dotnet "C:/Program Files/dotnet/sdk/10.0.403/MSBuild.dll" tests/SpiderLocomotion.Integration/SpiderLocomotion.Integration.csproj -target:Build -nologo -verbosity:minimal -property:Configuration=Release
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case all --private-memory --output tests/SpiderLocomotion.Integration/results-all
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case floor --speed 2
```

Options:

- `--case floor|wall|ceiling|turn|stop|reverse|slow|blocked|all` (default `floor`). `all` includes the original cases and `blocked`.
- `--frames 180..300` (default 240), after 30 settling frames, at a fixed 1/60-second timestep. Only explicit `--case blocked` accepts `180..3600` (up to 60 simulated seconds); `all` and every other case retain the 180..300 range.
- `--speed` (default 9.5), with acceleration of 8 units/s². `slow` always uses 2.
- `--output <directory>` (default this project's `results/`). Reports in that directory are overwritten; use separate directories for before/after comparisons.
- `--max-update-ms <milliseconds>` (default 250): assert that **every motor + gait fixed update** stays below this responsiveness watchdog. It does not interrupt a hung call; retain the external 120-second timeout. This generous watchdog is not a 60 FPS budget.
- `--max-p95-ms <milliseconds>` (optional): assert the per-fixed-update p95 cost. Both timing limits must be positive and finite; neither changes the simulation.
- `--private-memory`: optionally sample process private bytes immediately before and after the observed loop, outside measured windows.
- Exit codes: 0 success, 1 failed invariant, 2 infrastructure/import/reflection error.

Rebuild after production changes; direct-DLL execution only tests the previously compiled assembly. The explicit MSBuild entry point above avoids the local `dotnet build`/`dotnet run` workload-logger shutdown crash. The harness disposes the production solver (cached filters/shapes) with `using`, after the motor and before the physics world.

## Scenarios and assertions

- Floor, wall and ceiling locomotion start already attached to static boxes; they do **not** test transitions between surfaces.
- `turn`: walk for one second, then request a 90° change while walking.
- `stop`: walk for 1.5 seconds, settle for 0.5 seconds, then check stationary body and feet with all eight supports planted.
- `reverse`: request a stationary 180° turn, with no translational command or permitted body drift beyond contact tolerance.
- `slow`: floor locomotion at 2 units/s.
- `blocked`: settle normally for 30 updates, then insert one static box with half-extents `(20,5,5)` just beyond all settled body/leg capsule endpoints (0.05-unit gap). Keep requesting forward movement into the box. Only this case uses the motor's existing `SetTransitionConstraint` to retain the floor plane rather than climb the obstacle. It must exercise rejected landing attempts and motor blocking, keep valid/finite poses and supports, and never cross the box's front plane beyond the unchanged contact tolerance. Limb endpoints plus radii bound whole segments, not just foot targets. Valid planted contacts may be on the ground or the box's front face. No minimum travel, travel ratio or completed step for every leg is required: remaining safely blocked is expected.

Every frame must retain eight valid legs, at least four supports, no more than four swing legs, and no mixture of the two tetrads. Expected groups are derived independently from authored indices: `{0,2,5,7}` and `{1,3,4,6}`. Safe subsets of a tetrad are allowed.

The harness checks the actual transformed skeleton tips against solver targets, body clearance, surface/contact validity, finite values, final physical heading, and completed steps for every leg. Walking scenarios require at least one unit of net displacement and at least 50% of commanded travel; this is a smoke-test threshold, not a promise of achieving the full commanded speed. Stopping drift cannot improve that ratio. Reverse uses stationary-turn assertions instead of walking-distance assertions. Only `blocked` uses the blocked-safety assertions above; the original walking/stop/reverse assertions are not relaxed or replaced.

Body/foot tolerances come from the unchanged production profile. Joint limits, support requirements and body validators are not disabled to make tests pass. Rejected IK/body/swing attempts remain in the reports even if recovery preserves all invariants.

## Fixed-update cost benchmark

The simulation timestep remains 1/60 s. After the unchanged 30 settling updates, every observed update contributes **one sample of motor cost + gait cost**, not a rendered-frame sample, and not the sum of their independently calculated percentiles. Timers bracket only `motor.Update` and `gait.Update`, in two windows; acquiring the pose, the existing reflection-based rejection/IK diagnostics, `physics.Step`, assertions, CSV/console output and metric reads are outside both windows. First-time diagnostics are deferred until immediately after the motor, before gait changes its state. No production solver/IK work is skipped. No forced GC, extra locomotion warmup or outlier removal is used; JIT/tiering, scheduling and GC pauses inside calls can affect results.

`COST` reports nearest-rank p95/max/mean milliseconds for the combined update, p95/max for each component, and updates over the 16.667 ms timestep. JSON also retains count, total and median. CSV has each update's raw timing, allocations, collections and optional solver counters.

Managed allocation deltas use `GC.GetAllocatedBytesForCurrentThread()` across the same two calls, excluding report/diagnostic/metric-read allocations. Total, p95 and max **bytes per fixed update** are reported. `GCCollections=[gen0,gen1,gen2]` sums `GC.CollectionCount` deltas across those windows: these are process-wide collection counts, not collections attributable only to the measured thread; higher generations also increment lower-generation counts. Native Jolt allocations and other threads are not included in managed bytes. Optional private-memory snapshots include the whole process and intervening reporting/physics work; their delta is not a leak assertion or a peak-memory measurement.

Cached optional accessors read `SpiderSurfaceSolver.FrameShapeQueries`, `FrameRaycasts`, `QueryBudgetExhausted` after gait, and `SpiderLocomotionProfile.MaximumShapeQueriesPerUpdate` / `MaximumRaycastsPerUpdate` before observation. Missing APIs report `n/a`/JSON `null`, never a fabricated zero. Available query counts have per-update p95/max (and full JSON statistics), plus the number of exhausted updates. Every available count must be nonnegative and at most its profile budget; profile budgets must stay within conservative caps of 1..512 shape queries and 1..192 raycasts. Counts are the **motor + gait total**: production `BeginFrame` resets them and reserves one third for the motor; `BeginGait` raises the limits to the full caps without resetting counters. Exhaustion can therefore occur below the full cap and is not itself a failure if safety is preserved. Accessor discovery and reflection fallback reads are outside measured windows.

For a stricter regression gate, for example (choose limits appropriate for the same machine/configuration):

```sh
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case blocked --max-p95-ms 8 --max-update-ms 50 --output tests/SpiderLocomotion.Integration/results-after-blocked
```

### Pre-optimization baseline — 2026-11-06

Sequential fresh-process Release runs, 240 observed updates each, speed 9.5, dt 1/60, private memory enabled, external timeout 120 s per command. Environment: .NET 10.0.14, Windows 10.0.26200, X64, 12 logical processors. Production assembly MVID `eae80af4-416b-4d6e-8246-ab886bc4f9f1` (SHA-256 prefix `5F47C078B7069F7C`). Production source hashes matched the original pre-instrumentation run and remained unchanged across both cases. These are single-run observations, not a rendered FPS claim or stable machine-independent thresholds.

| Case | Result | Fixed update p95 / max / mean (ms) | Motor p95 / max (ms) | Gait p95 / max (ms) | > dt |
| --- | --- | --- | --- | --- | --- |
| floor | PASS | 12.494 / 25.537 / 4.701 | 3.660 / 13.554 | 10.666 / 23.628 | 4/240 |
| blocked | PASS | 16.935 / 30.102 / 6.402 | 6.107 / 20.517 | 10.434 / 22.809 | 14/240 |

| Case | Managed bytes total | Bytes/update p95 / max | GC gen0/1/2 | Private bytes before → after |
| --- | --- | --- | --- | --- |
| floor | 191,705,168 | 2,028,216 / 2,796,232 | 11/1/0 | 32,112,640 → 48,422,912 |
| blocked | 232,868,568 | 2,224,256 / 2,739,384 | 14/1/0 | 32,870,400 → 46,501,888 |

- Original `floor` before instrumentation: PASS, displacement 28.662827, commanded-travel ratio 88.36%, total elapsed 1.35 s. Instrumented `floor` retained exactly the same displacement, ratio, landing counts and 17/240 motor-blocked updates; all original assertions passed.
- `blocked`: displacement 0.728056 (no minimum required), 213/240 motor-blocked updates, 17 rejected landing samples, 28 valid obstacle-contact samples, eight valid legs throughout, at least four supports, zero nonfinite/invalid-contact samples and zero penetration-violation frames. Maximum penetration 0.00759149 is below the unchanged 0.015 tolerance. Box front Z = 2.477224. The blocked leg need not complete a step (`landed=[0,11,15,15,7,13,15,15]`).
- Solver counters/profile budget: `n/a` in this baseline because the future API was absent.
- Saved full reports: `results-baseline-floor/` and `results-baseline-blocked/`; captured console output: `console.log` inside each directory. Environment reports include production/harness assembly hashes, source hashes and runtime/configuration. Generated reports are local artifacts; the concise baseline is retained here.

Commands used for the final comparable runs (rebuild after production edits, then use different output directories for the next comparison):

```sh
dotnet build -c Release tests/SpiderLocomotion.Integration/SpiderLocomotion.Integration.csproj --no-restore
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case floor --private-memory --output tests/SpiderLocomotion.Integration/results-baseline-floor
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case blocked --private-memory --output tests/SpiderLocomotion.Integration/results-baseline-blocked
```

The local CLI intermittently threw `ObjectDisposedException` from `Microsoft.DotNet.Cli.Installer.Windows.TimestampedFileLogger` on process exit, even after successful compilation or `RESULT PASS`. Both final direct-DLL runs exited **0**; do not treat a CLI shutdown crash as a successful command. A negative gate check with both timing limits set to 0.001 ms exited **1**, retaining the functional `floor` outcome while reporting both cost failures.

### Final optimized benchmark — 2026-11-06

Direct MSBuild Release build and all three final direct-DLL runs exited **0**. Each run used a **fresh process**, speed 9.5, 30 settling updates, `--private-memory`, and enforced **p95 ≤ 8 ms / max ≤ 50 ms**, with a 120-second external timeout. Production assembly MVID `c8060b93-d827-4451-8a1a-f49e3ce3d277` (SHA-256 prefix `495CAE297EA7`). All final processes used identical production/harness assemblies and source hashes; no source changes were detected during measurement.

| Case | Updates | Result | Motor + gait p95 / max / mean (ms) | Managed bytes total | Bytes/update p95 / max | GC 0/1/2 |
| --- | --- | --- | --- | --- | --- | --- |
| floor | 240 | PASS | 4.161 / 11.249 / 2.028 | 14,581,272 | 163,224 / 186,216 | 1/1/0 |
| blocked | 240 | PASS | 5.960 / 10.390 / 2.946 | 16,310,336 | 188,240 / 266,872 | 1/1/0 |
| blocked long | 3600 | PASS | 2.278 / 9.651 / 0.824 | 245,234,672 | 184,400 / 266,872 | 15/1/0 |

| Case | Shapes p95 / max | Raycasts p95 / max | Exhausted updates | Private bytes before → after |
| --- | --- | --- | --- | --- |
| floor | 218 / 326 | 110 / 110 | 4/240 | 28,299,264 → 45,174,784 |
| blocked | 512 / 512 | 110 / 192 | 216/240 | 28,180,480 → 45,002,752 |
| blocked long | 512 / 512 | 110 / 192 | 3576/3600 | 28,688,384 → 51,879,936 |

All counters stayed within **512 shapes / 192 rays**; no update exceeded 16.667 ms. The long run includes **60 simulated seconds** (plus settling), completed in 3.28 s wall time. Long and short percentiles span different warmup proportions; every observed sample is retained. The long-run allocation total covers 15× as many updates and is not directly comparable to a 240-update total. Private-memory snapshots are not a peak/leak measurement.

Fresh-process 240-update comparison against the original baseline:

| Case | p95 ms baseline → final | Max ms baseline → final | Managed allocation reduction |
| --- | --- | --- | --- |
| floor | 12.494 → 4.161 (-66.70%) | 25.537 → 11.249 (-55.95%) | 191,705,168 → 14,581,272 bytes (-92.39%) |
| blocked | 16.935 → 5.960 (-64.81%) | 30.102 → 10.390 (-65.48%) | 232,868,568 → 16,310,336 bytes (-93.00%) |

`floor` retained **88.73%** commanded travel (28.781855 units; baseline 88.36%), with all eight legs completing steps. Both blocked runs retained eight valid legs and at least five supports, with zero nonfinite/invalid-contact samples, surface losses or penetration-violation frames; maximum penetration 0.00759149 stayed below the unchanged 0.015 tolerance. The 3600-update run remained at 0.733333 net displacement, with 3576 motor-blocked updates and 5676 rejected landing samples; walking progress is intentionally not required for `blocked`.

The intermediate budgeted diagnostic run had `turn` at 43.96% travel and failed the unchanged 50% threshold. Subsequent production geometric step-path prechecks and removal of redundant endpoint overlaps corrected that transient regression **without relaxing any assert**. The parent's saved `results-optimized/summary.json` was checked: **8/8 PASS**, `floor` 88.73%, `turn` 76.63%. That saved `all` result is distinct from the three fresh-process final benchmarks above; a new `all` run was not needed for the requested final measurements. Intermediate reports remain in `results-budgeted-*/` rather than replacing the baseline.

Final reports (`summary.json`, `environment.json`, per-update CSV and `console.log`): `results-final-floor/`, `results-final-blocked/`, `results-final-blocked3600/`. CSV/JSON timing/allocation statistics, GC totals, query counters, exhaustion counts and real 8/50 gates were cross-checked for all **4,080** observed updates. Frame-bound checks rejected `floor 301`, `all 3600`, `blocked 3601` and `blocked 179` with exit 2; the successful long run verified `blocked 3600` is accepted.

Reproduce after rebuilding with the MSBuild command above (use different output directories to preserve these reports):

```sh
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case floor --max-p95-ms 8 --max-update-ms 50 --private-memory --output tests/SpiderLocomotion.Integration/results-final-floor
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case blocked --max-p95-ms 8 --max-update-ms 50 --private-memory --output tests/SpiderLocomotion.Integration/results-final-blocked
dotnet tests/SpiderLocomotion.Integration/bin/Release/net10.0-windows/SpiderLocomotion.Integration.dll --case blocked --frames 3600 --max-p95-ms 8 --max-update-ms 50 --private-memory --output tests/SpiderLocomotion.Integration/results-final-blocked3600
```

## Reports and limitations

Each run produces `summary.json`, per-frame CSVs, and environment metadata with assembly/rig/source hashes. It also reports source changes detected during execution, since a running process uses the already loaded assembly. Output directories and build artifacts are ignored by Git.

The harness commands the motor directly, not the patrol/pursuit system. It does not test animated clips, rendered mesh deformation, damage hitboxes, ragdolls, jumping, general obstacle navigation, surface transitions or moving supports. The single blocked-box case is a safety/cost regression, not obstacle traversal coverage. Reflection intentionally fails if the internal leg initialization contract changes. Appearance and navigation still require in-game validation.
