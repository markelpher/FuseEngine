# Spider local navigation regression tests

Real Jolt queries against `Fuse`, without rendering, modifying existing tests or replacing motor/gait decisions. Tests cover low/off-centre obstacles, body clearance, floor/wall/ceiling frames, two committed route stages, target invalidation, cooldowns and the shared query caps.

```sh
dotnet run --project tests/SpiderNavigation.Tests/SpiderNavigation.Tests.csproj -c Release
```

The new production API is:

```csharp
bool SpiderSurfaceSolver.IsSameSurfacePathClear(
    Vector3 start, Vector3 end, Vector3 surfaceNormal, float clearance,
    BodyID selfBody, float maxDistance = float.MaxValue)
```

`start` is the spider body centre; the path is projected onto `surfaceNormal`. At most two shape queries and zero rays test the physical capsule and a conservative low feet/leg footprint. `false` includes invalid input and insufficient budget; it is not proof that a detour exists. This method does not establish continuous floor support. The existing public detour signature is preserved; its two-stage overload is internal to the planner.

A local search consumes at most 98 shapes (including the initial corridor check) and 25 rays, always within the current navigation-stage limits (default 170 shapes / 64 rays). Active plans do not query physics again. Existing `FrameShapeQueries`, `FrameRaycasts` and `QueryBudgetExhausted` remain the metrics for the shared frame, with no counter resets between navigation, motor and gait.

Routes are local approximations: at most 12 m lateral/forward offsets and two support samples per leg, not a navmesh, continuous support proof or rig-specific IK envelope. The conservative footprint uses profile clearance and transition lookahead. The parent still owns arrival/stopping hysteresis, F3/F6/state policy, plan lifecycle calls and constraining automatic surface transitions while detouring. Call `TryGetDirection` with the current target even when search is disabled: this also invalidates a committed plan after target displacement greater than `max(2 m, 3 * clearance)` and schedules a throttled retry. While `IsSameSurfaceDetourActive`, the parent should use the existing motor `SetTransitionConstraint` API with the source normal; `PlannedTransition` intentionally remains invalid. Do not cancel the route just because the front corridor becomes clear at its first lateral corner.

## Validation snapshot

- `dotnet run --project tests/SpiderNavigation.Tests/SpiderNavigation.Tests.csproj -c Release`: **11/11 PASS**. The floor/wall/ceiling low-box search uses **24 shapes / 7 rays** (30 shapes including three extra test verification corridors). A failed solid-target search uses 72 shapes / 1 ray. Thirty clear-floor calls interleaved with `ClearPlan` produce three searches, six shapes, zero rays.
- `dotnet build tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --nologo -v minimal`: **PASS**.
- Existing integration `--case all --output tests/SpiderAi.Integration/results-navigation-final`: **exit 1**, assertions unchanged. All scenarios retain the hard 512/192 caps. Low-box records 237 detour frames, 4.7928 m lateral excursion, passes the box, stays at floor clearance, has zero capsule penetration/support loss, maximum speed 9.50004 m/s, maximum displacement 0.158334 m/frame. It reaches the target region by six seconds, but the current parent's arrival/stop oscillation leaves a final distance of 1.311763 m and 2.306838 m of final-window path. Therefore **the full low-box scenario is not yet passing**. Other arrival scenarios still fail STOP; `blocked` still has a 15.847258 m/s automatic-transition spike. Those parent/motor policies were not moved into this local planner to hide the failures.
