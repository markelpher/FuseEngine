# Spider native-resource regression tests

Standalone `net10.0-windows` console referencing the real Fuse project and its installed Jolt bindings. No window, OpenGL context, audio, imported rig or procedural gait is created. Synthetic one-node skeletons exercise damage bodies and the fallback death body. The disabled-patrol test constructs the real motor/patrol on a Jolt floor, but does not execute pursuit navigation or require a rendered entity.

```sh
dotnet build tests/SpiderNativeResources.Tests/SpiderNativeResources.Tests.csproj
dotnet run --no-build --project tests/SpiderNativeResources.Tests/SpiderNativeResources.Tests.csproj
```

Use an external timeout of 120 seconds. Exit code 0 means all checks passed; 1 means a failed assertion or infrastructure/native dependency error. Build artifacts are the only files written.

Checks cover:

- Ownership of all default filters, all three EnemyBodyFilter constructors and SurfaceBodyFilter.
- Callback userdata is no longer GC-rooted after repeated disposal. The test tracks the userdata delegate weakly before disposal and forces GC; it never dereferences a freed GCHandle.
- The production BodyCreationSettings factory owns its native handle, supports repeated disposal and leaves the independently owned shape usable.
- Raycast callbacks and BodyLock borrowed wrappers remain usable after temporary filters/settings are disposed.
- A full 1024-body pool leaves failed RigidBody builds unbuilt, invalid and without a retained shape; retry succeeds after freeing a slot.
- Spider initialization fails with a managed, descriptive exception before touching assets/GL when its movement body cannot be created. EnemySystem.SpawnSpider returns null on repeated exhausted-pool attempts without changing PhysicsSystem.BodiesCount or registering entities, spiders, sensors or raycast exclusions. Damage-body failure can safely sync/dispose without sending invalid IDs to Jolt.
- Segment queries populate and reuse the solver's capsule/sphere cache and cached filters. Idempotent Dispose releases all cached native handles and callback userdata and empties the cache/eviction queue. Overlap, motion, shape-cast and support-probe calls then fail closed, including after BeginFrame/BeginGait resets. The post-disposal test nulls the scene's physics dependency by reflection, so accidental world access fails managed rather than entering native code with invalid handles.
- Navigation exhausts its one-third ray allowance; BeginMotor raises the cumulative limit to two-thirds without resetting the counter, and BeginGait exposes the full remainder without exceeding the profile cap. This test consumes TakeRaycast by reflection to isolate budget accounting from geometry.
- With SpiderPatrol.Enabled=false, 120 actual patrol updates on a Jolt floor each start with both full-update budgets exhausted. Only patrol.Update may reset/prepare the frame and run the motor. Every update must perform fresh support probes, leave budgets unexhausted, retain floor contact with zero lost-surface grace time, and command zero movement speed. No fake gait or entity is used. The static Enabled flag is restored in finally.
- EnemySystem.Clear removes movement/damage registrations and cached raycast exclusions. Death unregisters the live body while keeping the entity visible until ragdoll cleanup. Scene.Remove also handles already-invalidated wrappers.

Reflection is limited to assertions about binding ownership/userdata, the private production settings factory, synthetic enemy setup, solver cache inspection, budget counter consumption/assertions and the post-disposal null-physics sentinel. No binding version-specific native layout or allocator address is inspected. The ownership workaround is specific to JoltPhysicsSharp 2.21.0; rerun these tests when upgrading the package.
