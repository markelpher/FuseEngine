
# FUSE ENGINE : the source engine inspiration 

It's simply an engine I decided to make in my free time to study OpenGL.

I don't yet have a clear goal for what kind of game to create with this engine, but for now I'm focused on making a game about bhop.

! The project was ported from C++ to C# simply because I prefer C#. !

![Logo](https://file.garden/Z5fpJocFXF3TjRhg/Captura%20de%20tela%202026-06-26%20171524.png)
![Preview](https://file.garden/Z5fpJocFXF3TjRhg/fuse%20engine/screenshot_2026-06-29_11-46-03.png)
![Preview2](https://file.garden/Z5fpJocFXF3TjRhg/fuse%20engine/screenshot_2026-06-29_11-49-00.png)

## Installation

```bash
  git clone https://github.com/Krueels/FuseEngine.git
```
Choose the "blowtorch" project for the map editor, and the "Fuse" project to open the game.

You also can open the game directly from the map editor by pressing F5.

## Fuse: controls and shortcuts

### Gameplay

| Key/action | Function |
|---|---|
| `W` `A` `S` `D` | Move the player |
| `Mouse` | Control the camera when the cursor is captured |
| `Space` | Jump; move upward in noclip mode |
| `Left Ctrl` | Crouch; move downward in noclip mode |
| `Left Shift` | Sprint |
| `F` | Toggle the flashlight |
| `E` | Interact with the object being viewed or pick up/drop an object when unarmed |
| `Left mouse button` | Fire a weapon; throw the held object |
| `R` | Reload the equipped weapon |
| `1` | Equip the Glock |
| `2` | Equip the AK |
| `0` | Holster the weapon |
| `Esc` | Pause/unpause and release/capture the cursor |
| `Mouse wheel` | Adjust the camera FOV in 2-degree increments when the interface is not using the mouse |

### Interface and development tools

| Key | Function |
|---|---|
| `` ` `` | Open/close the console |
| `Insert` | Open/close the ImGui debug interface |
| `F1` | Toggle noclip mode |
| `F2` | Capture a screenshot |
| `F3` | Toggle spider pursuit; switching it on also enables spider AI |
| `F4` | Reload all shaders |
| `F5` | Reload the current map |
| `F6` | Pause/resume AI, including active spider pursuit |
| `F8` | Toggle enemy selection mode; requires the Debug Drawer to be active |
| `F9` | Toggle the Debug Drawer |
| `F10` | Toggle post-processing |
| `F12` | Toggle shadows |

### Test actions

These keys work during gameplay or when the weapon context is active:

| Key | Function |
|---|---|
| `G` | Create an explosion at the raycast hit point |
| `J` | Spawn an enemy at the raycast hit point |
| `V` | Spawn a spider at the raycast hit point |
| `T` | Apply a decal and play the spray sound at the raycast hit point |
| `Z` | Apply test damage to the player |

When an interface window is capturing the keyboard or mouse, the corresponding gameplay controls are blocked.

### Spider locomotion checks

The procedural spider uses alternating tetrads, with collision-checked steps and at least four supporting legs. To inspect it in game, spawn one with `V`, enable the Debug Drawer with `F9`, then use `F8` and click the enemy. The selection HUD shows AI state, actual/intended speed, leg support counts, body-motion rejection reasons and bounded shape/raycast counts. `F3` toggles pursuit; `F6` pauses/resumes AI without clearing the chase selection. Spiders brake and hold near the player, restart when the player moves away, and use committed local detours around obstacles. F3 OFF resumes local patrol. Player death disables pursuit; enable F3 again after respawn.

Headless regression checks (run from the repository root):

```sh
dotnet run --project tests/SpiderLocomotion.Tests
dotnet run --project tests/SpiderLocomotion.Integration -- --case all
dotnet run --project tests/SpiderNativeResources.Tests
dotnet run --project tests/SpiderTargetSurface.Tests
dotnet run --project tests/SpiderNavigation.Tests
dotnet run --project tests/SpiderAi.Integration -- --case all
```

The integration harness uses the real FBX rig and Jolt, without a rendering window, and reports per-fixed-update CPU cost, allocations and query budgets. For a sustained obstruction test, use `--case blocked --frames 3600`. The native-resource harness checks ownership/disposal and safe failure when the physics body pool is full. See `tests/SpiderLocomotion.Integration/README.md` for coverage and limitations; visual validation in game is still necessary.
