# Xovium Model Editor

[![CI](https://github.com/Xovium-Tech/xovium-model-editor/actions/workflows/ci.yml/badge.svg)](https://github.com/Xovium-Tech/xovium-model-editor/actions/workflows/ci.yml)
[![Latest release](https://img.shields.io/github/v/release/Xovium-Tech/xovium-model-editor)](https://github.com/Xovium-Tech/xovium-model-editor/releases/latest)

A desktop vehicle editor for Xovium. Open an existing vehicle or import a 3D asset, attach sensors, move and rotate their mounting points in 3D, and save a portable `.xovmodel` project or export simulator assets.

## Supported formats and platforms

The supported desktop build is **Ubuntu 24.04 x86-64**, with an OpenGL 3.3 display. Imports include SDF vehicle models and Assimp-supported OBJ, DAE, STL, FBX, glTF and GLB meshes. Mesh format support depends on the installed Assimp version; unsupported SDF constructs produce an import error or an explicit import note.

| Export target | Output and sensor support |
| --- | --- |
| Gazebo Harmonic / Jetty | Model SDF, model.config, local resources and world-system setup. Native sensor definitions; existing vehicle joints and plugins are retained. |
| Isaac Sim / lightweight Isaac viewer | USD meshes, mounting hierarchy and camera prims. RGB cameras include Xovium streaming attributes. Other sensor definitions remain in metadata. |
| Unreal Engine | OBJ meshes, scene description and Unreal Editor Python importer for visual meshes and cameras. |
| FlightGear | AC3D visual model, XML wrapper and sensor metadata. |

Every export contains `report.json` describing native components, retained metadata and required runtime integration. Exporting a sensor definition does not automatically connect it to PX4 or implement its measurements in another simulator.

## 1. Dependencies

```bash
sudo apt update
sudo apt install build-essential cmake git python3 pkg-config libassimp-dev libpng-dev \
  libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev
```

Zenity is optional for native file dialogs (`sudo apt install zenity`); paths can also be entered directly. Build scripts download pinned raylib, Dear ImGui, rlImGui, ImGuizmo and nlohmann/json sources. They do not install system packages or require Blender, Isaac, Unreal or Gazebo to run the editor.

## 2. Get the source and build

```bash
git clone https://github.com/Xovium-Tech/xovium-model-editor.git
cd xovium-model-editor
./build.sh
```

The editor and mesh importer are built in `.build/cmake`. To keep build outputs elsewhere:

```bash
XOVIUM_BUILD_DIR=../xov-tests/model-editor/build \
XOVIUM_DEPS_DIR=../xov-tests/model-editor/deps ./build.sh
```

Use the same environment variables with `run.sh` when using a custom build directory.

## 3. Use the editor

```bash
./run.sh
# Or open a vehicle/project immediately:
./run.sh /path/to/model.sdf
./run.sh /path/to/vehicle.xovmodel
```

1. Choose **New** or **Open model**. Existing Xovium vehicles are under `xovium/shared/models/cessna/source/model.sdf` and `xovium/shared/models/rover/source/model.sdf`.
2. Add airspeed, GPS/GNSS, barometer, IMU, magnetometer, RGB camera, depth camera, LiDAR or contact sensors. Select the parent link in the inspector.
3. Use **Move** or **Rotate** handles in the viewport, or enter position and rotation values. Local/world handles and snapping are available. Camera frustums and the camera preview show the mounting direction.
4. Set sampling rate, topic and sensor-specific parameters. Add simulator plugin XML when the target simulator provides the corresponding plugin library.
5. **Save .xovmodel** packages the project and its resources. **Export** creates simulator assets in a new or empty directory and reports conversion limits.

Right-drag orbits the viewport, middle-drag pans, and the wheel zooms. **F** frames the model. **Ctrl+S** saves, **Ctrl+Z/Ctrl+Y** undo/redo, and **Delete** removes a selected sensor or plugin. The camera preview is a mounting preview, not simulated sensor output.

Coordinates use metres with **X forward, Y left, Z up**. The inspector displays degrees; stored rotations use roll, pitch and yaw in radians. Sensors remain attached to their chosen link.

### Command line

```bash
./run.sh --cli new --workspace /tmp/my-vehicle --name MyVehicle --vehicle fixed-wing
./run.sh --cli import /path/to/model.sdf --workspace /tmp/imported-vehicle
./run.sh --cli validate --workspace /tmp/imported-vehicle
./run.sh --cli save --workspace /tmp/imported-vehicle --output /path/to/vehicle.xovmodel
./run.sh --cli export --workspace /tmp/imported-vehicle --target gazebo --output /path/to/empty-export
```

Targets are `gazebo`, `isaac`, `sih`, `unreal` and `flightgear`. Use `./run.sh --cli --help` for all options. Imported models and source files are read without modification. Save/export writes are transactional; an export refuses to overwrite a nonempty directory.

## Project format and interoperability

`.xovmodel` is a ZIP bundle containing versioned `project.json` and local assets. The document stores links, visuals, inertial properties, sensors, plugins and optional joints. Imported SDF is retained for dynamics and extension settings. The JSON schema is in [schema/model.schema.json](schema/model.schema.json).

A Gazebo export can be selected as a custom `model.sdf` in Xovium Installer. The current installer converts geometry and uses its chosen Cessna/rover dynamics adapter for native simulators; it does not automatically translate arbitrary sensor plugins. Use each export report to configure the simulator's sensor systems and bridge routing.

## Validation

CI builds the editor/importer on Ubuntu 24.04, checks source syntax, and exercises mesh import, validation, portable save and Gazebo export. Local validation also covers actual PX4 Cessna/rover imports, native SDF parsing, USD camera transforms, texture packaging and interactive viewport controls. Simulator runtime behavior requires testing in the corresponding simulator.

## Limitations

- Imported geometry alone does not define vehicle aerodynamics, actuators or a flight controller. New model exports need an appropriate dynamics/collision setup before simulation.
- PX4 Gazebo bridges expect specific sensor names, parent links and topics. Arbitrary sensor placement/naming requires matching bridge configuration.
- GPS, barometer, airspeed and other metadata-only exports need a runtime adapter in Isaac, Unreal or FlightGear. This editor does not modify installed simulator bridges.
- USD/native visual exports preserve static mounting hierarchy; they do not recreate arbitrary articulated dynamics. Unreal native packages require running the supplied importer in Unreal Editor. FlightGear exports require a separate aircraft/FDM setup.
- The viewport and visual converters use base-color textures. Other imported texture maps are retained in the project but are not fully recreated as simulator materials.
- Nested models, includes, unresolved external resources and unsupported SDF frame relationships are rejected instead of being silently flattened. Imported extension settings are retained where possible and listed in the export report.

## License

BSD 3-Clause. Copyright (c) 2026 Xovium Tech. See [LICENSE](LICENSE) and [third-party notices](THIRD_PARTY.md).
