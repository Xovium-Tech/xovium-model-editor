# Third-party notices

The editor is distributed as source. The build downloads pinned dependencies and retains their upstream license files in the dependency cache.

| Dependency | License | Upstream |
| --- | --- | --- |
| raylib 5.5 | zlib | https://github.com/raysan5/raylib |
| Dear ImGui 1.92.5 docking | MIT | https://github.com/ocornut/imgui |
| rlImGui | zlib | https://github.com/raylib-extras/rlImGui |
| ImGuizmo | MIT | https://github.com/CedricGuillemet/ImGuizmo |
| nlohmann/json 3.11.3 | MIT | https://github.com/nlohmann/json |
| Assimp (system package) | BSD 3-Clause | https://github.com/assimp/assimp |
| libpng (system package) | libpng-2.0 | https://github.com/pnggroup/libpng |

See `app/dependencies.json` for exact downloaded revisions. Imported vehicle models, meshes and textures remain subject to their original licenses; importing or exporting them does not relicense them. Preserve their source license notices when sharing a project. The editor includes no PX4, Gazebo Fuel, Unreal or NVIDIA model assets in its repository.
