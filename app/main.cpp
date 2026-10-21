#include "imgui.h"
// ImGuizmo requires the ImGui declarations above.
#include "ImGuizmo.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

using Json = nlohmann::json;
namespace fs = std::filesystem;
namespace ui = ImGui;
#ifndef XOVIUM_SOURCE_DIR
#define XOVIUM_SOURCE_DIR "."
#endif
#ifndef XOVIUM_MESH_IMPORT_DEFAULT
#define XOVIUM_MESH_IMPORT_DEFAULT "mesh-import"
#endif
static const ImVec4 accent{.25f, .75f, .67f, 1}, muted{.52f, .61f, .66f, 1};
struct Result {
    int code = 0;
    std::string text;
};
Result command(const std::vector<std::string> &args) {
    int pipes[2];
    if (pipe(pipes))
        return {1, "Cannot create process pipe"};
    pid_t child = fork();
    if (child == 0) {
        close(pipes[0]);
        dup2(pipes[1], 1);
        dup2(pipes[1], 2);
        close(pipes[1]);
        std::vector<char *> values;
        for (const auto &a : args)
            values.push_back(const_cast<char *>(a.c_str()));
        values.push_back(nullptr);
        execvp(values[0], values.data());
        _exit(127);
    }
    close(pipes[1]);
    if (child < 0) {
        close(pipes[0]);
        return {1, "Cannot launch process"};
    }
    std::string out;
    char bytes[8192];
    ssize_t count;
    while ((count = read(pipes[0], bytes, sizeof(bytes))) > 0)
        if (out.size() < 4 * 1024 * 1024)
            out.append(bytes, count);
    close(pipes[0]);
    int status = 0;
    while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
    }
    return {WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status), out};
}
fs::path temporary() {
    std::string name = (fs::temp_directory_path() / "xovium-model-editor-XXXXXX").string();
    std::vector<char> b(name.begin(), name.end());
    b.push_back(0);
    if (!mkdtemp(b.data()))
        throw std::runtime_error("Cannot create workspace");
    return b.data();
}
Json readJson(const fs::path &p) {
    std::ifstream in(p);
    if (!in)
        throw std::runtime_error("Cannot read " + p.string());
    Json j;
    in >> j;
    return j;
}
void writeJson(const fs::path &p, const Json &j) {
    std::ofstream out(p);
    out << j.dump(2) << '\n';
    if (!out)
        throw std::runtime_error("Cannot write " + p.string());
}
std::array<float, 16> elements(Matrix m) {
    return {m.m0, m.m1, m.m2,  m.m3,  m.m4,  m.m5,  m.m6,  m.m7,
            m.m8, m.m9, m.m10, m.m11, m.m12, m.m13, m.m14, m.m15};
}
Matrix matrix(const std::array<float, 16> &a) {
    Matrix m{};
    m.m0 = a[0];
    m.m1 = a[1];
    m.m2 = a[2];
    m.m3 = a[3];
    m.m4 = a[4];
    m.m5 = a[5];
    m.m6 = a[6];
    m.m7 = a[7];
    m.m8 = a[8];
    m.m9 = a[9];
    m.m10 = a[10];
    m.m11 = a[11];
    m.m12 = a[12];
    m.m13 = a[13];
    m.m14 = a[14];
    m.m15 = a[15];
    return m;
}
Matrix product(Matrix a, Matrix b) {
    auto x = elements(a), y = elements(b);
    std::array<float, 16> z{};
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            for (int k = 0; k < 4; k++)
                z[c * 4 + r] += x[k * 4 + r] * y[c * 4 + k];
    return matrix(z);
}
Matrix pose(const Json &p) {
    float x = p.at(0), y = p.at(1), z = p.at(2), r = p.at(3), t = p.at(4), w = p.at(5);
    float cr = cosf(r), sr = sinf(r), cp = cosf(t), sp = sinf(t), cy = cosf(w), sy = sinf(w);
    Matrix m = MatrixIdentity();
    m.m0 = cy * cp;
    m.m1 = sy * cp;
    m.m2 = -sp;
    m.m4 = cy * sp * sr - sy * cr;
    m.m5 = sy * sp * sr + cy * cr;
    m.m6 = cp * sr;
    m.m8 = cy * sp * cr + sy * sr;
    m.m9 = sy * sp * cr - cy * sr;
    m.m10 = cp * cr;
    m.m12 = x;
    m.m13 = y;
    m.m14 = z;
    return m;
}
Json unpose(Matrix m) {
    float pitch = asinf(std::clamp(-m.m2, -1.f, 1.f)), roll, yaw;
    if (fabsf(cosf(pitch)) > 1e-5f) {
        roll = atan2f(m.m6, m.m10);
        yaw = atan2f(m.m1, m.m0);
    } else {
        roll = 0;
        yaw = atan2f(-m.m4, m.m5);
    }
    return Json::array({m.m12, m.m13, m.m14, roll, pitch, yaw});
}
Vector3 location(Matrix m) {
    return {m.m12, m.m13, m.m14};
}
Vector3 point(Matrix m, Vector3 p) {
    return Vector3Transform(p, m);
}
Color tint(const Json &j, Color fallback = WHITE) {
    if (!j.is_array() || j.size() < 3)
        return fallback;
    return {static_cast<unsigned char>(std::clamp(j[0].get<float>(), 0.f, 1.f) * 255),
            static_cast<unsigned char>(std::clamp(j[1].get<float>(), 0.f, 1.f) * 255),
            static_cast<unsigned char>(std::clamp(j[2].get<float>(), 0.f, 1.f) * 255), 255};
}
bool textField(const char *label, Json &j, size_t capacity = 256, bool multiline = false) {
    std::vector<char> b(std::max(capacity, j.is_string()
                                               ? j.get_ref<const std::string &>().size() + 128
                                               : capacity),
                        0);
    if (j.is_string()) {
        const auto &s = j.get_ref<const std::string &>();
        std::copy(s.begin(), s.end(), b.begin());
    }
    ui::TextUnformatted(label);
    std::string key = "##" + std::string(label);
    bool changed = multiline ? ui::InputTextMultiline(key.c_str(), b.data(), b.size(), {0, 140})
                             : ui::InputText(key.c_str(), b.data(), b.size());
    if (changed)
        j = std::string(b.data());
    return changed;
}
struct MeshView {
    Model model{};
    int visual = 0;
    Texture2D texture{};
};
struct Editor {
    fs::path root, workspace, helper, savedFile;
    Json doc, catalog;
    std::vector<MeshView> meshes;
    std::vector<Json> undo, redo;
    Camera3D camera{};
    RenderTexture2D viewport{}, preview{};
    int viewportW = 0, viewportH = 0;
    Vector3 target{};
    float azimuth = -2.25f, elevation = .45f, distance = 7;
    int selected = -1;
    std::string selection = "sensor";
    int operation = 0;
    bool local = true, snap = false, wire = false, dirty = false, gizmoUsing = false,
         editActive = false;
    bool cameraPreview = true, closeWanted = false;
    std::string status = "Ready", lastError, report;
    std::future<Result> task;
    std::string taskKind;
    fs::path taskWorkspace, taskPath;
    bool busy = false;
    unsigned long undoSerial = 0;
    std::string modal;
    char pathBuffer[4096]{};
    int exportTarget = 0;
    std::string screenshot;
    int maxFrames = 0, frames = 0;
    bool smoke = false;
    Editor(fs::path source) : root(std::move(source)) {
        helper = getenv("XOVIUM_MESH_IMPORT") ? getenv("XOVIUM_MESH_IMPORT")
                                              : XOVIUM_MESH_IMPORT_DEFAULT;
        workspace = temporary();
    }
    ~Editor() {
        if (busy)
            task.wait();
        for (auto &m : meshes) {
            if (m.texture.id)
                UnloadTexture(m.texture);
            UnloadModel(m.model);
        }
        if (viewport.id)
            UnloadRenderTexture(viewport);
        if (preview.id)
            UnloadRenderTexture(preview);
        std::error_code ec;
        fs::remove_all(workspace, ec);
        if (!taskWorkspace.empty() && taskWorkspace != workspace)
            fs::remove_all(taskWorkspace, ec);
    }
    std::vector<std::string> backend(std::vector<std::string> args) {
        std::vector<std::string> c = {"python3", "-B", (root / "backend/editor.py").string()};
        c.insert(c.end(), args.begin(), args.end());
        return c;
    }
    void init() {
        catalog = readJson(root / "backend/sensors.json");
        Result r = command(backend({"new", "--workspace", workspace.string(), "--name",
                                    "Untitled vehicle", "--vehicle", "fixed-wing"}));
        if (r.code)
            throw std::runtime_error(r.text);
        doc = readJson(workspace / "project.json");
        reload();
    }
    void remember() {
        ++undoSerial;
        undo.push_back(doc);
        if (undo.size() > 100)
            undo.erase(undo.begin());
        redo.clear();
        dirty = true;
    }
    void history(bool back) {
        auto &from = back ? undo : redo;
        auto &to = back ? redo : undo;
        if (from.empty() || busy)
            return;
        to.push_back(doc);
        doc = from.back();
        from.pop_back();
        dirty = true;
        selected = -1;
        reload();
    }
    Matrix linkMatrix(const std::string &name) {
        for (const auto &l : doc["links"])
            if (l["name"] == name)
                return pose(l["pose"]);
        return MatrixIdentity();
    }
    Matrix sensorMatrix(const Json &s) {
        return product(linkMatrix(s.value("parent", "base_link")), pose(s["pose"]));
    }
    Matrix visualMatrix(int i) {
        const auto &v = doc["visuals"][i];
        Matrix m = product(linkMatrix(v.value("link", "base_link")), pose(v["pose"]));
        auto s = v.value("scale", Json::array({1, 1, 1}));
        return product(m, MatrixScale(s[0], s[1], s[2]));
    }
    void loadMesh(const Json &data, int visual, const fs::path &textureRoot) {
        const auto &vertices = data.at("vertices");
        const auto &faces = data.at("faces");
        if (faces.empty())
            return;
        if (faces.size() > 2000000)
            throw std::runtime_error("Mesh exceeds the editor's two million triangle limit");
        Mesh mesh{};
        mesh.vertexCount = static_cast<int>(faces.size() * 3);
        mesh.triangleCount = static_cast<int>(faces.size());
        mesh.vertices = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
        mesh.normals = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
        mesh.texcoords = (float *)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
        mesh.colors = (unsigned char *)MemAlloc(mesh.vertexCount * 4);
        auto uv = data.value("uv", Json::array());
        Color color = tint(data.value("color", Json::array()), {170, 187, 198, 255});
        size_t index = 0;
        for (const auto &f : faces) {
            Vector3 p[3];
            for (int c = 0; c < 3; c++) {
                size_t k = f.at(c).get<size_t>();
                const auto &v = vertices.at(k);
                p[c] = {v.at(0), v.at(1), v.at(2)};
            }
            Vector3 normal = Vector3Normalize(
                Vector3CrossProduct(Vector3Subtract(p[1], p[0]), Vector3Subtract(p[2], p[0])));
            float shade =
                .58f + .42f * fabsf(Vector3DotProduct(normal, Vector3Normalize({.4f, -.6f, .8f})));
            for (int c = 0; c < 3; c++, index++) {
                size_t k = f.at(c).get<size_t>();
                mesh.vertices[index * 3] = p[c].x;
                mesh.vertices[index * 3 + 1] = p[c].y;
                mesh.vertices[index * 3 + 2] = p[c].z;
                mesh.normals[index * 3] = normal.x;
                mesh.normals[index * 3 + 1] = normal.y;
                mesh.normals[index * 3 + 2] = normal.z;
                mesh.texcoords[index * 2] = uv.size() > k ? uv[k][0].get<float>() : 0;
                mesh.texcoords[index * 2 + 1] = uv.size() > k ? uv[k][1].get<float>() : 0;
                mesh.colors[index * 4] = (unsigned char)(color.r * shade);
                mesh.colors[index * 4 + 1] = (unsigned char)(color.g * shade);
                mesh.colors[index * 4 + 2] = (unsigned char)(color.b * shade);
                mesh.colors[index * 4 + 3] = 255;
            }
        }
        UploadMesh(&mesh, false);
        MeshView view;
        view.model = LoadModelFromMesh(mesh);
        view.visual = visual;
        std::string texture = data.value("texture", "");
        if (!texture.empty()) {
            fs::path p = textureRoot / texture;
            if (!fs::is_regular_file(p))
                p = workspace / texture;
            if (fs::is_regular_file(p)) {
                view.texture = LoadTexture(p.c_str());
                if (view.texture.id)
                    view.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = view.texture;
            }
        }
        meshes.push_back(view);
    }
    void reload() {
        for (auto &m : meshes) {
            if (m.texture.id)
                UnloadTexture(m.texture);
            UnloadModel(m.model);
        }
        meshes.clear();
        for (size_t i = 0; i < doc["visuals"].size(); i++) {
            const auto &v = doc["visuals"][i];
            const auto &g = v["geometry"];
            std::string type = g.value("type", "");
            if (type == "mesh") {
                fs::path p = workspace / g.at("path").get<std::string>();
                Json data = readJson(p);
                for (auto &item : data)
                    loadMesh(item, (int)i, p.parent_path());
            } else {
                Mesh mesh{};
                if (type == "box") {
                    auto s = g["size"];
                    mesh = GenMeshCube(s[0], s[1], s[2]);
                } else if (type == "sphere")
                    mesh = GenMeshSphere(g["radius"], 24, 24);
                else if (type == "cylinder") {
                    mesh = GenMeshCylinder(g["radius"], g["length"], 24);
                    for (int k = 0; k < mesh.vertexCount; k++) {
                        float y = mesh.vertices[k * 3 + 1], z = mesh.vertices[k * 3 + 2];
                        mesh.vertices[k * 3 + 1] = -z;
                        mesh.vertices[k * 3 + 2] = y - g["length"].get<float>() * .5f;
                    }
                    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float),
                                     0);
                } else
                    continue;
                MeshView item;
                item.model = LoadModelFromMesh(mesh);
                item.visual = (int)i;
                item.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color =
                    tint(v.value("color", g.value("color", Json::array())), {170, 189, 200, 255});
                meshes.push_back(item);
            }
        }
    }
    void frameModel() {
        BoundingBox box{{1e20f, 1e20f, 1e20f}, {-1e20f, -1e20f, -1e20f}};
        for (const auto &m : meshes)
            for (int mesh = 0; mesh < m.model.meshCount; mesh++) {
                auto bounds = GetMeshBoundingBox(m.model.meshes[mesh]);
                Matrix tr = visualMatrix(m.visual);
                for (int n = 0; n < 8; n++) {
                    Vector3 v = point(tr, {n & 1 ? bounds.max.x : bounds.min.x,
                                           n & 2 ? bounds.max.y : bounds.min.y,
                                           n & 4 ? bounds.max.z : bounds.min.z});
                    box.min = Vector3Min(box.min, v);
                    box.max = Vector3Max(box.max, v);
                }
            }
        if (meshes.empty())
            return;
        target = Vector3Scale(Vector3Add(box.min, box.max), .5f);
        distance = std::max(1.f, Vector3Distance(box.min, box.max) * 1.5f);
    }
    void start(std::string kind, std::vector<std::string> args) {
        if (busy)
            return;
        lastError.clear();
        status = kind + "…";
        taskKind = kind;
        busy = true;
        auto c = backend(std::move(args));
        task = std::async(std::launch::async, [c] { return command(c); });
    }
    void open(const fs::path &p) {
        taskWorkspace = temporary();
        taskPath = p;
        start("Opening", {"import", p.string(), "--workspace", taskWorkspace.string(),
                          "--mesh-helper", helper.string()});
    }
    void save(const fs::path &p) {
        writeJson(workspace / "project.json", doc);
        taskPath = p;
        start("Saving", {"save", "--workspace", workspace.string(), "--output", p.string()});
    }
    void poll() {
        if (!busy || task.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return;
        Result r = task.get();
        busy = false;
        if (r.code) {
            lastError = r.text;
            status = "Operation failed";
            if (taskKind == "Opening" && taskWorkspace != workspace) {
                std::error_code ec;
                fs::remove_all(taskWorkspace, ec);
                taskWorkspace.clear();
            }
            return;
        }
        try {
            if (taskKind == "Opening") {
                Json fresh = readJson(taskWorkspace / "project.json");
                auto old = workspace;
                Json previous = doc;
                workspace = taskWorkspace;
                doc = std::move(fresh);
                try {
                    reload();
                    frameModel();
                } catch (...) {
                    doc = std::move(previous);
                    workspace = old;
                    reload();
                    throw;
                }
                selected = -1;
                undo.clear();
                redo.clear();
                dirty = false;
                savedFile = taskPath.extension() == ".xovmodel" ? taskPath : fs::path{};
                std::error_code ec;
                fs::remove_all(old, ec);
            } else if (taskKind == "Saving") {
                savedFile = taskPath;
                dirty = false;
            } else if (taskKind == "Exporting") {
                report = r.text;
                modal = "Export result";
            }
            status = taskKind + " complete";
        } catch (const std::exception &e) {
            lastError = e.what();
            status = "Operation failed";
            if (taskWorkspace != workspace && !taskWorkspace.empty()) {
                std::error_code ec;
                fs::remove_all(taskWorkspace, ec);
                taskWorkspace.clear();
            }
        }
    }
    std::string unique(const std::string &base, const std::string &list) {
        std::string name = base;
        int n = 2;
        auto used = [&] {
            for (const auto &s : doc[list])
                if (s.value("name", "") == name || s.value("id", "") == name)
                    return true;
            return false;
        };
        while (used())
            name = base + "_" + std::to_string(n++);
        return name;
    }
    void addSensor(const Json &type) {
        remember();
        std::string id = type.at("id");
        std::string name = unique(type.value("default_name", id), "sensors");
        doc["sensors"].push_back(
            {{"id", name},
             {"name", name},
             {"type", id},
             {"parent", doc["links"][0]["name"]},
             {"pose", Json::array({0, 0, .25, 0, 0, 0})},
             {"enabled", true},
             {"rate_hz", type.value("default_rate_hz", 30.0)},
             {"topic", ""},
             {"parameters", type.value("default_parameters", Json::object())}});
        selected = (int)doc["sensors"].size() - 1;
        selection = "sensor";
    }
    void addPlugin() {
        remember();
        std::string name = unique("custom_plugin", "plugins");
        doc["plugins"].push_back(
            {{"id", name},
             {"name", name},
             {"target", "gazebo"},
             {"filename", ""},
             {"xml", "<plugin name=\"" + name + "\" filename=\"plugin-library\">\n</plugin>"}});
        selected = (int)doc["plugins"].size() - 1;
        selection = "plugin";
    }
    Json *selectedItem() {
        std::string list = selection == "sensor"   ? "sensors"
                           : selection == "visual" ? "visuals"
                           : selection == "link"   ? "links"
                                                   : "plugins";
        if (selected < 0 || selected >= (int)doc[list].size())
            return nullptr;
        return &doc[list][selected];
    }
    void eraseSelected() {
        if (selection != "sensor" && selection != "plugin")
            return;
        if (!selectedItem())
            return;
        remember();
        auto &items = doc[selection == "sensor" ? "sensors" : "plugins"];
        items.erase(items.begin() + selected);
        selected = -1;
    }
    void dialog(const std::string &name, const std::string &initial = "") {
        modal = name;
        snprintf(pathBuffer, sizeof(pathBuffer), "%s", initial.c_str());
    }
    void browse(bool saveFile = false, bool directory = false) {
        auto args =
            std::vector<std::string>{"zenity", "--file-selection", "--title=Xovium Model Editor"};
        if (saveFile) {
            args.push_back("--save");
            args.push_back("--confirm-overwrite");
        }
        if (directory)
            args.push_back("--directory");
        if (pathBuffer[0])
            args.push_back("--filename=" + std::string(pathBuffer));
        Result r = command(args);
        if (!r.code) {
            while (!r.text.empty() && (r.text.back() == '\n' || r.text.back() == '\r'))
                r.text.pop_back();
            snprintf(pathBuffer, sizeof(pathBuffer), "%s", r.text.c_str());
        } else if (r.code == 127)
            lastError = "Zenity is unavailable; enter the path directly.";
    }
    void drawWorld(bool markers) {
        float grid = std::max(1.f, powf(10.f, floorf(log10f(std::max(distance, 1.f))) - 1));
        for (int n = -15; n <= 15; n++) {
            Color c = n == 0 ? Color{49, 64, 74, 255} : Color{30, 41, 50, 255};
            DrawLine3D({n * grid, -15 * grid, 0}, {n * grid, 15 * grid, 0}, c);
            DrawLine3D({-15 * grid, n * grid, 0}, {15 * grid, n * grid, 0}, c);
        }
        for (auto &m : meshes) {
            m.model.transform = visualMatrix(m.visual);
            DrawModel(m.model, {0, 0, 0}, 1, WHITE);
            if (wire || (selection == "visual" && selected == m.visual))
                DrawModelWires(m.model, {0, 0, 0}, 1, {70, 180, 170, 100});
        }
        if (!markers)
            return;
        DrawLine3D({0, 0, .005f}, {1, 0, .005f}, RED);
        DrawLine3D({0, 0, .005f}, {0, 1, .005f}, GREEN);
        DrawLine3D({0, 0, 0}, {0, 0, 1}, BLUE);
        for (size_t i = 0; i < doc["sensors"].size(); i++) {
            const auto &s = doc["sensors"][i];
            Matrix m = sensorMatrix(s);
            bool sel = selection == "sensor" && selected == (int)i;
            Color color = !s.value("enabled", true) ? Color{95, 101, 110, 255}
                          : sel                     ? Color{255, 193, 92, 255}
                                                    : Color{62, 210, 185, 255};
            float radius = std::clamp(distance * .009f, .025f, .2f);
            DrawSphere(location(m), radius, color);
            DrawLine3D(location(m), point(m, {radius * 4, 0, 0}), color);
            std::string type = s.value("type", "");
            if (type == "camera" || type == "depth_camera") {
                auto params = s["parameters"];
                float fov = params.value("horizontal_fov", 1.0472f),
                      aspect = params.value("width", 1280.f) /
                               std::max(1.f, params.value("height", 720.f));
                float length = radius * 8, half = length * tanf(fov / 2);
                Vector3 corners[4] = {{length, half, half / aspect},
                                      {length, -half, half / aspect},
                                      {length, -half, -half / aspect},
                                      {length, half, -half / aspect}};
                for (int n = 0; n < 4; n++) {
                    DrawLine3D(location(m), point(m, corners[n]), color);
                    DrawLine3D(point(m, corners[n]), point(m, corners[(n + 1) % 4]), color);
                }
            }
        }
    }
    void drawViewport() {
        ImVec2 size = ui::GetContentRegionAvail();
        int w = std::max(1, (int)size.x), h = std::max(1, (int)size.y);
        ImVec2 top = ui::GetCursorScreenPos();
        if (w != viewportW || h != viewportH) {
            if (viewport.id)
                UnloadRenderTexture(viewport);
            viewport = LoadRenderTexture(w, h);
            viewportW = w;
            viewportH = h;
        }
        camera.position = Vector3Add(target, {distance * cosf(elevation) * cosf(azimuth),
                                              distance * cosf(elevation) * sinf(azimuth),
                                              distance * sinf(elevation)});
        camera.target = target;
        camera.up = {0, 0, 1};
        camera.fovy = 45;
        camera.projection = CAMERA_PERSPECTIVE;
        BeginTextureMode(viewport);
        ClearBackground({19, 27, 35, 255});
        BeginMode3D(camera);
        drawWorld(true);
        EndMode3D();
        EndTextureMode();
        rlImGuiImageRenderTexture(&viewport);
        bool hover = ui::IsItemHovered();
        if (hover && !busy) {
            Vector2 delta = GetMouseDelta();
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                azimuth -= delta.x * .008f;
                elevation = std::clamp(elevation + delta.y * .008f, -1.48f, 1.48f);
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
                Vector3 forward = Vector3Normalize(Vector3Subtract(target, camera.position)),
                        right = Vector3Normalize(Vector3CrossProduct(forward, {0, 0, 1})),
                        up = Vector3CrossProduct(right, forward);
                target = Vector3Add(target, Vector3Scale(Vector3Add(Vector3Scale(right, -delta.x),
                                                                    Vector3Scale(up, delta.y)),
                                                         distance * .0015f));
            }
            distance = std::clamp(distance * expf(-GetMouseWheelMove() * .12f), .1f, 100000.f);
            if (IsKeyPressed(KEY_F))
                frameModel();
        }
        Json *item = selectedItem();
        if (item && !busy && selection != "plugin") {
            Matrix parent = selection == "sensor"   ? linkMatrix(item->value("parent", "base_link"))
                            : selection == "visual" ? linkMatrix(item->value("link", "base_link"))
                                                    : MatrixIdentity();
            Matrix world = product(parent, pose((*item)["pose"]));
            auto view = elements(GetCameraMatrix(camera)),
                 projection =
                     elements(MatrixPerspective(camera.fovy * DEG2RAD, (double)w / h, .01, 100000));
            auto transform = elements(world);
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist(ui::GetWindowDrawList());
            ImGuizmo::SetRect(top.x, top.y, (float)w, (float)h);
            float increment = operation == 0 ? .1f : 15.f;
            float snapping[] = {increment, increment, increment};
            ImGuizmo::Manipulate(view.data(), projection.data(),
                                 operation == 0 ? ImGuizmo::TRANSLATE : ImGuizmo::ROTATE,
                                 local ? ImGuizmo::LOCAL : ImGuizmo::WORLD, transform.data(),
                                 nullptr, snap ? snapping : nullptr);
            bool usingNow = ImGuizmo::IsUsing();
            if (usingNow) {
                if (!gizmoUsing)
                    remember();
                (*item)["pose"] = unpose(product(MatrixInvert(parent), matrix(transform)));
                dirty = true;
            }
            gizmoUsing = usingNow;
        }
        if (hover && !busy && !ImGuizmo::IsOver() && !gizmoUsing &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mouse = GetMousePosition();
            Ray ray = GetScreenToWorldRayEx({mouse.x - top.x, mouse.y - top.y}, camera, w, h);
            float nearest = std::numeric_limits<float>::max();
            int hit = -1;
            std::string kind;
            for (size_t i = 0; i < doc["sensors"].size(); i++) {
                auto collision =
                    GetRayCollisionSphere(ray, location(sensorMatrix(doc["sensors"][i])),
                                          std::clamp(distance * .014f, .04f, .3f));
                if (collision.hit && collision.distance < nearest) {
                    nearest = collision.distance;
                    hit = (int)i;
                    kind = "sensor";
                }
            }
            if (hit < 0)
                for (const auto &m : meshes)
                    for (int n = 0; n < m.model.meshCount; n++) {
                        auto collision =
                            GetRayCollisionMesh(ray, m.model.meshes[n], visualMatrix(m.visual));
                        if (collision.hit && collision.distance < nearest) {
                            nearest = collision.distance;
                            hit = m.visual;
                            kind = "visual";
                        }
                    }
            if (hit >= 0) {
                selected = hit;
                selection = kind;
            }
        }
        ui::GetWindowDrawList()->AddText({top.x + 16, top.y + 14}, IM_COL32(140, 161, 174, 255),
                                         "RMB orbit   MMB pan   Wheel zoom   F frame model");
        std::string footer = "X forward / Y left / Z up   |   metres";
        ui::GetWindowDrawList()->AddText({top.x + 16, top.y + h - 25}, IM_COL32(105, 130, 145, 255),
                                         footer.c_str());
    }
    void drawPreview(const Json &s) {
        Matrix m = sensorMatrix(s);
        Camera3D cam{};
        cam.position = location(m);
        cam.target = point(m, {1, 0, 0});
        cam.up = Vector3Normalize(Vector3Subtract(point(m, {0, 0, 1}), cam.position));
        auto p = s["parameters"];
        float aspect = p.value("width", 1280.f) / std::max(1.f, p.value("height", 720.f));
        int previewHeight = std::clamp((int)std::round(640 / aspect), 1, 2048);
        if (!preview.id || preview.texture.height != previewHeight) {
            if (preview.id)
                UnloadRenderTexture(preview);
            preview = LoadRenderTexture(640, previewHeight);
        }
        cam.fovy = 2 * atanf(tanf(p.value("horizontal_fov", 1.0472f) * .5f) / aspect) * RAD2DEG;
        cam.projection = CAMERA_PERSPECTIVE;
        BeginTextureMode(preview);
        ClearBackground({28, 38, 48, 255});
        rlSetClipPlanes(std::max(.001f, p.value("near", .05f)), p.value("far", 1000.f));
        BeginMode3D(cam);
        drawWorld(false);
        EndMode3D();
        EndTextureMode();
        rlSetClipPlanes(.01, 100000);
        int width = std::max(1, (int)ui::GetContentRegionAvail().x);
        rlImGuiImageRect(&preview.texture, width, width * previewHeight / 640,
                         {0, 0, (float)preview.texture.width, -(float)preview.texture.height});
        ui::TextWrapped("Mounting preview; not simulated sensor data");
    }
    void transformInspector(Json &item) {
        if (!item.contains("pose"))
            return;
        float position[3], rotation[3];
        for (int i = 0; i < 3; i++) {
            position[i] = item["pose"][i];
            rotation[i] = item["pose"][i + 3].get<float>() * RAD2DEG;
        }
        ui::TextUnformatted("Position (m)");
        if (ui::DragFloat3("##position", position, .01f))
            for (int i = 0; i < 3; i++)
                item["pose"][i] = position[i];
        ui::TextUnformatted("Rotation (degrees)");
        if (ui::DragFloat3("##rotation", rotation, .5f))
            for (int i = 0; i < 3; i++)
                item["pose"][i + 3] = rotation[i] * DEG2RAD;
    }
    void inspector() {
        Json *item = selectedItem();
        if (!item) {
            ui::TextColored(accent, "VEHICLE");
            textField("Name", doc["name"]);
            const char *names[] = {"fixed-wing", "rover", "custom"};
            int current = doc.value("vehicle", "") == "rover"    ? 1
                          : doc.value("vehicle", "") == "custom" ? 2
                                                                 : 0;
            ui::TextUnformatted("Vehicle family");
            if (ui::Combo("##family", &current, names, 3))
                doc["vehicle"] = names[current];
            ui::Spacing();
            ui::TextWrapped("Select a sensor, visual or link to edit its mounting transform.");
            return;
        }
        ui::TextColored(accent, "%s", selection.c_str());
        if (selection == "link")
            ui::TextWrapped("%s", item->value("name", "").c_str());
        else
            textField("Name", (*item)["name"]);
        if (selection == "sensor" || selection == "visual") {
            const char *key = selection == "sensor" ? "parent" : "link";
            std::string current = item->value(key, "base_link");
            ui::TextUnformatted("Attach to");
            if (ui::BeginCombo("##attach", current.c_str())) {
                for (const auto &link : doc["links"]) {
                    auto name = link["name"].get<std::string>();
                    if (ui::Selectable(name.c_str(), current == name))
                        (*item)[key] = name;
                }
                ui::EndCombo();
            }
        }
        transformInspector(*item);
        ui::Separator();
        if (selection == "sensor") {
            ui::TextDisabled("%s", item->value("type", "").c_str());
            bool enabled = item->value("enabled", true);
            if (ui::Checkbox("Enabled", &enabled))
                (*item)["enabled"] = enabled;
            float hz = item->value("rate_hz", 30.f);
            ui::TextUnformatted("Rate (Hz)");
            if (ui::DragFloat("##rate", &hz, 1, 0, 2000))
                (*item)["rate_hz"] = hz;
            textField("Topic", (*item)["topic"]);
            const Json *type = nullptr;
            for (const auto &t : catalog["types"])
                if (t["id"] == (*item)["type"])
                    type = &t;
            if (type) {
                ui::TextWrapped("%s", type->value("description", "").c_str());
                if (type->contains("parameters"))
                    for (const auto &p : (*type)["parameters"]) {
                        std::string key = p.at("key"), label = p.value("label", key);
                        ui::TextWrapped("%s", label.c_str());
                        label = "##param_" + key;
                        auto &value = (*item)["parameters"][key];
                        std::string kind = p.value("type", "float");
                        if (kind == "string")
                            textField(label.c_str(), value);
                        else if (kind == "bool") {
                            bool v = value.is_boolean() ? value.get<bool>() : false;
                            if (ui::Checkbox(label.c_str(), &v))
                                value = v;
                        } else if (kind == "int") {
                            int v = value.is_number() ? value.get<int>() : 0;
                            if (ui::DragInt(label.c_str(), &v, p.value("step", 1.f),
                                            p.value("min", 0), p.value("max", 100000)))
                                value = v;
                        } else {
                            float v = value.is_number() ? value.get<float>() : 0;
                            if (ui::DragFloat(label.c_str(), &v, p.value("step", .01f),
                                              p.value("min", 0.f), p.value("max", 100000.f)))
                                value = v;
                        }
                    }
            }
            if ((item->value("type", "") == "camera" ||
                 item->value("type", "") == "depth_camera") &&
                ui::CollapsingHeader("Camera preview", ImGuiTreeNodeFlags_DefaultOpen))
                drawPreview(*item);
            ui::Spacing();
            if (ui::Button("Duplicate")) {
                Json duplicate = *item;
                remember();
                duplicate["name"] = unique(item->value("name", "sensor"), "sensors");
                duplicate["id"] = duplicate["name"];
                doc["sensors"].push_back(duplicate);
                selected = (int)doc["sensors"].size() - 1;
            }
            ui::SameLine();
            if (ui::Button("Remove"))
                eraseSelected();
        } else if (selection == "link") {
            float mass = item->value("mass", 1.f);
            ui::TextUnformatted("Mass (kg)");
            if (ui::DragFloat("##mass", &mass, .1f, .001f, 1000000))
                (*item)["mass"] = mass;
            ui::TextWrapped("Link inertial and joint data are preserved in the project. Geometry "
                            "alone does not define flight dynamics.");
        } else if (selection == "visual") {
            float scale[3];
            for (int i = 0; i < 3; i++)
                scale[i] = (*item)["scale"][i];
            ui::TextUnformatted("Scale");
            if (ui::DragFloat3("##scale", scale, .01f, .0001f, 10000))
                for (int i = 0; i < 3; i++)
                    (*item)["scale"][i] = scale[i];
        } else {
            const char *targets[] = {"gazebo", "isaac", "unreal", "flightgear", "sih"};
            int t = 0;
            for (int i = 0; i < 5; i++)
                if (item->value("target", "") == targets[i])
                    t = i;
            ui::TextUnformatted("Target");
            if (ui::Combo("##target", &t, targets, 5))
                (*item)["target"] = targets[t];
            textField("Library", (*item)["filename"]);
            textField("Plugin XML", (*item)["xml"], 16384, true);
            ui::TextWrapped("Plugin code must exist in the target simulator. The editor preserves "
                            "configuration; it does not execute arbitrary plugins.");
            if (ui::Button("Remove plugin"))
                eraseSelected();
        }
    }
    void dialogs() {
        if (modal.empty())
            return;
        ui::OpenPopup(modal.c_str());
        ui::SetNextWindowSize({650, 0}, ImGuiCond_Appearing);
        if (ui::BeginPopupModal(modal.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            if (modal == "Discard changes?") {
                ui::TextWrapped("This project has unsaved changes.");
                if (ui::Button("Keep editing")) {
                    closeWanted = false;
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
                ui::SameLine();
                if (ui::Button("Discard and close")) {
                    dirty = false;
                    closeWanted = true;
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
            } else if (modal == "New model") {
                ui::TextWrapped("Create an empty vehicle project. Unsaved edits to the current "
                                "model will be discarded.");
                ui::InputText("Name", pathBuffer, sizeof(pathBuffer));
                static int family = 0;
                const char *families[] = {"fixed-wing", "rover", "custom"};
                ui::Combo("Vehicle family", &family, families, 3);
                if (ui::Button("Create")) {
                    taskWorkspace = temporary();
                    taskPath.clear();
                    start("Opening", {"new", "--workspace", taskWorkspace.string(), "--name",
                                      pathBuffer, "--vehicle", families[family]});
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
                ui::SameLine();
                if (ui::Button("Cancel")) {
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
            } else if (modal == "Export result") {
                ui::TextWrapped("%s", report.c_str());
                if (ui::Button("Close")) {
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
            } else {
                bool opening = modal == "Open model", exporting = modal == "Export model";
                ui::TextWrapped(
                    "%s", opening ? "Open .xovmodel, SDF or a 3D asset (OBJ, DAE, glTF/GLB, STL, "
                                    "FBX). Import keeps source files separate."
                          : exporting ? "Export to a new or empty directory. The report lists "
                                        "native components and sensors that need a runtime adapter."
                                      : "Save a portable .xovmodel bundle with model geometry, "
                                        "sensor mounting and plugin configuration.");
                ui::SetNextItemWidth(520);
                ui::InputText("##path", pathBuffer, sizeof(pathBuffer));
                ui::SameLine();
                if (ui::Button("Browse"))
                    browse(!opening && !exporting, exporting);
                static const char *targets[] = {"gazebo", "isaac", "sih", "unreal", "flightgear"};
                if (exporting) {
                    ui::Combo("Simulator", &exportTarget, targets, 5);
                    ui::TextWrapped("Gazebo: native SDF sensors; PX4 routing and world systems "
                                    "must match. Isaac/SIH: USD geometry and cameras; other sensor "
                                    "definitions are retained. Unreal/FlightGear: native geometry "
                                    "plus import/configuration files. Exported metadata requires a "
                                    "runtime adapter to produce measurements.");
                }
                if (opening && dirty)
                    ui::TextColored({1, .7f, .3f, 1},
                                    "Opening another model discards unsaved changes.");
                bool ready = pathBuffer[0];
                ui::BeginDisabled(!ready || busy);
                if (ui::Button(opening ? "Open" : exporting ? "Export" : "Save", {130, 32})) {
                    fs::path p = pathBuffer;
                    try {
                        if (opening)
                            open(p);
                        else if (exporting) {
                            writeJson(workspace / "project.json", doc);
                            taskPath = p;
                            start("Exporting",
                                  {"export", "--workspace", workspace.string(), "--target",
                                   targets[exportTarget], "--output", p.string()});
                        } else {
                            if (p.extension() != ".xovmodel")
                                p += ".xovmodel";
                            save(p);
                        }
                        modal.clear();
                        ui::CloseCurrentPopup();
                    } catch (const std::exception &e) {
                        lastError = e.what();
                    }
                }
                ui::EndDisabled();
                ui::SameLine();
                if (ui::Button("Cancel", {100, 32})) {
                    modal.clear();
                    ui::CloseCurrentPopup();
                }
            }
            ui::EndPopup();
        }
    }
    void draw() {
        poll();
        ImGuizmo::BeginFrame();
        int width = GetScreenWidth(), height = GetScreenHeight();
        ui::SetNextWindowPos({0, 0});
        ui::SetNextWindowSize({(float)width, (float)height});
        ui::Begin("Xovium Model Editor", nullptr,
                  ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                      ImGuiWindowFlags_NoSavedSettings);
        ui::TextColored(accent, "X O V I U M  /  MODEL EDITOR");
        ui::SameLine();
        ui::TextDisabled("  %s%s", doc.value("name", "Vehicle").c_str(), dirty ? " *" : "");
        ui::Spacing();
        ui::BeginDisabled(busy);
        if (ui::Button("New"))
            dialog("New model", "Untitled vehicle");
        ui::SameLine();
        if (ui::Button("Open model"))
            dialog("Open model");
        ui::SameLine();
        if (ui::Button("Save .xovmodel")) {
            if (savedFile.empty())
                dialog("Save model", doc.value("name", "vehicle") + ".xovmodel");
            else
                save(savedFile);
        }
        ui::SameLine();
        if (ui::Button("Save as"))
            dialog("Save model", doc.value("name", "vehicle") + ".xovmodel");
        ui::SameLine();
        if (ui::Button("Export"))
            dialog("Export model");
        ui::SameLine();
        if (ui::Button("Undo"))
            history(true);
        ui::SameLine();
        if (ui::Button("Redo"))
            history(false);
        ui::SameLine();
        if (ui::Button("Frame model"))
            frameModel();
        ui::EndDisabled();
        ui::Separator();
        float available = height - 125.f;
        ui::BeginChild("scene", {220, available}, ImGuiChildFlags_Borders);
        ui::TextColored(accent, "SCENE");
        ui::BeginDisabled(busy);
        if (ui::Button("+ Sensor", {-1, 30}))
            ui::OpenPopup("Add sensor");
        if (ui::BeginPopup("Add sensor")) {
            for (const auto &type : catalog["types"])
                if (ui::Selectable(type.value("label", type.value("id", "")).c_str()))
                    addSensor(type);
            ui::EndPopup();
        }
        if (ui::Button("+ Simulator plugin", {-1, 26}))
            addPlugin();
        ui::Spacing();
        auto list = [&](const char *title, const char *key, const char *kind) {
            if (ui::CollapsingHeader(title, ImGuiTreeNodeFlags_DefaultOpen)) {
                for (size_t i = 0; i < doc[key].size(); i++) {
                    ui::PushID(key);
                    ui::PushID((int)i);
                    auto name = doc[key][i].value("name", kind);
                    if (ui::Selectable(name.c_str(), selection == kind && selected == (int)i)) {
                        selected = (int)i;
                        selection = kind;
                    }
                    ui::PopID();
                    ui::PopID();
                }
            }
        };
        list("Links", "links", "link");
        list("Visuals", "visuals", "visual");
        list("Sensors", "sensors", "sensor");
        list("Plugins", "plugins", "plugin");
        ui::EndDisabled();
        ui::EndChild();
        ui::SameLine();
        ui::BeginChild("canvas", {std::max(200.f, width - 580.f), available},
                       ImGuiChildFlags_Borders);
        ui::RadioButton("Move", &operation, 0);
        ui::SameLine();
        ui::RadioButton("Rotate", &operation, 1);
        ui::SameLine();
        ui::Checkbox("Local", &local);
        ui::SameLine();
        ui::Checkbox("Snap", &snap);
        ui::SameLine();
        ui::Checkbox("Wireframe", &wire);
        drawViewport();
        ui::EndChild();
        ui::SameLine();
        ui::BeginChild("inspector", {0, available}, ImGuiChildFlags_Borders);
        ui::BeginDisabled(busy);
        ui::PushItemWidth(-1);
        unsigned long beforeSerial = undoSerial;
        Json before = doc;
        inspector();
        ui::PopItemWidth();
        if (doc != before) {
            if (!editActive && beforeSerial == undoSerial) {
                ++undoSerial;
                undo.push_back(before);
                if (undo.size() > 100)
                    undo.erase(undo.begin());
                redo.clear();
            }
            dirty = true;
            editActive = true;
        }
        if (!ui::IsAnyItemActive())
            editActive = false;
        ui::EndDisabled();
        if (doc.contains("warnings") && !doc["warnings"].empty()) {
            ui::Separator();
            ui::TextColored({1, .7f, .3f, 1}, "Import notes");
            for (const auto &w : doc["warnings"])
                ui::TextWrapped("%s", w.get<std::string>().c_str());
        }
        ui::EndChild();
        ui::TextColored(busy ? accent : muted, "%s", status.c_str());
        if (!lastError.empty()) {
            ui::SameLine();
            if (ui::SmallButton("Details"))
                ui::OpenPopup("Operation error");
            ui::SetNextWindowSize({700, 0}, ImGuiCond_Appearing);
            if (ui::BeginPopupModal("Operation error", nullptr,
                                    ImGuiWindowFlags_AlwaysAutoResize)) {
                ui::TextWrapped("%s", lastError.c_str());
                if (ui::Button("Close")) {
                    lastError.clear();
                    ui::CloseCurrentPopup();
                }
                ui::EndPopup();
            }
        }
        dialogs();
        ui::End();
        if (!ui::GetIO().WantTextInput && !busy &&
            !ui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)) {
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            if (ctrl && IsKeyPressed(KEY_Z))
                history(true);
            if (ctrl && IsKeyPressed(KEY_Y))
                history(false);
            if (ctrl && IsKeyPressed(KEY_S)) {
                if (savedFile.empty())
                    dialog("Save model", doc.value("name", "vehicle") + ".xovmodel");
                else
                    save(savedFile);
            }
            if (IsKeyPressed(KEY_DELETE))
                eraseSelected();
        }
    }
};
void theme() {
    auto &s = ui::GetStyle();
    s.WindowRounding = 0;
    s.ChildRounding = 6;
    s.FrameRounding = 4;
    s.GrabRounding = 4;
    s.WindowPadding = {12, 10};
    s.FramePadding = {9, 6};
    s.ItemSpacing = {8, 8};
    s.Colors[ImGuiCol_WindowBg] = {.055f, .075f, .095f, 1};
    s.Colors[ImGuiCol_ChildBg] = {.07f, .095f, .12f, 1};
    s.Colors[ImGuiCol_FrameBg] = {.12f, .16f, .19f, 1};
    s.Colors[ImGuiCol_Button] = {.13f, .21f, .24f, 1};
    s.Colors[ImGuiCol_ButtonHovered] = {.18f, .32f, .34f, 1};
    s.Colors[ImGuiCol_Header] = {.13f, .28f, .29f, 1};
    s.Colors[ImGuiCol_HeaderHovered] = {.16f, .34f, .35f, 1};
    s.Colors[ImGuiCol_CheckMark] = accent;
    s.Colors[ImGuiCol_SliderGrab] = accent;
    s.Colors[ImGuiCol_Text] = {.88f, .92f, .94f, 1};
    s.Colors[ImGuiCol_TextDisabled] = muted;
    s.Colors[ImGuiCol_Border] = {.14f, .20f, .24f, 1};
}
int main(int argc, char **argv) {
    fs::path root = XOVIUM_SOURCE_DIR, open;
    std::string screenshot, stateReport;
    int frames = 0;
    bool smoke = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--root" && i + 1 < argc)
            root = argv[++i];
        else if (a == "--screenshot" && i + 1 < argc)
            screenshot = argv[++i];
        else if (a == "--state-report" && i + 1 < argc)
            stateReport = argv[++i];
        else if (a == "--frames" && i + 1 < argc)
            frames = std::stoi(argv[++i]);
        else if (a == "--smoke")
            smoke = true;
        else if (a == "--help") {
            std::cout << "Xovium Model Editor\nUsage: xovium-model-editor [--root PATH] "
                         "[MODEL.xovmodel|MODEL.sdf|MESH]\n";
            return 0;
        } else
            open = a;
    }
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1440, 900, "Xovium Model Editor");
    SetWindowMinSize(1100, 700);
    rlSetClipPlanes(.01, 100000);
    SetExitKey(KEY_NULL);
    SetTargetFPS(60);
    rlImGuiBeginInitImGui();
    ui::GetIO().IniFilename = nullptr;
    if (fs::is_regular_file("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
        ui::GetIO().FontDefault = ui::GetIO().Fonts->AddFontFromFileTTF(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16);
    rlImGuiEndInitImGui();
    theme();
    int result = 0;
    try {
        {
            Editor editor(root);
            editor.init();
            editor.screenshot = screenshot;
            editor.maxFrames = frames;
            if (!open.empty())
                editor.open(open);
            if (smoke) {
                for (const auto &t : editor.catalog["types"])
                    if (t["id"] == "camera") {
                        editor.addSensor(t);
                        auto &s = editor.doc["sensors"].back();
                        s["pose"] = Json::array({.8, 0, .35, 0, .12, 0});
                    }
                editor.frameModel();
            }
            while (true) {
                if (WindowShouldClose()) {
                    if (editor.busy)
                        editor.status = "Wait for the current operation before closing";
                    else if (editor.dirty)
                        editor.modal = "Discard changes?";
                    else
                        break;
                }
                if (editor.closeWanted && !editor.dirty)
                    break;
                BeginDrawing();
                ClearBackground({14, 19, 24, 255});
                rlImGuiBegin();
                editor.draw();
                rlImGuiEnd();
                EndDrawing();
                editor.frames++;
                if (frames > 0 && editor.frames >= frames && !editor.busy) {
                    if (!screenshot.empty()) {
                        Image frame = LoadImageFromScreen();
                        ExportImage(frame, screenshot.c_str());
                        UnloadImage(frame);
                    }
                    break;
                }
            }
            if (!stateReport.empty())
                writeJson(stateReport, editor.doc);
            if (smoke && editor.doc["sensors"].empty())
                result = 1;
        }
    } catch (const std::exception &e) {
        std::cerr << "Editor: " << e.what() << '\n';
        result = 1;
    }
    rlImGuiShutdown();
    CloseWindow();
    return result;
}
