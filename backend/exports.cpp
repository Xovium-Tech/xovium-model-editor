#include "project.hpp"
#include <tinyxml2.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace xovium {
namespace {
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using Matrix = std::array<std::array<double, 4>, 4>;
constexpr double pi = 3.14159265358979323846;
const Json zeroPose = {0, 0, 0, 0, 0, 0};
const Json unitScale = {1, 1, 1};
const std::map<std::string, std::string> types{{"airspeed", "air_speed"},
                                               {"gps", "navsat"},
                                               {"barometer", "air_pressure"},
                                               {"imu", "imu"},
                                               {"magnetometer", "magnetometer"},
                                               {"camera", "camera"},
                                               {"depth_camera", "depth_camera"},
                                               {"lidar", "gpu_lidar"},
                                               {"contact", "contact"}};
const std::map<std::string, std::string> systems{{"airspeed", "AirSpeed"},
                                                 {"gps", "NavSat"},
                                                 {"barometer", "AirPressure"},
                                                 {"imu", "Imu"},
                                                 {"magnetometer", "Magnetometer"},
                                                 {"camera", "Sensors"},
                                                 {"depth_camera", "Sensors"},
                                                 {"lidar", "Sensors"},
                                                 {"contact", "Contact"}};
std::string str(const Json &value) {
    return value.is_string() ? value.get<std::string>() : value.dump();
}
std::string number(double value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(12) << value;
    return out.str();
}
std::string numbers(const Json &values, const std::string &separator = " ") {
    std::string result;
    for (const auto &value : values) {
        if (!result.empty())
            result += separator;
        result += number(value.get<double>());
    }
    return result;
}
std::string array(const Json &values) {
    return "(" + numbers(values, ", ") + ")";
}
std::string quote(const std::string &value) {
    return Json(value).dump();
}
std::string usdName(std::string value) {
    for (char &c : value)
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
              c == '_'))
            c = '_';
    if (value.empty())
        value = "model";
    if (value[0] >= '0' && value[0] <= '9')
        value = "_" + value;
    return value;
}
void write(const fs::path &path, const std::string &data) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out || !(out << data))
        throw std::runtime_error("Cannot write export file: " + path.string());
    out.close();
    if (!out)
        throw std::runtime_error("Cannot finish export file: " + path.string());
}
std::string read(const fs::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("Cannot read asset: " + path.string());
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}
bool within(const fs::path &path, const fs::path &root) {
    auto p = path.begin();
    for (auto r = root.begin(); r != root.end(); ++r, ++p)
        if (p == path.end() || *p != *r)
            return false;
    return true;
}
fs::path asset(const fs::path &workspace, const std::string &relative) {
    auto path = fs::weakly_canonical(workspace / relative);
    if (!within(path, workspace) || !fs::is_regular_file(path))
        throw std::runtime_error("Missing or external workspace asset: " + relative);
    return path;
}
Matrix matrix(const Json &pose = zeroPose) {
    double x = pose[0], y = pose[1], z = pose[2], r = pose[3], p = pose[4], h = pose[5];
    double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p), ch = std::cos(h),
           sh = std::sin(h);
    return {{{ch * cp, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr, x},
             {sh * cp, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr, y},
             {-sp, cp * sr, cp * cr, z},
             {0, 0, 0, 1}}};
}
Matrix multiply(const Matrix &a, const Matrix &b) {
    Matrix out{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                out[i][j] += a[i][k] * b[k][j];
    return out;
}
Json point(const Matrix &m, const Json &p) {
    Json result = Json::array();
    for (int i = 0; i < 3; ++i) {
        double v = m[i][3];
        for (int j = 0; j < 3; ++j)
            v += m[i][j] * p[j].get<double>();
        result.push_back(v);
    }
    return result;
}
Json quaternion(const Matrix &m) {
    double trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0) {
        double s = std::sqrt(trace + 1) * 2;
        return {s / 4, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
    }
    int i = 0;
    for (int axis = 1; axis < 3; ++axis)
        if (m[axis][axis] > m[i][i])
            i = axis;
    int j = (i + 1) % 3, k = (i + 2) % 3;
    double s = std::sqrt(std::max(0.0, 1 + m[i][i] - m[j][j] - m[k][k])) * 2;
    Json q = {0., 0., 0., 0.};
    q[0] = (m[k][j] - m[j][k]) / s;
    q[i + 1] = s / 4;
    q[j + 1] = (m[j][i] + m[i][j]) / s;
    q[k + 1] = (m[k][i] + m[i][k]) / s;
    return q;
}
Json meshes(const Json &visual, const fs::path &workspace) {
    const auto &geometry = visual.at("geometry");
    std::string kind = geometry.at("type");
    Json result, vertices = Json::array(), faces = Json::array();
    if (kind == "mesh") {
        result = Json::parse(read(asset(workspace, geometry.at("path"))));
        if (result.is_object())
            result = result.contains("meshes") ? result["meshes"] : result["parts"];
        if (!result.is_array())
            throw std::runtime_error("Mesh asset must contain an array of mesh parts");
    } else if (kind == "box") {
        double sx = geometry["size"][0].get<double>() / 2,
               sy = geometry["size"][1].get<double>() / 2,
               sz = geometry["size"][2].get<double>() / 2;
        for (auto v : std::vector<std::array<int, 3>>{{-1, -1, -1},
                                                      {1, -1, -1},
                                                      {1, 1, -1},
                                                      {-1, 1, -1},
                                                      {-1, -1, 1},
                                                      {1, -1, 1},
                                                      {1, 1, 1},
                                                      {-1, 1, 1}})
            vertices.push_back({v[0] * sx, v[1] * sy, v[2] * sz});
        faces = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                 {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    } else if (kind == "cylinder") {
        double radius = geometry["radius"], length = geometry["length"];
        for (double z : {-length / 2, length / 2}) {
            for (int i = 0; i < 32; ++i)
                vertices.push_back(
                    {radius * std::cos(i * 2 * pi / 32), radius * std::sin(i * 2 * pi / 32), z});
        }
        vertices.push_back({0, 0, -length / 2});
        vertices.push_back({0, 0, length / 2});
        for (int i = 0; i < 32; ++i) {
            int j = (i + 1) % 32;
            for (auto face :
                 Json{{i, j, j + 32}, {i, j + 32, i + 32}, {64, j, i}, {65, i + 32, j + 32}})
                faces.push_back(face);
        }
    } else if (kind == "sphere") {
        double radius = geometry["radius"];
        for (int j = 0; j < 17; ++j) {
            double p = pi * j / 16;
            for (int i = 0; i < 32; ++i)
                vertices.push_back({radius * std::sin(p) * std::cos(i * 2 * pi / 32),
                                    radius * std::sin(p) * std::sin(i * 2 * pi / 32),
                                    radius * std::cos(p)});
        }
        for (int j = 0; j < 16; ++j)
            for (int i = 0; i < 32; ++i) {
                int a = j * 32 + i, b = j * 32 + (i + 1) % 32;
                faces.push_back({a, b, b + 32});
                faces.push_back({a, b + 32, a + 32});
            }
    } else
        throw std::runtime_error("Unsupported geometry type: " + kind);
    if (kind != "mesh")
        result = Json::array({Json{{"vertices", vertices}, {"faces", faces}}});
    for (auto &mesh : result) {
        if (!mesh.contains("uv"))
            mesh["uv"] = Json::array();
        if (!mesh.contains("color"))
            mesh["color"] = visual.value("color", Json{.65, .65, .65, 1});
        if (!mesh.contains("texture"))
            mesh["texture"] = "";
        if (visual.contains("color")) {
            if (kind == "mesh")
                for (int i = 0; i < 4; ++i)
                    mesh["color"][i] =
                        mesh["color"][i].get<double>() * visual["color"][i].get<double>();
            else
                mesh["color"] = visual["color"];
        }
        if (!mesh["texture"].get<std::string>().empty())
            asset(workspace, mesh["texture"]);
    }
    return result;
}
void obj(const Json &parts, const fs::path &path, const std::string &texturePrefix = "../") {
    fs::path mtlPath = path;
    mtlPath.replace_extension(".mtl");
    std::ostringstream out, mtl;
    out << "mtllib " << mtlPath.filename().string() << '\n';
    size_t offset = 1, uvOffset = 1, index = 0;
    for (const auto &mesh : parts) {
        std::string name = "part_" + std::to_string(index++);
        out << "o " << name << "\nusemtl " << name << '\n';
        for (const auto &p : mesh["vertices"])
            out << "v " << numbers(p) << '\n';
        for (const auto &p : mesh["uv"])
            out << "vt " << numbers(p) << '\n';
        for (const auto &face : mesh["faces"]) {
            out << 'f';
            for (const auto &n : face) {
                size_t j = n.get<size_t>();
                out << ' ' << offset + j;
                if (!mesh["uv"].empty())
                    out << '/' << uvOffset + j;
            }
            out << '\n';
        }
        offset += mesh["vertices"].size();
        uvOffset += mesh["uv"].size();
        mtl << "newmtl " << name << "\nKd "
            << numbers(Json{mesh["color"][0], mesh["color"][1], mesh["color"][2]}) << "\nd "
            << str(mesh["color"][3]) << '\n';
        std::string texture = mesh["texture"];
        if (!texture.empty())
            mtl << "map_Kd " << texturePrefix << texture << '\n';
    }
    write(path, out.str());
    write(mtlPath, mtl.str());
}
std::vector<std::string> split(const std::string &value, char separator) {
    std::vector<std::string> result;
    std::istringstream in(value);
    std::string part;
    while (std::getline(in, part, separator))
        result.push_back(part);
    return result;
}
std::string attribute(const XMLElement *node, const char *key, const std::string &fallback = "") {
    auto value = node ? node->Attribute(key) : nullptr;
    return value ? value : fallback;
}
XMLElement *find(XMLElement *parent, const std::string &path) {
    for (const auto &part : split(path, '/')) {
        if (!parent)
            return nullptr;
        parent = parent->FirstChildElement(part.c_str());
    }
    return parent;
}
XMLElement *ensure(XMLElement *parent, const std::string &path) {
    for (const auto &part : split(path, '/')) {
        auto child = parent->FirstChildElement(part.c_str());
        if (!child) {
            child = parent->GetDocument()->NewElement(part.c_str());
            parent->InsertEndChild(child);
        }
        parent = child;
    }
    return parent;
}
XMLElement *set(XMLElement *parent, const std::string &path, const Json &value) {
    auto node = ensure(parent, path);
    node->SetText(str(value).c_str());
    return node;
}
void clearAttributes(XMLElement *node) {
    while (node->FirstAttribute())
        node->DeleteAttribute(node->FirstAttribute()->Name());
}
void removeChildren(XMLElement *parent, const char *name) {
    for (auto node = parent->FirstChildElement(name); node;) {
        auto next = node->NextSiblingElement(name);
        parent->DeleteChild(node);
        node = next;
    }
}
std::vector<XMLElement *> children(XMLElement *parent, const char *name) {
    std::vector<XMLElement *> result;
    for (auto node = parent->FirstChildElement(name); node; node = node->NextSiblingElement(name))
        result.push_back(node);
    return result;
}
void parseXml(XMLDocument &xml, const std::string &text) {
    auto upper = text;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return std::toupper(c); });
    if (upper.find("<!DOCTYPE") != std::string::npos || upper.find("<!ENTITY") != std::string::npos)
        throw std::runtime_error("XML entities and DTDs are not supported");
    if (xml.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS || !xml.RootElement())
        throw std::runtime_error("Invalid source XML: " +
                                 std::string(xml.ErrorStr() ? xml.ErrorStr() : "missing root"));
}
XMLElement *parsedElement(XMLDocument &destination, const std::string &text) {
    XMLDocument temporary;
    parseXml(temporary, text);
    return temporary.RootElement()->DeepClone(&destination)->ToElement();
}
void saveXml(XMLDocument &xml, const fs::path &path) {
    tinyxml2::XMLPrinter printer;
    xml.Print(&printer);
    write(path, std::string("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n") + printer.CStr());
}
Json parameter(const Json &p, const std::vector<std::string> &names,
               const Json &fallback = nullptr) {
    for (const auto &name : names) {
        const Json *value = &p;
        bool found = true;
        for (const auto &part : split(name, '.')) {
            if (!value->is_object() || !value->contains(part)) {
                found = false;
                break;
            }
            value = &value->at(part);
        }
        if (found)
            return *value;
    }
    return fallback;
}
void noise(XMLElement *node, const std::string &path, Json parameters) {
    if (parameters.is_number())
        parameters = Json{{"stddev", parameters}};
    if (!parameters.is_object())
        return;
    auto target = ensure(node, path);
    target->SetAttribute("type",
                         parameters.value("type", attribute(target, "type", "gaussian")).c_str());
    for (const char *key : {"mean", "stddev", "bias_mean", "bias_stddev", "dynamic_bias_stddev",
                            "dynamic_bias_correlation_time", "precision"})
        if (parameters.contains(key))
            set(target, key, parameters[key]);
}
bool editedParameters(XMLElement *node, const std::string &kind, const Json &parameters,
                      const Json &baseline) {
    std::map<std::string, std::vector<std::string>> paths;
    if (kind == "camera" || kind == "depth_camera") {
        paths = {{"width", {"camera/image/width"}},
                 {"height", {"camera/image/height"}},
                 {"horizontal_fov", {"camera/horizontal_fov"}},
                 {"near", {"camera/clip/near"}},
                 {"far", {"camera/clip/far"}}};
        if (kind == "depth_camera")
            for (const std::string field : {"near", "far"})
                paths[field].push_back("camera/depth_camera/clip/" + field);
    } else if (kind == "contact")
        paths["collision"] = {"contact/collision"};
    else if (kind == "airspeed")
        paths["noise"] = {"air_speed/pressure/noise"};
    else if (kind == "barometer")
        paths["pressure_noise"] = {"air_pressure/pressure/noise"};
    else if (kind == "magnetometer")
        for (char axis : std::string("xyz"))
            paths["noise"].push_back("magnetometer/" + std::string(1, axis) + "/noise");
    else if (kind == "imu")
        for (auto pair : std::vector<std::pair<std::string, std::string>>{
                 {"accel", "linear_acceleration"}, {"gyro", "angular_velocity"}})
            for (char axis : std::string("xyz"))
                paths[pair.first + "_noise"].push_back("imu/" + pair.second + "/" + axis +
                                                       "/noise");
    else if (kind == "gps")
        for (const std::string field : {"position", "velocity"})
            for (const std::string axis : {"horizontal", "vertical"})
                paths[field + "_noise"].push_back("navsat/" + field + "_sensing/" + axis +
                                                  "/noise");
    else if (kind == "lidar") {
        std::string prefix = find(node, "ray") && !find(node, "lidar") ? "ray" : "lidar";
        for (const std::string axis : {"horizontal", "vertical"})
            for (const std::string field : {"samples", "min_angle", "max_angle"})
                paths[axis + "_" + field] = {prefix + "/scan/" + axis + "/" + field};
        for (const std::string field : {"min", "max"})
            paths[field + "_range"] = {prefix + "/range/" + field};
    }
    std::vector<std::string> changed;
    for (auto it = parameters.begin(); it != parameters.end(); ++it)
        if (!baseline.contains(it.key()) || *it != baseline[it.key()]) {
            if (!paths.count(it.key()))
                return false;
            changed.push_back(it.key());
        }
    for (const auto &key : changed)
        for (const auto &path : paths[key]) {
            if (path.size() >= 6 && path.substr(path.size() - 6) == "/noise")
                noise(node, path, parameters[key]);
            else
                set(node, path, parameters[key]);
        }
    return true;
}
XMLElement *sensorXml(XMLDocument &xml, const Json &sensor, XMLElement *original) {
    std::string kind = sensor.at("type");
    Json p = sensor.value("parameters", Json::object());
    auto node = original ? original->DeepClone(&xml)->ToElement() : xml.NewElement("sensor");
    node->SetAttribute("name", sensor["name"].get<std::string>().c_str());
    node->SetAttribute("type", types.at(kind).c_str());
    auto pose = set(node, "pose", numbers(sensor.value("pose", zeroPose)));
    clearAttributes(pose);
    if (!original)
        set(node, "always_on", true);
    if (!original || sensor.value("rate_hz", Json(0)) != sensor.value("source_rate_hz", Json()))
        set(node, "update_rate", sensor.value("rate_hz", Json(0)));
    std::string topic = sensor.value("topic", std::string());
    if (!topic.empty())
        set(node, "topic", topic);
    else
        removeChildren(node, "topic");
    if (kind == "contact" && !topic.empty())
        set(node, "contact/topic", topic);
    const std::map<std::string, std::string> aliases{{"gps", "navsat"},
                                                     {"ray", "gpu_lidar"},
                                                     {"lidar", "gpu_lidar"},
                                                     {"gpu_ray", "gpu_lidar"},
                                                     {"depth", "depth_camera"}};
    auto originalType = attribute(original, "type");
    if (aliases.count(originalType))
        originalType = aliases.at(originalType);
    if (original && sensor.contains("source_parameters") &&
        sensor["source_parameters"].is_object() && originalType == types.at(kind) &&
        editedParameters(node, kind, p, sensor["source_parameters"]))
        return node;
    if (kind == "camera" || kind == "depth_camera") {
        set(node, "camera/horizontal_fov",
            parameter(p, {"horizontal_fov", "hfov", "hfov_rad"}, 1.0471975512));
        set(node, "camera/image/width", parameter(p, {"width", "image_width", "image.width"}, 640));
        set(node, "camera/image/height",
            parameter(p, {"height", "image_height", "image.height"}, 480));
        set(node, "camera/image/format",
            parameter(p, {"format", "image.format"},
                      kind == "depth_camera" ? "R_FLOAT32" : "R8G8B8"));
        set(node, "camera/clip/near", parameter(p, {"near", "near_clip", "clip.near"}, .1));
        set(node, "camera/clip/far", parameter(p, {"far", "far_clip", "clip.far"}, 100));
        if (kind == "depth_camera") {
            set(node, "camera/depth_camera/clip/near",
                parameter(p, {"depth_near", "near", "near_clip", "clip.near"}, .1));
            set(node, "camera/depth_camera/clip/far",
                parameter(p, {"depth_far", "far", "far_clip", "clip.far"}, 100));
        }
        if (p.contains("distortion") && p["distortion"].is_object())
            for (const char *key : {"k1", "k2", "k3", "p1", "p2"})
                if (p["distortion"].contains(key))
                    set(node, std::string("camera/distortion/") + key, p["distortion"][key]);
        if (p.contains("noise_stddev"))
            set(node, "camera/noise/stddev", p["noise_stddev"]);
    } else if (kind == "lidar") {
        for (auto old : children(node, "ray")) {
            if (!find(node, "lidar"))
                old->SetName("lidar");
            else
                node->DeleteChild(old);
        }
        for (const std::string axis : {"horizontal", "vertical"}) {
            Json defaults = axis == "horizontal" ? Json{360, -pi, pi} : Json{1, 0, 0};
            int index = 0;
            for (const std::string field : {"samples", "min_angle", "max_angle"})
                set(node, "lidar/scan/" + axis + "/" + field,
                    parameter(
                        p, {axis + "_" + field, axis + "." + field, "scan." + axis + "." + field},
                        defaults[index++]));
            set(node, "lidar/scan/" + axis + "/resolution",
                parameter(p, {axis + "_resolution", axis + ".resolution"}, 1));
        }
        for (const auto &pair : std::vector<std::pair<std::string, double>>{
                 {"min", .1}, {"max", 30}, {"resolution", .01}})
            set(node, "lidar/range/" + pair.first,
                parameter(p, {"range_" + pair.first, "range." + pair.first, pair.first + "_range"},
                          pair.second));
        set(node, "lidar/noise/type", "gaussian");
        set(node, "lidar/noise/stddev", parameter(p, {"noise_stddev"}, 0));
    } else if (kind == "contact") {
        set(node, "contact/collision", parameter(p, {"collision", "collision_name"}, "collision"));
        set(node, "contact/topic",
            topic.empty() ? sensor["name"].get<std::string>() + "/contacts" : topic);
    } else if (kind == "airspeed" || kind == "barometer") {
        auto prefix = types.at(kind);
        noise(node, prefix + "/pressure/noise",
              parameter(p, {"noise", "pressure_noise"},
                        Json{{"stddev", p.value("noise_stddev", Json(0))}}));
        if (kind == "barometer")
            set(node, prefix + "/reference_altitude", p.value("reference_altitude", Json(0)));
    } else if (kind == "magnetometer")
        for (const std::string axis : {"x", "y", "z"})
            noise(node, "magnetometer/" + axis + "/noise",
                  parameter(p, {axis + "_noise", "noise." + axis, "noise"},
                            Json{{"stddev", p.value("noise_stddev", Json(0))}}));
    else if (kind == "imu") {
        for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
                 {"angular_velocity", "gyro"}, {"linear_acceleration", "accel"}})
            for (const std::string axis : {"x", "y", "z"})
                noise(node, "imu/" + pair.first + "/" + axis + "/noise",
                      parameter(p,
                                {pair.first + "." + axis, "noise." + pair.first + "." + axis,
                                 pair.second + "_noise"},
                                Json{{"stddev", p.value(pair.second + "_noise_stddev", Json(0))}}));
        if (p.contains("enable_orientation"))
            set(node, "imu/enable_orientation", p["enable_orientation"]);
    } else if (kind == "gps")
        for (const std::string field : {"position", "velocity"})
            for (const std::string axis : {"horizontal", "vertical"})
                noise(
                    node, "navsat/" + field + "_sensing/" + axis + "/noise",
                    parameter(p,
                              {field + "_sensing." + axis, field + "_" + axis + "_noise",
                               field + "_noise"},
                              Json{{"stddev", p.value(field + "_" + axis + "_stddev", Json(0))}}));
    return node;
}
void sourceUris(XMLElement *root, const fs::path &workspace, const fs::path &source) {
    if (std::string(root->Name()) == "uri" && root->GetText()) {
        std::string value = root->GetText();
        auto begin = value.find_first_not_of(" \n\r\t"), end = value.find_last_not_of(" \n\r\t");
        value = begin == std::string::npos ? "" : value.substr(begin, end - begin + 1);
        if (!value.empty() &&
            (value.find("://") == std::string::npos || value.rfind("file://", 0) == 0)) {
            if (value.rfind("file://", 0) == 0)
                value.erase(0, 7);
            for (auto candidate : {workspace / value, source.parent_path() / value}) {
                auto path = fs::weakly_canonical(candidate);
                if (fs::is_regular_file(path) && within(path, workspace)) {
                    root->SetText(fs::relative(path, workspace).generic_string().c_str());
                    break;
                }
            }
        }
    }
    for (auto child = root->FirstChildElement(); child; child = child->NextSiblingElement())
        sourceUris(child, workspace, source);
}
void gazebo(const Json &document, const fs::path &workspace, const fs::path &output, Json &report) {
    XMLDocument xml;
    XMLElement *root;
    XMLElement *model;
    std::string source = document.value("source_sdf", std::string());
    fs::path sourcePath;
    if (!source.empty()) {
        sourcePath = asset(workspace, source);
        parseXml(xml, read(sourcePath));
        root = xml.RootElement();
        model = root->FirstChildElement("model");
        if (!model)
            throw std::runtime_error("Original SDF must contain a top-level model");
        sourceUris(root, workspace, sourcePath);
    } else {
        root = xml.NewElement("sdf");
        xml.InsertEndChild(root);
        model = xml.NewElement("model");
        root->InsertEndChild(model);
    }
    std::string version = attribute(root, "version", "1.11");
    auto components = split(version, '.');
    if (components.size() < 2 || std::stoi(components[0]) < 1 ||
        (std::stoi(components[0]) == 1 && std::stoi(components[1]) < 11))
        version = "1.11";
    root->SetAttribute("version", version.c_str());
    model->SetAttribute("name", document["name"].get<std::string>().c_str());
    XMLDocument saved;
    std::map<std::pair<std::string, std::string>, XMLElement *> originals, visuals;
    std::map<std::string, XMLElement *> links;
    std::set<std::string> knownTypes{"gps", "ray", "lidar", "gpu_ray", "depth"};
    for (const auto &type : types)
        knownTypes.insert(type.second);
    for (auto link : children(model, "link")) {
        std::string name = attribute(link, "name");
        for (auto sensor : children(link, "sensor")) {
            if (knownTypes.count(attribute(sensor, "type"))) {
                originals[{name, attribute(sensor, "name")}] =
                    sensor->DeepClone(&saved)->ToElement();
                link->DeleteChild(sensor);
            } else
                report["warnings"].push_back("Preserved unsupported source sensor " +
                                             attribute(sensor, "name") + " (" +
                                             attribute(sensor, "type") + ") unchanged in Gazebo.");
        }
        for (auto visual : children(link, "visual")) {
            visuals[{name, attribute(visual, "name")}] = visual->DeepClone(&saved)->ToElement();
            link->DeleteChild(visual);
        }
        links[name] = link;
    }
    std::set<std::string> desired;
    for (const auto &link : document["links"])
        desired.insert(link["name"]);
    for (auto link : children(model, "link"))
        if (!desired.count(attribute(link, "name"))) {
            links.erase(attribute(link, "name"));
            model->DeleteChild(link);
        }
    for (const auto &definition : document["links"]) {
        std::string name = definition["name"];
        auto link = links.count(name) ? links.at(name) : nullptr;
        if (!link) {
            link = xml.NewElement("link");
            link->SetAttribute("name", name.c_str());
            model->InsertEndChild(link);
        }
        links[name] = link;
        clearAttributes(set(link, "pose", numbers(definition.value("pose", zeroPose))));
        set(link, "inertial/mass", definition.value("mass", Json(1)));
        if (definition.contains("inertial_pose"))
            clearAttributes(set(link, "inertial/pose", numbers(definition["inertial_pose"])));
        Json inertia = definition.value("inertia", Json{1, 1, 1, 0, 0, 0});
        size_t i = 0;
        for (const char *key : {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"})
            set(link, std::string("inertial/inertia/") + key, inertia[i++]);
    }
    size_t index = 0;
    for (const auto &visual : document["visuals"]) {
        std::string linkName = visual["link"], name = visual["name"];
        auto key = std::make_pair(visual.value("source_link", linkName),
                                  visual.value("source_name", name));
        auto node = visuals.count(key) ? visuals.at(key)->DeepClone(&xml)->ToElement()
                                       : xml.NewElement("visual");
        node->SetAttribute("name", name.c_str());
        links.at(linkName)->InsertEndChild(node);
        removeChildren(node, "geometry");
        clearAttributes(set(node, "pose", numbers(visual.value("pose", zeroPose))));
        const auto &geometry = visual["geometry"];
        std::string kind = geometry["type"];
        Json scale = visual.value("scale", unitScale);
        if (kind == "mesh" || scale != unitScale) {
            std::string filename = "meshes/visual_" + std::to_string(index) + ".obj";
            obj(meshes(visual, workspace), output / filename);
            set(node, "geometry/mesh/uri", filename);
            set(node, "geometry/mesh/scale", numbers(scale));
        } else {
            std::vector<std::string> fields = kind == "box" ? std::vector<std::string>{"size"}
                                              : kind == "sphere"
                                                  ? std::vector<std::string>{"radius"}
                                                  : std::vector<std::string>{"radius", "length"};
            for (const auto &field : fields)
                set(node, "geometry/" + kind + "/" + field,
                    geometry[field].is_array() ? Json(numbers(geometry[field])) : geometry[field]);
        }
        if (visual.contains("color")) {
            set(node, "material/diffuse", numbers(visual["color"]));
            set(node, "material/ambient", numbers(visual["color"]));
        }
        ++index;
    }
    std::set<std::string> enabledSystems;
    for (const auto &sensor : document["sensors"]) {
        if (!sensor.value("enabled", true))
            continue;
        std::string parent = sensor["parent"], name = sensor["name"];
        auto key = std::make_pair(sensor.value("source_parent", parent),
                                  sensor.value("source_name", name));
        XMLElement *original = originals.count(key) ? originals.at(key) : nullptr;
        if (!original && sensor.contains("xml") && !sensor["xml"].get<std::string>().empty())
            original = parsedElement(saved, sensor["xml"]);
        auto node = sensorXml(xml, sensor, original);
        if (!source.empty())
            sourceUris(node, workspace, sourcePath);
        links.at(parent)->InsertEndChild(node);
        enabledSystems.insert(systems.at(sensor["type"]));
    }
    if (source.empty())
        for (const auto &joint : document.value("joints", Json::array())) {
            XMLElement *node;
            if (joint.contains("xml") && !joint["xml"].get<std::string>().empty())
                node = parsedElement(xml, joint["xml"]);
            else {
                node = xml.NewElement("joint");
                node->SetAttribute("name", joint["name"].get<std::string>().c_str());
                node->SetAttribute("type", joint.value("type", std::string("fixed")).c_str());
                set(node, "parent", joint["parent"]);
                set(node, "child", joint["child"]);
                set(node, "pose", numbers(joint.value("pose", zeroPose)));
            }
            model->InsertEndChild(node);
        }
    removeChildren(model, "plugin");
    for (auto link : children(model, "link"))
        removeChildren(link, "plugin");
    for (const auto &plugin : document.value("plugins", Json::array())) {
        std::string target = plugin.value("target", std::string("gazebo"));
        if (target != "gazebo" && target != "all" && target != "*")
            continue;
        auto node = plugin.contains("xml") && !plugin["xml"].get<std::string>().empty()
                        ? parsedElement(xml, plugin["xml"])
                        : xml.NewElement("plugin");
        if (std::string(node->Name()) != "plugin")
            throw std::runtime_error("Plugin XML must have a plugin root");
        node->SetAttribute("name", plugin["name"].get<std::string>().c_str());
        node->SetAttribute("filename", plugin["filename"].get<std::string>().c_str());
        if (!source.empty())
            sourceUris(node, workspace, sourcePath);
        std::string parent = plugin.value("parent", std::string("__model__"));
        (parent == "__model__" ? model : links.at(parent))->InsertEndChild(node);
    }
    // Source files may already have an XML declaration; serialize just the model tree.
    XMLDocument serial;
    serial.InsertEndChild(root->DeepClone(&serial));
    saveXml(serial, output / "model.sdf");
    XMLDocument config;
    auto configRoot = config.NewElement("model");
    config.InsertEndChild(configRoot);
    set(configRoot, "name", document["name"]);
    set(configRoot, "version", "1.0");
    set(configRoot, "sdf", "model.sdf")->SetAttribute("version", version.c_str());
    saveXml(config, output / "model.config");
    XMLDocument worldXml;
    auto worldRoot = worldXml.NewElement("sdf");
    worldRoot->SetAttribute("version", "1.11");
    worldXml.InsertEndChild(worldRoot);
    auto world = worldXml.NewElement("world");
    world->SetAttribute("name", "sensor_preview");
    worldRoot->InsertEndChild(world);
    std::vector<std::string> allSystems{"Physics", "UserCommands", "SceneBroadcaster"};
    allSystems.insert(allSystems.end(), enabledSystems.begin(), enabledSystems.end());
    for (const auto &system : allSystems) {
        std::string stem;
        for (char c : system) {
            if (c >= 'A' && c <= 'Z') {
                if (!stem.empty())
                    stem += '-';
                stem += char(c - 'A' + 'a');
            } else
                stem += c;
        }
        auto node = worldXml.NewElement("plugin");
        node->SetAttribute("filename", ("gz-sim-" + stem + "-system").c_str());
        node->SetAttribute("name", ("gz::sim::systems::" + system).c_str());
        world->InsertEndChild(node);
        if (system == "Sensors")
            set(node, "render_engine", "ogre2");
    }
    set(world, "gravity", "0 0 -9.80665");
    set(world, "magnetic_field", "0.000006 0.000023 -0.000042");
    set(world, "spherical_coordinates/surface_model", "EARTH_WGS84");
    set(world, "spherical_coordinates/world_frame_orientation", "ENU");
    for (const char *field : {"latitude_deg", "longitude_deg", "elevation"})
        set(world, std::string("spherical_coordinates/") + field, 0);
    saveXml(worldXml, output / "world-systems.sdf");
    report["warnings"].push_back("Merge world-systems.sdf systems into your world and set its "
                                 "geographic origin. Model plugins must be installed separately.");
    report["warnings"].push_back(
        "PX4 requires compatible actuator plugins, joints, airframe settings and primary sensor "
        "topic names. Export does not establish that integration.");
    if (source.empty())
        report["warnings"].push_back(
            "New geometry has no generated collision shapes or flight/vehicle dynamics. Configure "
            "them before physical simulation.");
}
void xform(std::ostream &out, const Matrix &matrix, const std::string &indent,
           const Json &scale = nullptr) {
    out << indent
        << "double3 xformOp:translate = " << array(Json{matrix[0][3], matrix[1][3], matrix[2][3]})
        << '\n'
        << indent << "quatd xformOp:orient = " << array(quaternion(matrix)) << '\n';
    Json order = {"xformOp:translate", "xformOp:orient"};
    if (!scale.is_null()) {
        out << indent << "double3 xformOp:scale = " << array(scale) << '\n';
        order.push_back("xformOp:scale");
    }
    out << indent << "uniform token[] xformOpOrder = " << order.dump() << '\n';
}
void usdMesh(std::ostream &out, const Json &mesh, const std::string &name,
             const std::string &indent) {
    const std::string pad = indent + "    ";
    out << indent << "def Mesh " << quote(name)
        << " (prepend apiSchemas = [\"MaterialBindingAPI\"])\n"
        << indent << "{\n"
        << pad << "point3f[] points = [";
    bool comma = false;
    for (const auto &vertex : mesh["vertices"]) {
        if (comma)
            out << ", ";
        comma = true;
        out << array(vertex);
    }
    out << "]\n" << pad << "int[] faceVertexCounts = [";
    comma = false;
    for (const auto &face : mesh["faces"]) {
        if (comma)
            out << ", ";
        comma = true;
        out << face.size();
    }
    out << "]\n" << pad << "int[] faceVertexIndices = [";
    comma = false;
    for (const auto &face : mesh["faces"])
        for (const auto &vertex : face) {
            if (comma)
                out << ", ";
            comma = true;
            out << vertex.get<size_t>();
        }
    out << "]\n"
        << pad << "uniform token subdivisionScheme = \"none\"\n"
        << pad << "bool doubleSided = true\n"
        << pad << "color3f[] primvars:displayColor = ["
        << array(Json{mesh["color"][0], mesh["color"][1], mesh["color"][2]}) << "]\n"
        << pad << "float[] primvars:displayOpacity = [" << str(mesh["color"][3]) << "]\n";
    if (!mesh["uv"].empty()) {
        out << pad << "texCoord2f[] primvars:st = [";
        comma = false;
        for (const auto &uv : mesh["uv"]) {
            if (comma)
                out << ", ";
            comma = true;
            out << array(uv);
        }
        out << "] (interpolation = \"vertex\")\n";
    }
    out << pad << "rel material:binding = <Material>\n"
        << pad << "def Material \"Material\"\n"
        << pad << "{\n"
        << pad << "    token outputs:surface.connect = <Shader.outputs:surface>\n"
        << pad << "    def Shader \"Shader\"\n"
        << pad << "    {\n"
        << pad << "        uniform token info:id = \"UsdPreviewSurface\"\n"
        << pad << "        color3f inputs:diffuseColor = "
        << array(Json{mesh["color"][0], mesh["color"][1], mesh["color"][2]}) << '\n'
        << pad << "        float inputs:opacity = " << str(mesh["color"][3]) << '\n'
        << pad << "        token outputs:surface\n";
    std::string texture = mesh["texture"];
    bool textured = !texture.empty() && !mesh["uv"].empty();
    if (textured)
        out << pad << "        color3f inputs:diffuseColor.connect = <../Texture.outputs:rgb>\n";
    out << pad << "    }\n";
    if (textured) {
        if (texture.find('@') != std::string::npos)
            throw std::runtime_error("USD texture paths must not contain @");
        out << pad << "    def Shader \"Texture\"\n"
            << pad << "    {\n"
            << pad << "        uniform token info:id = \"UsdUVTexture\"\n"
            << pad << "        asset inputs:file = @" << texture << "@\n"
            << pad << "        token inputs:sourceColorSpace = \"sRGB\"\n"
            << pad << "        float2 inputs:st.connect = <../UV.outputs:result>\n"
            << pad << "        float3 outputs:rgb\n"
            << pad << "    }\n"
            << pad << "    def Shader \"UV\"\n"
            << pad << "    {\n"
            << pad << "        uniform token info:id = \"UsdPrimvarReader_float2\"\n"
            << pad << "        token inputs:varname = \"st\"\n"
            << pad << "        float2 outputs:result\n"
            << pad << "    }\n";
    }
    out << pad << "}\n" << indent << "}\n";
}
void usd(const Json &document, const fs::path &workspace, const fs::path &output, Json &report) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17)
        << "#usda 1.0\n(\n    defaultPrim = \"Model\"\n    metersPerUnit = 1\n    upAxis = "
           "\"Z\"\n)\ndef Xform \"Model\"\n{\n";
    const Matrix optical{{{0, 0, -1, 0}, {-1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 1}}};
    size_t linkIndex = 0;
    for (const auto &link : document["links"]) {
        std::string linkName = link["name"];
        out << "    def Xform "
            << quote("link_" + std::to_string(linkIndex++) + "_" + usdName(linkName))
            << "\n    {\n";
        xform(out, matrix(link.value("pose", zeroPose)), "        ");
        out << "        custom string xovium:linkName = " << quote(linkName) << '\n';
        size_t index = 0;
        for (const auto &visual : document["visuals"]) {
            size_t i = index++;
            if (visual["link"] != linkName)
                continue;
            out << "        def Xform \"visual_" << i << "\"\n        {\n";
            xform(out, matrix(visual.value("pose", zeroPose)), "            ",
                  visual.value("scale", unitScale));
            size_t part = 0;
            for (const auto &mesh : meshes(visual, workspace))
                usdMesh(out, mesh, "part_" + std::to_string(part++), "            ");
            out << "        }\n";
        }
        index = 0;
        for (const auto &sensor : document["sensors"]) {
            size_t i = index++;
            if (sensor["parent"] != linkName)
                continue;
            std::string kind = sensor["type"], name = sensor["name"];
            bool camera = kind == "camera" || kind == "depth_camera";
            Json p = sensor.value("parameters", Json::object());
            out << "        def " << (camera ? "Camera" : "Xform") << " \"sensor_" << i << '_'
                << usdName(name) << "\"\n        {\n";
            Matrix transform = matrix(sensor.value("pose", zeroPose));
            if (camera)
                transform = multiply(transform, optical);
            xform(out, transform, "            ");
            out << "            custom string xovium:sensor = " << quote(sensor.dump())
                << "\n            custom bool xovium:enabled = "
                << (sensor.value("enabled", true) ? "true" : "false") << '\n';
            if (camera) {
                int width = parameter(p, {"width", "image_width", "image.width"}, 640),
                    height = parameter(p, {"height", "image_height", "image.height"}, 480);
                double fov = parameter(p, {"horizontal_fov", "hfov", "hfov_rad"}, 1.0471975512),
                       aperture = 20.955;
                double near = parameter(p, {"near", "near_clip", "clip.near"}, .1),
                       far = parameter(p, {"far", "far_clip", "clip.far"}, 100);
                out << "            float horizontalAperture = " << aperture
                    << "\n            float verticalAperture = " << aperture * height / width
                    << "\n            float focalLength = " << aperture / (2 * std::tan(fov / 2))
                    << "\n            float2 clippingRange = " << array(Json{near, far})
                    << "\n            custom bool px4:camera:enabled = "
                    << (sensor.value("enabled", true) && kind == "camera" ? "true" : "false")
                    << "\n            custom int px4:camera:width = " << width
                    << "\n            custom int px4:camera:height = " << height
                    << "\n            custom int px4:camera:fps = "
                    << std::max(1, static_cast<int>(std::nearbyint(sensor.value("rate_hz", 30.0))))
                    << '\n';
                if (p.contains("port"))
                    out << "            custom int px4:camera:port = " << str(p["port"]) << '\n';
                if (p.contains("host"))
                    out << "            custom string px4:camera:host = " << quote(p["host"])
                        << '\n';
            }
            out << "        }\n";
        }
        out << "    }\n";
    }
    out << "}\n";
    write(output / "model.usda", out.str());
    report["warnings"].push_back(
        "USD preserves link/visual/sensor placement. Joints, collision, mass and vehicle dynamics "
        "remain in project.json; no physics adapter is generated.");
    report["warnings"].push_back(
        "RGB Camera prims include Xovium px4:camera settings. Streaming needs the installed "
        "px4.isaac.camera extension. Depth cameras need a render product and depth annotator; "
        "other sensor types are metadata only.");
}
const char *unrealRecipe();
Json flattenedMeshes(const Json &document, const fs::path &workspace) {
    std::map<std::string, Matrix> links;
    for (const auto &link : document["links"])
        links[link["name"]] = matrix(link.value("pose", zeroPose));
    Json parts = Json::array();
    for (const auto &visual : document["visuals"]) {
        auto transform = multiply(links.at(visual["link"]), matrix(visual.value("pose", zeroPose)));
        Json scale = visual.value("scale", unitScale);
        for (auto mesh : meshes(visual, workspace)) {
            for (auto &vertex : mesh["vertices"]) {
                Json scaled = Json::array();
                for (int i = 0; i < 3; ++i)
                    scaled.push_back(vertex[i].get<double>() * scale[i].get<double>());
                vertex = point(transform, scaled);
            }
            parts.push_back(std::move(mesh));
        }
    }
    return parts;
}
void unreal(const Json &document, const fs::path &workspace, const fs::path &output, Json &report) {
    std::map<std::string, Matrix> links;
    for (const auto &link : document["links"])
        links[link["name"]] = matrix(link.value("pose", zeroPose));
    Json entries = Json::array(), cameras = Json::array();
    size_t index = 0;
    for (const auto &visual : document["visuals"]) {
        Json parts = meshes(visual, workspace);
        for (auto &mesh : parts) {
            for (auto &vertex : mesh["vertices"])
                vertex = Json{100 * vertex[0].get<double>(), -100 * vertex[1].get<double>(),
                              100 * vertex[2].get<double>()};
            for (auto &face : mesh["faces"])
                std::reverse(face.begin(), face.end());
        }
        std::string filename = "meshes/visual_" + std::to_string(index++) + ".obj";
        obj(parts, output / filename);
        auto transform = multiply(links.at(visual["link"]), matrix(visual.value("pose", zeroPose)));
        double signs[3] = {1, -1, 1};
        Matrix rotation = transform;
        Json position = Json::array();
        for (int i = 0; i < 3; ++i) {
            position.push_back(100 * signs[i] * transform[i][3]);
            for (int j = 0; j < 3; ++j)
                rotation[i][j] = signs[i] * transform[i][j] * signs[j];
        }
        entries.push_back(Json{{"name", visual["name"]},
                               {"file", filename},
                               {"position", position},
                               {"quaternion_wxyz", quaternion(rotation)},
                               {"scale", visual.value("scale", unitScale)}});
    }
    for (const auto &sensor : document["sensors"]) {
        std::string kind = sensor["type"];
        if ((kind != "camera" && kind != "depth_camera") || !sensor.value("enabled", true))
            continue;
        auto transform =
            multiply(links.at(sensor["parent"]), matrix(sensor.value("pose", zeroPose)));
        double signs[3] = {1, -1, 1};
        Matrix rotation = transform;
        Json position = Json::array();
        for (int i = 0; i < 3; ++i) {
            position.push_back(100 * signs[i] * transform[i][3]);
            for (int j = 0; j < 3; ++j)
                rotation[i][j] = signs[i] * transform[i][j] * signs[j];
        }
        double fov = parameter(sensor.value("parameters", Json::object()),
                               {"horizontal_fov", "hfov", "hfov_rad"}, 1.0471975512);
        cameras.push_back(Json{{"name", sensor["name"]},
                               {"position", position},
                               {"quaternion_wxyz", quaternion(rotation)},
                               {"fov", fov * 180 / pi}});
    }
    write(output / "scene.json",
          Json{{"name", document["name"]}, {"meshes", entries}, {"cameras", cameras}}.dump(2) +
              "\n");
    write(output / "import_unreal.py", unrealRecipe());
    // Xovium's vehicle adapter consumes one combined canonical FLU/metre visual.
    // Keep the editor scene's centimetre/left-handed per-visual meshes separate.
    obj(flattenedMeshes(document, workspace), output / "model.obj", "");
    report["warnings"].push_back(
        "Run import_unreal.py in Unreal Editor with the Python Editor Script Plugin enabled. It "
        "creates visual mesh and camera actors in the current level; review and save that level.");
    report["warnings"].push_back(
        "OBJ files under meshes/ and scene.json use centimetres and Unreal handedness. model.obj "
        "is a combined FLU/metre mesh for Xovium's adapter. Collision, joints, dynamics and all "
        "sensor sampling/streaming require runtime components. Camera actors alone do not generate "
        "sensor images.");
}
void flightgear(const Json &document, const fs::path &workspace, const fs::path &output,
                Json &report) {
    Json parts = flattenedMeshes(document, workspace);
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "AC3Db\n";
    size_t index = 0;
    for (const auto &mesh : parts) {
        out << "MATERIAL \"part_" << index++ << "\" rgb "
            << numbers(Json{mesh["color"][0], mesh["color"][1], mesh["color"][2]})
            << " amb 0.2 0.2 0.2 emis 0 0 0 spec 0 0 0 shi 0 trans "
            << number(1 - mesh["color"][3].get<double>()) << '\n';
    }
    out << "OBJECT world\nkids " << parts.size() << '\n';
    index = 0;
    for (const auto &mesh : parts) {
        out << "OBJECT poly\nname \"part_" << index << "\"\n";
        std::string texture = mesh["texture"];
        if (!texture.empty())
            out << "texture " << quote(texture) << '\n';
        out << "numvert " << mesh["vertices"].size() << '\n';
        for (const auto &p : mesh["vertices"])
            out << numbers(Json{-p[0].get<double>(), p[2], p[1]}) << '\n';
        out << "numsurf " << mesh["faces"].size() << '\n';
        for (const auto &face : mesh["faces"]) {
            out << "SURF 0x30\nmat " << index << "\nrefs " << face.size() << '\n';
            for (const auto &vertex : face) {
                size_t i = vertex.get<size_t>();
                out << i << ' ' << numbers(mesh["uv"].empty() ? Json{0, 0} : mesh["uv"][i]) << '\n';
            }
        }
        out << "kids 0\n";
        ++index;
    }
    write(output / "model.ac", out.str());
    XMLDocument xml;
    auto root = xml.NewElement("PropertyList");
    xml.InsertEndChild(root);
    set(root, "path", "model.ac");
    saveXml(xml, output / "model.xml");
    report["warnings"].push_back(
        "model.xml/model.ac are visual model assets, not a complete flyable aircraft. Supply an "
        "aircraft -set.xml, FDM and property/telemetry integration separately.");
    report["warnings"].push_back(
        "Sensors and mounting poses are preserved in sensors.json in canonical FLU metres/radians; "
        "FlightGear does not execute those sensor definitions.");
}
void copyAssets(const fs::path &workspace, const fs::path &output) {
    auto assets = workspace / "assets";
    if (!fs::exists(assets))
        return;
    if (fs::is_symlink(assets))
        throw std::runtime_error("Workspace assets must not be a symbolic link");
    for (const auto &entry : fs::recursive_directory_iterator(assets)) {
        if (entry.is_symlink())
            throw std::runtime_error("Workspace assets must not contain symbolic links");
        if (entry.is_regular_file()) {
            auto target = output / "assets" / fs::relative(entry.path(), assets);
            fs::create_directories(target.parent_path());
            fs::copy_file(entry.path(), target);
        }
    }
}
Json exportInto(const Json &document, const fs::path &workspace, const std::string &target,
                const fs::path &output) {
    fs::create_directories(output);
    Json report{{"format", "xovium-export"}, {"version", 1},           {"target", target},
                {"name", document["name"]},  {"files", Json::array()}, {"warnings", Json::array()},
                {"sensors", Json::array()}};
    for (const auto &sensor : document.value("sensors", Json::array())) {
        std::string kind = sensor["type"];
        if (!types.count(kind))
            throw std::runtime_error("Unsupported sensor type: " + kind);
        bool camera = kind == "camera" || kind == "depth_camera";
        std::string status =
            !sensor.value("enabled", true) ? "disabled"
            : target == "gazebo"           ? "native_definition"
            : ((target == "isaac" || target == "sih" || target == "unreal") && camera)
                ? "native_camera"
                : "metadata_only";
        report["sensors"].push_back(Json{{"id", sensor["id"]},
                                         {"name", sensor["name"]},
                                         {"type", kind},
                                         {"status", status},
                                         {"runtime_integration_required", status != "disabled"}});
    }
    copyAssets(workspace, output);
    write(output / "project.json", document.dump(2) + "\n");
    write(output / "sensors.json", Json{{"frame", "FLU"},
                                        {"units", "m"},
                                        {"angles", "radians"},
                                        {"sensors", document.value("sensors", Json::array())},
                                        {"plugins", document.value("plugins", Json::array())}}
                                           .dump(2) +
                                       "\n");
    if (target == "gazebo")
        gazebo(document, workspace, output, report);
    else if (target == "isaac" || target == "sih")
        usd(document, workspace, output, report);
    else if (target == "unreal")
        unreal(document, workspace, output, report);
    else if (target == "flightgear")
        flightgear(document, workspace, output, report);
    std::vector<std::string> files{"report.json"};
    for (const auto &entry : fs::recursive_directory_iterator(output))
        if (entry.is_regular_file())
            files.push_back(fs::relative(entry.path(), output).generic_string());
    std::sort(files.begin(), files.end());
    report["files"] = files;
    write(output / "report.json", report.dump(2) + "\n");
    return report;
}

const char *unrealRecipe() {
    return R"UNREAL("""Run in Unreal Editor with Python Editor Script Plugin enabled. Creates actors in the current level."""
from pathlib import Path
import json
import unreal
root = Path(__file__).resolve().parent
scene = json.loads((root / "scene.json").read_text())
folder = "/Game/XoviumModels/" + "".join(c if c.isalnum() or c == "_" else "_" for c in scene["name"])
unreal.EditorAssetLibrary.make_directory(folder)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

def place(kind, entry):
    w, x, y, z = entry["quaternion_wxyz"]
    actor = actors.spawn_actor_from_class(kind, unreal.Vector(*entry["position"]), unreal.Quat(x, y, z, w).rotator())
    actor.set_actor_label(entry["name"])
    return actor

for index, entry in enumerate(scene["meshes"]):
    task = unreal.AssetImportTask()
    task.filename = str(root / entry["file"])
    task.destination_path = folder
    task.destination_name = "visual_" + str(index)
    task.automated = True
    task.save = True
    options = unreal.FbxImportUI()
    options.import_mesh = True
    options.import_as_skeletal = False
    options.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    options.static_mesh_import_data.set_editor_property("convert_scene", False)
    options.static_mesh_import_data.set_editor_property("convert_scene_unit", False)
    options.static_mesh_import_data.set_editor_property("combine_meshes", True)
    task.options = options
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    meshes = [unreal.load_asset(path) for path in task.imported_object_paths]
    mesh = next((asset for asset in meshes if isinstance(asset, unreal.StaticMesh)), None)
    if mesh is None:
        raise RuntimeError("Unreal did not import " + entry["file"])
    actor = place(unreal.StaticMeshActor, entry)
    actor.static_mesh_component.set_static_mesh(mesh)
    actor.set_actor_scale3d(unreal.Vector(*entry["scale"]))
for entry in scene["cameras"]:
    actor = place(unreal.CameraActor, entry)
    actor.get_camera_component().set_field_of_view(entry["fov"])
print("Imported model geometry and camera poses. Configure physics and sensor runtime components separately.")
)UNREAL";
}

} // namespace

Json exportProject(const Json &document, const fs::path &workspacePath, const std::string &target,
                   const fs::path &outputPath) {
    const std::set<std::string> targets{"gazebo", "isaac", "sih", "unreal", "flightgear"};
    if (!targets.count(target))
        throw std::runtime_error("Unknown export target: " + target);
    auto workspace = fs::weakly_canonical(fs::absolute(workspacePath));
    auto requested = fs::absolute(outputPath);
    if (fs::is_symlink(requested))
        throw std::runtime_error("Export directory must not be a symbolic link");
    auto output = fs::weakly_canonical(requested);
    if (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))
        throw std::runtime_error("Export directory must be empty");
    if (output == workspace || within(output, workspace / "assets"))
        throw std::runtime_error("Export output must not overwrite workspace assets");
    fs::create_directories(output.parent_path());
    std::string pattern = (output.parent_path() / ".xovium-export-XXXXXX").string();
    std::vector<char> temporary(pattern.begin(), pattern.end());
    temporary.push_back(0);
    char *created = mkdtemp(temporary.data());
    if (!created)
        throw std::runtime_error("Cannot create temporary export directory");
    fs::path staging = created;
    try {
        auto stage = staging / "export";
        Json result = exportInto(document, workspace, target, stage);
        if (fs::exists(output)) {
            if (!fs::is_directory(output) || !fs::is_empty(output) || fs::is_symlink(output))
                throw std::runtime_error("Export directory changed during export");
            fs::remove(output);
        }
        fs::rename(stage, output);
        fs::remove(staging);
        return result;
    } catch (...) {
        std::error_code ignored;
        fs::remove_all(staging, ignored);
        throw;
    }
}
} // namespace xovium
