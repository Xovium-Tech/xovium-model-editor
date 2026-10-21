#include "project.hpp"
#include <algorithm>
#include <archive.h>
#include <archive_entry.h>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <poll.h>
#include <random>
#include <set>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <tinyxml2.h>
#include <tuple>
#include <unistd.h>
#include <vector>
#ifndef XOVIUM_SOURCE_DIR
#define XOVIUM_SOURCE_DIR "."
#endif

namespace xovium {
namespace {
constexpr std::uint64_t maxFileBytes = 512ULL * 1024 * 1024, maxBytes = 2ULL * 1024 * 1024 * 1024;
constexpr std::size_t maxFiles = 10000;
fs::path sourceRoot = XOVIUM_SOURCE_DIR;
Json catalog;
using Element = tinyxml2::XMLElement;
using Xml = std::unique_ptr<tinyxml2::XMLDocument>;
using Matrix = std::array<std::array<double, 4>, 4>;
[[noreturn]] void fail(const std::string &message) {
    throw std::runtime_error(message);
}
std::string lower(std::string s) {
    for (char &c : s)
        c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}
std::string trim(const std::string &s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos)
        return {};
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}
bool starts(const std::string &s, const std::string &prefix) {
    return s.compare(0, prefix.size(), prefix) == 0;
}
bool within(const fs::path &path, const fs::path &root) {
    auto p = path.begin(), r = root.begin();
    for (; r != root.end(); ++r, ++p)
        if (p == path.end() || *p != *r)
            return false;
    return true;
}
std::string id() {
    static std::random_device random;
    std::array<unsigned char, 16> b{};
    for (auto &n : b)
        n = static_cast<unsigned char>(random());
    b[6] = (b[6] & 15) | 64;
    b[8] = (b[8] & 63) | 128;
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < b.size(); ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10)
            out << '-';
        out << std::setw(2) << int(b[i]);
    }
    return out.str();
}
struct TempDir {
    fs::path path;
    explicit TempDir(const fs::path &parent, const std::string &prefix = ".xovmodel-") {
        fs::create_directories(parent);
        std::string pattern = (parent / (prefix + "XXXXXX")).string();
        std::vector<char> chars(pattern.begin(), pattern.end());
        chars.push_back(0);
        if (!mkdtemp(chars.data()))
            fail("Cannot create temporary directory: " + std::string(std::strerror(errno)));
        path = chars.data();
    }
    ~TempDir() {
        std::error_code error;
        fs::remove_all(path, error);
    }
    TempDir(const TempDir &) = delete;
    TempDir &operator=(const TempDir &) = delete;
};
std::string readText(const fs::path &path, std::uint64_t limit = maxFileBytes) {
    if (!fs::is_regular_file(path))
        fail("Expected a regular file: " + path.string());
    if (fs::file_size(path) > limit)
        fail("File exceeds supported size: " + path.string());
    std::ifstream file(path, std::ios::binary);
    if (!file)
        fail("Cannot read " + path.string());
    std::ostringstream data;
    data << file.rdbuf();
    if (file.bad())
        fail("Cannot read " + path.string());
    return data.str();
}
Json readJson(const fs::path &path) {
    try {
        return Json::parse(readText(path));
    } catch (const Json::exception &error) {
        fail("Invalid JSON in " + path.string() + ": " + error.what());
    }
}
void writeText(const fs::path &path, const std::string &text) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file)
        fail("Cannot write " + path.string());
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
    file.close();
    if (!file)
        fail("Cannot write " + path.string());
}
void atomicJson(const fs::path &path, const Json &value) {
    if (fs::is_symlink(path))
        fail("Refusing to replace a symlink: " + path.string());
    TempDir temp(path.parent_path(), ".json-");
    writeText(temp.path / "value", value.dump(2) + "\n");
    fs::rename(temp.path / "value", path);
}
const Json &field(const Json &object, const char *key) {
    static const Json null;
    return object.is_object() && object.contains(key) ? object.at(key) : null;
}
std::string text(const Json &value, const std::string &label, bool empty = false) {
    if (!value.is_string())
        fail(label + " must be a string");
    std::string result = value.get<std::string>();
    if (result.size() > 4096 || (!empty && trim(result).empty()) ||
        std::any_of(result.begin(), result.end(), [](unsigned char c) { return c < 32; }))
        fail(label + " must be a valid " +
             (empty ? std::string("string") : std::string("nonempty string")));
    return result;
}
double number(const Json &value, const std::string &label,
              double minimum = -std::numeric_limits<double>::infinity(),
              double maximum = std::numeric_limits<double>::infinity(), bool integer = false) {
    if (!value.is_number() ||
        (integer && !value.is_number_integer() && !value.is_number_unsigned()))
        fail(label + " must be a " +
             (integer ? std::string("finite integer") : std::string("finite number")));
    double result = value.get<double>();
    if (!std::isfinite(result))
        fail(label + " must be finite");
    if (result < minimum || result > maximum)
        fail(label + " is outside the supported range");
    return result;
}
void vector(const Json &value, std::size_t count, const std::string &label,
            double minimum = -std::numeric_limits<double>::infinity(),
            double maximum = std::numeric_limits<double>::infinity()) {
    if (!value.is_array() || value.size() != count)
        fail(label + " must contain " + std::to_string(count) + " numbers");
    for (const auto &item : value)
        number(item, label, minimum, maximum);
}
Xml xml(const std::string &value) {
    if (value.size() > 16 * 1024 * 1024)
        fail("XML must be under 16 MiB");
    std::string uppercase = value;
    for (char &c : uppercase)
        c = char(std::toupper(static_cast<unsigned char>(c)));
    if (uppercase.find("<!DOCTYPE") != std::string::npos ||
        uppercase.find("<!ENTITY") != std::string::npos)
        fail("XML entities and DTDs are not supported");
    auto doc = std::make_unique<tinyxml2::XMLDocument>();
    if (doc->Parse(value.data(), value.size()) != tinyxml2::XML_SUCCESS || !doc->RootElement() ||
        doc->RootElement()->NextSiblingElement())
        fail("Invalid XML: " + std::string(doc->ErrorStr() ? doc->ErrorStr() : "missing root"));
    return doc;
}
Xml xml(const Json &value) {
    if (!value.is_string())
        fail("XML must be a string");
    return xml(value.get<std::string>());
}
std::string xmlString(const tinyxml2::XMLNode *node) {
    tinyxml2::XMLPrinter printer;
    node->Accept(&printer);
    return printer.CStr();
}
std::string localName(const Element *node) {
    std::string name = node->Name();
    auto colon = name.find(':');
    return colon == std::string::npos ? name : name.substr(colon + 1);
}
void walk(Element *element, const std::function<void(Element *)> &visit) {
    if (!element)
        return;
    visit(element);
    for (auto *child = element->FirstChildElement(); child; child = child->NextSiblingElement())
        walk(child, visit);
}
Element *child(Element *element, const std::string &path) {
    std::stringstream input(path);
    std::string part;
    while (element && std::getline(input, part, '/'))
        element = element->FirstChildElement(part.c_str());
    return element;
}
std::string content(Element *element, const std::string &path, const std::string &fallback = "") {
    auto *node = child(element, path);
    return node && node->GetText() ? node->GetText() : fallback;
}
std::string attribute(Element *node, const char *name, const std::string &fallback = "") {
    const char *value = node->Attribute(name);
    return value ? value : fallback;
}
std::vector<Element *> children(Element *parent, const char *tag) {
    std::vector<Element *> list;
    for (auto *p = parent->FirstChildElement(tag); p; p = p->NextSiblingElement(tag))
        list.push_back(p);
    return list;
}
const std::set<std::string> resourceTags = {
    "uri",           "albedo_map",   "normal_map",     "roughness_map",
    "metalness_map", "emissive_map", "environment_map"};
void validateMesh(const Json &meshes, const fs::path &workspace) {
    if (!meshes.is_array() || meshes.empty())
        fail("Mesh JSON must contain a nonempty mesh list");
    for (const auto &mesh : meshes) {
        if (!mesh.is_object())
            fail("Invalid mesh record");
        const auto &vertices = field(mesh, "vertices");
        const auto &faces = field(mesh, "faces");
        if (!vertices.is_array() || vertices.empty() || !faces.is_array() || faces.empty())
            fail("Mesh requires vertices and triangle faces");
        for (const auto &point : vertices)
            vector(point, 3, "Mesh vertex");
        for (const auto &face : faces) {
            if (!face.is_array() || face.size() != 3)
                fail("Mesh faces must be triangles");
            for (const auto &index : face)
                number(index, "Triangle index", 0, double(vertices.size() - 1), true);
        }
        const Json uv = mesh.value("uv", Json::array());
        if (!uv.is_array() || (!uv.empty() && uv.size() != vertices.size()))
            fail("Mesh UVs must match vertices");
        for (const auto &p : uv)
            vector(p, 2, "Texture coordinate");
        vector(mesh.value("color", Json::array({.7, .7, .7, 1})), 4, "Mesh color", 0, 1);
        if (mesh.contains("texture") && !mesh["texture"].is_null() && mesh["texture"] != "")
            assetPath(text(mesh["texture"], "Texture path"), workspace);
        Json dependencies = mesh.value("dependencies", Json::array());
        if (!dependencies.is_array())
            fail("Mesh dependencies must be a list");
        for (const auto &path : dependencies)
            assetPath(text(path, "Texture dependency"), workspace);
    }
}
std::vector<fs::path> safeFiles(const fs::path &root) {
    if (fs::is_symlink(root))
        fail("Asset root must not be a symlink");
    std::vector<fs::path> files;
    std::uint64_t total = 0;
    if (!fs::exists(root))
        return files;
    for (const auto &entry : fs::recursive_directory_iterator(root)) {
        if (entry.is_symlink())
            fail("Symlinks are not allowed in a model bundle");
        if (entry.is_directory())
            continue;
        if (!entry.is_regular_file())
            fail("Only regular files can be bundled");
        auto size = entry.file_size();
        if (size > maxFileBytes)
            fail("Asset exceeds the 512 MiB size limit");
        total += size;
        files.push_back(entry.path());
        if (total > maxBytes || files.size() >= maxFiles)
            fail("Model bundle exceeds size/file limits");
    }
    std::sort(files.begin(), files.end());
    return files;
}
void archiveCheck(int code, archive *handle, const std::string &action) {
    if (code < ARCHIVE_OK)
        fail(action + ": " +
             std::string(archive_error_string(handle) ? archive_error_string(handle)
                                                      : "archive error"));
}
struct ArchiveReader {
    archive *value = archive_read_new();
    ~ArchiveReader() {
        archive_read_free(value);
    }
};
struct ArchiveWriter {
    archive *value = archive_write_new();
    ~ArchiveWriter() {
        archive_write_free(value);
    }
};
void unpack(const fs::path &source, const fs::path &destination) {
    ArchiveReader archive;
    archive_read_support_format_zip(archive.value);
    archive_read_support_filter_all(archive.value);
    archiveCheck(archive_read_open_filename(archive.value, source.c_str(), 65536), archive.value,
                 "Cannot open project archive");
    archive_entry *entry = nullptr;
    std::set<std::string> seen;
    std::uint64_t total = 0;
    std::size_t count = 0;
    int status;
    while ((status = archive_read_next_header(archive.value, &entry)) == ARCHIVE_OK) {
        if (++count > maxFiles)
            fail("Project archive has too many entries");
        const char *raw = archive_entry_pathname(entry);
        if (!raw)
            fail("Archive entry has no name");
        std::string name = raw;
        if (!name.empty() && name.back() == '/')
            name.pop_back();
        fs::path relative = name;
        bool bad = name.empty() || relative.is_absolute() || name.find('\\') != std::string::npos ||
                   name.find(':') != std::string::npos ||
                   std::any_of(name.begin(), name.end(), [](unsigned char c) { return c < 32; });
        std::stringstream parts(name);
        std::string part;
        while (std::getline(parts, part, '/'))
            if (part.empty() || part == "." || part == "..")
                bad = true;
        if (bad || (name != "project.json" && *relative.begin() != "assets"))
            fail("Unsafe project archive path: " + name);
        if (!seen.insert(name).second)
            fail("Duplicate archive path: " + name);
        auto type = archive_entry_filetype(entry);
        if (archive_entry_symlink(entry) || archive_entry_hardlink(entry) ||
            (type != AE_IFREG && type != AE_IFDIR && type != 0) ||
            archive_entry_is_encrypted(entry))
            fail("Archive links, encryption and special files are not allowed");
        const auto size = archive_entry_size(entry);
        if (size < 0 || std::uint64_t(size) > maxFileBytes ||
            total + std::uint64_t(size) > maxBytes)
            fail("Project archive exceeds size limits");
        total += std::uint64_t(size);
        const fs::path target = destination / relative;
        if (type == AE_IFDIR) {
            fs::create_directories(target);
            continue;
        }
        fs::create_directories(target.parent_path());
        std::ofstream output(target, std::ios::binary);
        if (!output)
            fail("Cannot extract archive file");
        std::array<char, 65536> buffer{};
        std::uint64_t actual = 0;
        la_ssize_t amount;
        while ((amount = archive_read_data(archive.value, buffer.data(), buffer.size())) > 0) {
            actual += std::uint64_t(amount);
            if (actual > std::uint64_t(size) || actual > maxFileBytes)
                fail("Archive entry exceeds declared size");
            output.write(buffer.data(), amount);
            if (!output)
                fail("Cannot write extracted asset");
        }
        if (amount < 0)
            archiveCheck(int(amount), archive.value, "Cannot extract project");
        if (actual != std::uint64_t(size))
            fail("Truncated archive entry");
    }
    if (status != ARCHIVE_EOF)
        archiveCheck(status, archive.value, "Cannot read project archive");
    archiveCheck(archive_read_close(archive.value), archive.value, "Cannot close project archive");
}
void addArchiveEntry(archive *archive, const std::string &name, const fs::path &file,
                     const std::string *memory = nullptr) {
    std::unique_ptr<archive_entry, decltype(&archive_entry_free)> entry(archive_entry_new(),
                                                                        archive_entry_free);
    archive_entry_set_pathname(entry.get(), name.c_str());
    archive_entry_set_filetype(entry.get(), AE_IFREG);
    archive_entry_set_perm(entry.get(), 0644);
    archive_entry_set_size(entry.get(),
                           static_cast<la_int64_t>(memory ? memory->size() : fs::file_size(file)));
    archiveCheck(archive_write_header(archive, entry.get()), archive,
                 "Cannot write archive header");
    auto write = [&](const char *bytes, std::size_t count) {
        while (count) {
            auto n = archive_write_data(archive, bytes, count);
            if (n <= 0)
                fail("Cannot write project archive");
            bytes += n;
            count -= std::size_t(n);
        }
    };
    if (memory)
        write(memory->data(), memory->size());
    else {
        std::ifstream input(file, std::ios::binary);
        if (!input)
            fail("Cannot read asset for archive");
        std::array<char, 65536> data{};
        while (input) {
            input.read(data.data(), data.size());
            if (input.gcount())
                write(data.data(), std::size_t(input.gcount()));
        }
        if (input.bad())
            fail("Cannot read asset for archive");
    }
    archiveCheck(archive_write_finish_entry(archive), archive, "Cannot finish archive entry");
}
} // namespace

void setRoot(const fs::path &root) {
    sourceRoot = fs::absolute(root);
    catalog = Json();
}
const Json &sensorCatalog() {
    if (catalog.is_null())
        catalog = readJson(sourceRoot / "backend/sensors.json");
    return catalog;
}
fs::path assetPath(const std::string &value, const fs::path &workspace, bool mustExist) {
    text(value, "Asset path");
    fs::path path = value;
    bool bad = path.is_absolute() || value.find('\\') != std::string::npos ||
               value.find(':') != std::string::npos;
    std::stringstream input(value);
    std::string part;
    while (std::getline(input, part, '/'))
        if (part.empty() || part == "." || part == "..")
            bad = true;
    if (value.empty() || value.back() == '/' || path.empty() || *path.begin() != "assets" || bad)
        fail("Asset path must stay inside assets/: " + value);
    if (workspace.empty())
        return path;
    fs::path root = fs::weakly_canonical(workspace), result = root;
    for (const auto &segment : path) {
        result /= segment;
        if (fs::is_symlink(result))
            fail("Asset symlinks are not allowed: " + value);
    }
    if (!within(fs::weakly_canonical(result), root))
        fail("Asset escapes workspace: " + value);
    if (mustExist && !fs::is_regular_file(result))
        fail("Missing project asset: " + value);
    return result;
}
void validateProject(const Json &document, const fs::path &workspace) {
    if (!document.is_object() || field(document, "format") != "xovium-model" ||
        !field(document, "version").is_number_integer() || field(document, "version") != 1)
        fail("Unsupported model project format or version");
    text(field(document, "name"), "Project name");
    const auto vehicle = field(document, "vehicle");
    if (vehicle != "fixed-wing" && vehicle != "rover" && vehicle != "custom")
        fail("Vehicle must be fixed-wing, rover or custom");
    if (field(document, "frame") != "FLU" || field(document, "units") != "m")
        fail("Projects use FLU axes and metres");
    for (const char *key : {"links", "visuals", "sensors", "plugins"})
        if (!field(document, key).is_array())
            fail(std::string(key) + " must be a list");
    if (document["links"].empty())
        fail("Project needs at least one link");
    std::set<std::string> links, ids;
    std::set<std::tuple<std::string, std::string, std::string>> names;
    auto unique = [&](const Json &item, const std::string &kind, const std::string &parent = "") {
        if (!item.is_object())
            fail("Invalid " + kind + " record");
        std::string name = text(field(item, "name"), kind + " name");
        if (name.find_first_of("/\\<>") != std::string::npos)
            fail("Invalid " + kind + " name: " + name);
        if (!names.emplace(kind, parent, name).second && kind != "plugin")
            fail("Duplicate " + kind + " name: " + name);
        if (kind != "link" && kind != "joint" &&
            !ids.insert(text(field(item, "id"), kind + " ID")).second)
            fail("Duplicate object ID");
    };
    for (const auto &link : document["links"]) {
        unique(link, "link");
        links.insert(link["name"].get<std::string>());
        vector(field(link, "pose"), 6, "Link pose");
        number(field(link, "mass"), "Link mass", 1e-12, 1e12);
        const auto &tensor = field(link, "inertia");
        vector(tensor, 6, "Link inertia");
        const double a = tensor[0], b = tensor[1], c = tensor[2], d = tensor[3], e = tensor[4],
                     f = tensor[5];
        if (std::min({a, b, c}) <= 0 || a * b - d * d <= 0 ||
            a * b * c + 2 * d * e * f - a * f * f - b * e * e - c * d * d <= 0)
            fail("Link inertia must be a positive-definite symmetric tensor");
        if (link.contains("inertial_pose"))
            vector(link["inertial_pose"], 6, "Inertial pose");
    }
    for (const auto &visual : document["visuals"]) {
        std::string parent = text(field(visual, "link"), "Visual parent");
        unique(visual, "visual", parent);
        if (!links.count(parent))
            fail("Visual has an unknown parent link");
        vector(field(visual, "pose"), 6, "Visual pose");
        vector(field(visual, "scale"), 3, "Visual scale", 1e-9, 1e9);
        vector(visual.value("color", Json::array({.7, .7, .7, 1})), 4, "Visual color", 0, 1);
        const auto &geometry = field(visual, "geometry");
        if (!geometry.is_object())
            fail("Visual requires geometry");
        std::string kind = text(field(geometry, "type"), "Geometry type");
        if (kind == "mesh") {
            auto path = assetPath(text(field(geometry, "path"), "Mesh path"), workspace);
            if (!workspace.empty())
                validateMesh(readJson(path), workspace);
        } else if (kind == "box")
            vector(field(geometry, "size"), 3, "Box size", 1e-9, 1e9);
        else if (kind == "sphere" || kind == "cylinder") {
            number(field(geometry, "radius"), "Radius", 1e-9, 1e9);
            if (kind == "cylinder")
                number(field(geometry, "length"), "Cylinder length", 1e-9, 1e9);
        } else
            fail("Unsupported geometry type: " + kind);
    }
    std::map<std::string, Json> types;
    for (const auto &type : sensorCatalog().at("types"))
        types.emplace(type.at("id").get<std::string>(), type);
    for (const auto &sensor : document["sensors"]) {
        auto parent = text(field(sensor, "parent"), "Sensor parent");
        unique(sensor, "sensor", parent);
        if (!links.count(parent))
            fail("Sensor has an unknown parent link");
        auto kind = text(field(sensor, "type"), "Sensor type");
        if (!types.count(kind))
            fail("Unsupported sensor type: " + kind);
        vector(field(sensor, "pose"), 6, "Sensor pose");
        if (!field(sensor, "enabled").is_boolean())
            fail("Sensor enabled must be boolean");
        number(field(sensor, "rate_hz"), "Sensor update rate", .01, 10000);
        text(sensor.value("topic", Json("")), "Sensor topic", true);
        const auto &parameters = field(sensor, "parameters");
        if (!parameters.is_object())
            fail("Sensor parameters must be an object");
        std::map<std::string, Json> definitions;
        for (const auto &p : types.at(kind)["parameters"])
            definitions.emplace(p.at("key").get<std::string>(), p);
        for (auto it = parameters.begin(); it != parameters.end(); ++it) {
            if (!definitions.count(it.key()))
                fail("Unknown " + kind + " parameter: " + it.key());
            const auto &spec = definitions.at(it.key());
            std::string type = spec.at("type");
            if (type == "int" || type == "float")
                number(it.value(), it.key(),
                       spec.value("min", -std::numeric_limits<double>::infinity()),
                       spec.value("max", std::numeric_limits<double>::infinity()), type == "int");
            else if (type == "string")
                text(it.value(), it.key(), true);
            else if (!it.value().is_boolean())
                fail(it.key() + " must be boolean");
        }
        Json params = types.at(kind)["default_parameters"];
        params.update(parameters);
        for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
                 {"near", "far"}, {"min_range", "max_range"}})
            if (params.contains(pair.first) &&
                params[pair.first].get<double>() >= params.at(pair.second).get<double>())
                fail(pair.second + " must exceed " + pair.first);
        for (const std::string axis : {"horizontal", "vertical"})
            if (params.contains(axis + "_min_angle") &&
                params[axis + "_min_angle"].get<double>() >
                    params.at(axis + "_max_angle").get<double>())
                fail(axis + " maximum angle must be at least minimum angle");
        if (sensor.contains("xml"))
            xml(sensor["xml"]);
    }
    for (const auto &plugin : document["plugins"]) {
        auto parent = plugin.value("parent", std::string("__model__"));
        unique(plugin, "plugin", parent);
        if (parent != "__model__" && !links.count(parent))
            fail("Plugin has an unknown parent link");
        text(field(plugin, "target"), "Plugin target");
        text(field(plugin, "filename"), "Plugin filename", true);
        auto node = xml(field(plugin, "xml"));
        if (std::string(node->RootElement()->Name()) != "plugin")
            fail("Plugin XML must contain a plugin element");
    }
    Json joints = document.value("joints", Json::array());
    if (!joints.is_array())
        fail("Joints must be a list");
    for (const auto &joint : joints) {
        unique(joint, "joint");
        if (links.count(joint.at("name").get<std::string>()))
            fail("Joint and link frame names must be distinct");
        auto parent = text(field(joint, "parent"), "Joint parent"),
             kid = text(field(joint, "child"), "Joint child");
        if ((parent != "world" && !links.count(parent)) || !links.count(kid))
            fail("Joint has an unknown parent or child");
        text(field(joint, "type"), "Joint type");
        vector(field(joint, "pose"), 6, "Joint pose");
        if (joint.contains("xml")) {
            auto node = xml(joint["xml"]);
            if (std::string(node->RootElement()->Name()) != "joint")
                fail("Joint XML must contain a joint element");
        }
    }
    if (document.contains("source_sdf")) {
        auto path = assetPath(text(document["source_sdf"], "Source SDF path"), workspace);
        if (!workspace.empty()) {
            auto tree = xml(readText(path, 16 * 1024 * 1024));
            auto root = fs::weakly_canonical(workspace);
            walk(tree->RootElement(), [&](Element *element) {
                if (!resourceTags.count(element->Name()) || !element->GetText())
                    return;
                std::string value = trim(element->GetText());
                if (value.empty() || starts(value, "__default__") || starts(value, "name://"))
                    return;
                if (value.find("://") != std::string::npos || fs::path(value).is_absolute() ||
                    value.find('\\') != std::string::npos)
                    fail("Source SDF resource must be a bundled relative path: " + value);
                auto target = fs::weakly_canonical(path.parent_path() / value);
                if (!within(target, root / "assets") || !fs::is_regular_file(target))
                    fail("Missing or nonportable source SDF resource: " + value);
                assetPath(target.lexically_relative(root).generic_string(), workspace);
            });
        }
    }
    if (document.contains("warnings")) {
        if (!document["warnings"].is_array())
            fail("Warnings must be a string list");
        for (const auto &warning : document["warnings"])
            if (!warning.is_string())
                fail("Warnings must be a string list");
    }
}
Json newProject(const std::string &name, const std::string &vehicle) {
    Json size = vehicle == "fixed-wing" ? Json::array({.8, 1.4, .15}) : Json::array({.4, .25, .25});
    double mass = vehicle == "fixed-wing" ? 1.5
                  : vehicle == "rover"    ? 20
                                          : 1,
           x = size[0], y = size[1], z = size[2];
    Json document = {
        {"format", "xovium-model"},
        {"version", 1},
        {"name", name},
        {"vehicle", vehicle},
        {"frame", "FLU"},
        {"units", "m"},
        {"links",
         Json::array(
             {{{"name", "base_link"},
               {"pose", Json::array({0, 0, 0, 0, 0, 0})},
               {"mass", mass},
               {"inertia", Json::array({mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12,
                                        mass * (x * x + y * y) / 12, 0, 0, 0})}}})},
        {"visuals", Json::array({{{"id", id()},
                                  {"name", "body"},
                                  {"link", "base_link"},
                                  {"pose", Json::array({0, 0, 0, 0, 0, 0})},
                                  {"scale", Json::array({1, 1, 1})},
                                  {"geometry", {{"type", "box"}, {"size", size}}},
                                  {"color", Json::array({.2, .55, .85, 1})}}})},
        {"sensors", Json::array()},
        {"plugins", Json::array()},
        {"joints", Json::array()},
        {"warnings", Json::array()}};
    validateProject(document);
    return document;
}
Json loadWorkspace(const fs::path &workspace) {
    if (fs::is_symlink(workspace) || fs::is_symlink(workspace / "project.json"))
        fail("Workspace and project file must not be symlinks");
    auto document = readJson(workspace / "project.json");
    validateProject(document, workspace);
    return document;
}
void writeWorkspace(const Json &document, const fs::path &workspace) {
    if (fs::is_symlink(workspace))
        fail("Workspace must not be a symlink");
    fs::create_directories(workspace);
    if (fs::is_symlink(workspace / "assets"))
        fail("Assets directory must not be a symlink");
    fs::create_directories(workspace / "assets");
    validateProject(document, workspace);
    atomicJson(workspace / "project.json", document);
}
fs::path saveProject(const fs::path &workspace, const fs::path &outputPath) {
    auto document = loadWorkspace(workspace);
    auto root = fs::canonical(workspace), output = fs::absolute(outputPath);
    if (lower(output.extension().string()) != ".xovmodel")
        fail("Save filename must end in .xovmodel");
    if (fs::is_symlink(output) || within(fs::weakly_canonical(output), root / "assets"))
        fail("Save outside the project assets directory");
    auto files = safeFiles(root / "assets");
    TempDir temp(output.parent_path(), ".archive-");
    ArchiveWriter archive;
    archiveCheck(archive_write_set_format_zip(archive.value), archive.value,
                 "Cannot select ZIP format");
    archiveCheck(archive_write_set_options(archive.value, "zip:compression=deflate"), archive.value,
                 "Cannot configure ZIP compression");
    archiveCheck(archive_write_open_filename(archive.value, (temp.path / "model.zip").c_str()),
                 archive.value, "Cannot open archive output");
    auto data = document.dump(2) + "\n";
    std::uint64_t total = data.size();
    for (const auto &file : files)
        total += fs::file_size(file);
    if (total > maxBytes)
        fail("Project archive exceeds size limits");
    addArchiveEntry(archive.value, "project.json", {}, &data);
    for (const auto &file : files)
        addArchiveEntry(archive.value, file.lexically_relative(root).generic_string(), file);
    archiveCheck(archive_write_close(archive.value), archive.value,
                 "Cannot finish project archive");
    fs::rename(temp.path / "model.zip", output);
    return output;
}

namespace {
Matrix matrix(const Json &pose) {
    vector(pose, 6, "Pose");
    double x = pose[0], y = pose[1], z = pose[2], r = pose[3], p = pose[4], h = pose[5],
           cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p), ch = std::cos(h),
           sh = std::sin(h);
    return {{{ch * cp, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr, x},
             {sh * cp, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr, y},
             {-sp, cp * sr, cp * cr, z},
             {0, 0, 0, 1}}};
}
Matrix multiply(const Matrix &a, const Matrix &b) {
    Matrix result{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                result[i][j] += a[i][k] * b[k][j];
    return result;
}
Matrix inverse(const Matrix &a) {
    Matrix out{};
    out[3][3] = 1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            out[i][j] = a[j][i];
        for (int j = 0; j < 3; ++j)
            out[i][3] -= out[i][j] * a[j][3];
    }
    return out;
}
Json pose(const Matrix &a) {
    double p = std::asin(std::clamp(-a[2][0], -1., 1.)), r, y;
    if (std::abs(std::cos(p)) > 1e-8) {
        r = std::atan2(a[2][1], a[2][2]);
        y = std::atan2(a[1][0], a[0][0]);
    } else {
        r = 0;
        y = std::atan2(-a[0][1], a[1][1]);
    }
    return Json::array({a[0][3], a[1][3], a[2][3], r, p, y});
}
double scalar(const std::string &value) {
    try {
        std::size_t used = 0;
        auto s = trim(value);
        double n = std::stod(s, &used);
        if (used != s.size() || !std::isfinite(n))
            fail("Invalid numeric SDF value: " + value);
        return n;
    } catch (const std::invalid_argument &) {
        fail("Invalid numeric SDF value: " + value);
    } catch (const std::out_of_range &) {
        fail("Invalid numeric SDF value: " + value);
    }
}
Json numbers(const std::string &value, std::size_t count, const std::string &fallback = "") {
    std::stringstream in(trim(value).empty() ? fallback : value);
    Json out = Json::array();
    std::string token;
    while (in >> token)
        out.push_back(scalar(token));
    vector(out, count, "SDF numeric value");
    return out;
}
Json color(const std::string &value) {
    std::stringstream in(value.empty() ? "0.7 0.7 0.7 1" : value);
    Json out = Json::array();
    std::string token;
    while (in >> token)
        out.push_back(scalar(token));
    if (out.size() == 3)
        out.push_back(1.0);
    vector(out, 4, "SDF visual color", 0, 1);
    return out;
}
std::pair<std::string, Matrix> sdfPose(Element *element, const std::string &parent) {
    auto *p = element->FirstChildElement("pose");
    if (!p)
        return {parent, matrix(Json::array({0, 0, 0, 0, 0, 0}))};
    if (attribute(p, "rotation_format", "euler_rpy") != "euler_rpy")
        fail("Quaternion SDF poses are not supported");
    auto values = numbers(p->GetText() ? p->GetText() : "", 6, "0 0 0 0 0 0");
    if (lower(attribute(p, "degrees", "false")) == "true")
        for (int i = 3; i < 6; ++i)
            values[i] = values[i].get<double>() * std::acos(-1.) / 180.;
    std::string frame = attribute(p, "relative_to", content(p, "relative_to", parent));
    return {frame, matrix(values)};
}
std::string decodeUri(const std::string &value) {
    std::string result;
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '%' && i + 2 < value.size() &&
            std::isxdigit(static_cast<unsigned char>(value[i + 1])) &&
            std::isxdigit(static_cast<unsigned char>(value[i + 2]))) {
            result += char(std::stoi(value.substr(i + 1, 2), nullptr, 16));
            i += 2;
        } else
            result += value[i];
    }
    std::replace(result.begin(), result.end(), '\\', '/');
    text(result, "Resource URI");
    return result;
}
std::vector<std::string> words(const std::string &line, bool comments = false) {
    std::vector<std::string> result;
    std::string token;
    char quote = 0;
    bool escape = false, started = false;
    for (char c : line) {
        if (escape) {
            token += c;
            escape = false;
            started = true;
            continue;
        }
        if (c == '\\' && quote != '\'') {
            escape = true;
            continue;
        }
        if (quote) {
            if (c == quote)
                quote = 0;
            else
                token += c;
            started = true;
            continue;
        }
        if (c == '\'' || c == '"') {
            quote = c;
            started = true;
            continue;
        }
        if (comments && c == '#')
            break;
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (started) {
                result.push_back(token);
                token.clear();
                started = false;
            }
        } else {
            token += c;
            started = true;
        }
    }
    if (quote || escape)
        fail("Invalid quoted source material path");
    if (started)
        result.push_back(token);
    return result;
}
void runHelper(const fs::path &helper, const fs::path &input, const fs::path &output) {
    int pipes[2];
    if (pipe(pipes))
        fail("Cannot create mesh helper pipe");
    pid_t pid = fork();
    if (pid < 0) {
        close(pipes[0]);
        close(pipes[1]);
        fail("Cannot launch mesh helper");
    }
    if (pid == 0) {
        close(pipes[0]);
        dup2(pipes[1], STDOUT_FILENO);
        dup2(pipes[1], STDERR_FILENO);
        close(pipes[1]);
        execl(helper.c_str(), helper.c_str(), input.c_str(), output.c_str(),
              static_cast<char *>(nullptr));
        _exit(127);
    }
    close(pipes[1]);
    fcntl(pipes[0], F_SETFL, fcntl(pipes[0], F_GETFL) | O_NONBLOCK);
    std::string captured;
    int status = 0;
    bool exited = false, eof = false;
    const auto start = std::chrono::steady_clock::now();
    while (!exited || !eof) {
        pollfd descriptor{pipes[0], POLLIN | POLLHUP, 0};
        poll(&descriptor, 1, 100);
        std::array<char, 8192> buffer{};
        ssize_t amount;
        while ((amount = read(pipes[0], buffer.data(), buffer.size())) > 0) {
            captured.append(buffer.data(), std::size_t(amount));
            if (captured.size() > 2 * 1024 * 1024)
                captured.erase(0, captured.size() - 2 * 1024 * 1024);
        }
        if (amount == 0)
            eof = true;
        if (!exited) {
            pid_t done = waitpid(pid, &status, WNOHANG);
            if (done == pid)
                exited = true;
            else if (done < 0 && errno != EINTR) {
                close(pipes[0]);
                fail("Cannot wait for mesh helper");
            }
        }
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(120)) {
            if (!exited) {
                kill(pid, SIGKILL);
                while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
                }
            }
            close(pipes[0]);
            fail("Mesh import exceeded 120 seconds");
        }
    }
    close(pipes[0]);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        if (captured.size() > 2000)
            captured = captured.substr(captured.size() - 2000);
        fail("Mesh import failed: " + trim(captured));
    }
}
struct Importer {
    fs::path source, root, stage, helper;
    std::string modelName;
    std::map<fs::path, std::string> meshCache, textureCache;
    std::set<fs::path> textureDirectories;
    Importer(const fs::path &input, const fs::path &output, const fs::path &meshHelper)
        : source(fs::canonical(input)), root(source.parent_path()), stage(output),
          helper(meshHelper.empty() ? fs::path() : fs::absolute(meshHelper)) {
        textureDirectories.insert(root);
    }
    fs::path allowed(fs::path value, const fs::path &extra = {}) const {
        value = fs::absolute(value);
        for (auto parent = value; !parent.empty(); parent = parent.parent_path()) {
            if (fs::is_symlink(parent))
                fail("Source symlinks are not supported: " + value.string());
            if (parent == root.parent_path() || parent == parent.parent_path())
                break;
        }
        auto path = fs::weakly_canonical(value);
        if (!within(path, root) && (extra.empty() || !within(path, fs::weakly_canonical(extra))))
            fail("Resource escapes the source model folder: " + path.string());
        if (!fs::is_regular_file(path))
            fail("Missing source asset: " + path.string());
        if (fs::file_size(path) > maxFileBytes)
            fail("Source asset is too large");
        return path;
    }
    fs::path local(const std::string &raw) const {
        std::string uri = decodeUri(trim(raw));
        fs::path path;
        if (starts(uri, "model://")) {
            std::string rest = uri.substr(8);
            auto slash = rest.find('/');
            std::string model = rest.substr(0, slash);
            if (model != root.filename().string() && model != modelName)
                fail("External model resource is not bundled: " + uri);
            path = root / (slash == std::string::npos ? "" : rest.substr(slash + 1));
        } else if (starts(uri, "file://")) {
            path = uri.substr(7);
            if (!path.is_absolute())
                path = root / path;
        } else if (uri.find("://") != std::string::npos)
            fail("Remote resource is not supported; download it into the model folder: " + uri);
        else
            path = root / uri;
        return allowed(path);
    }
    std::string copySource(const fs::path &value) {
        auto path = allowed(value);
        fs::path relative = fs::path("assets/source") / path.lexically_relative(root),
                 target = stage / relative;
        fs::create_directories(target.parent_path());
        if (!fs::exists(target))
            fs::copy_file(path, target);
        return relative.generic_string();
    }
    void sidecars(const fs::path &path) {
        std::string extension = lower(path.extension().string());
        if (extension == ".obj") {
            std::stringstream lines(readText(path));
            std::string line;
            while (std::getline(lines, line)) {
                if (!starts(trim(line), "mtllib "))
                    continue;
                auto fields = words(trim(line));
                for (std::size_t i = 1; i < fields.size(); ++i) {
                    auto material = allowed(path.parent_path() / fields[i]);
                    copySource(material);
                    textureDirectories.insert(material.parent_path());
                    std::stringstream entries(readText(material));
                    std::string entry;
                    while (std::getline(entries, entry)) {
                        auto tokens = words(entry, true);
                        if (tokens.empty())
                            continue;
                        std::string type = lower(tokens.front());
                        if (starts(type, "map_") || type == "bump" || type == "disp" ||
                            type == "decal" || type == "norm")
                            copySource(allowed(material.parent_path() / tokens.back()));
                    }
                }
            }
        }
        if (extension == ".gltf") {
            auto data = readJson(path);
            for (const std::string fieldName : {"buffers", "images"})
                if (data.contains(fieldName))
                    for (auto &record : data[fieldName]) {
                        auto uri = record.value("uri", std::string());
                        if (!uri.empty() && !starts(uri, "data:")) {
                            auto target = allowed(path.parent_path() / decodeUri(uri));
                            copySource(target);
                            record["uri"] =
                                target.lexically_relative(path.parent_path()).generic_string();
                        }
                    }
            atomicJson(stage / copySource(path), data);
        }
        if (extension == ".dae") {
            auto doc = xml(readText(path, 16 * 1024 * 1024));
            walk(doc->RootElement(), [&](Element *image) {
                if (localName(image) != "image")
                    return;
                walk(image, [&](Element *ref) {
                    if (localName(ref) != "init_from" || !ref->GetText() ||
                        trim(ref->GetText()).empty())
                        return;
                    std::string raw = decodeUri(trim(ref->GetText()));
                    if (starts(raw, "file://"))
                        raw = raw.substr(7);
                    auto target = allowed(path.parent_path() / raw);
                    copySource(target);
                    ref->SetText(
                        target.lexically_relative(path.parent_path()).generic_string().c_str());
                });
            });
            writeText(stage / copySource(path), xmlString(doc.get()));
        }
    }
    fs::path textureSource(const std::string &value, const fs::path &meshPath) const {
        fs::path path = decodeUri(value);
        if (path.is_absolute())
            return path;
        auto first = meshPath.parent_path() / path;
        if (fs::is_regular_file(first))
            return first;
        for (const auto &directory : textureDirectories) {
            auto candidate = directory / path;
            if (fs::is_regular_file(candidate))
                return candidate;
        }
        return first;
    }
    std::string texture(const fs::path &value, const fs::path &extra) {
        auto path = allowed(value, extra);
        auto previous = textureCache.find(path);
        if (previous != textureCache.end())
            return previous->second;
        fs::path relative = fs::path("assets/textures") / (id() + lower(path.extension().string()));
        fs::create_directories((stage / relative).parent_path());
        fs::copy_file(path, stage / relative);
        if (within(path, root))
            copySource(path);
        return textureCache[path] = relative.generic_string();
    }
    std::string mesh(const fs::path &value) {
        auto path = allowed(value);
        auto previous = meshCache.find(path);
        if (previous != meshCache.end())
            return previous->second;
        if (helper.empty() || !fs::is_regular_file(helper))
            fail("Mesh import requires --mesh-helper or XOVIUM_MESH_IMPORT");
        copySource(path);
        sidecars(path);
        TempDir temp(stage, "mesh-");
        auto output = temp.path / "mesh.json";
        runHelper(helper, path, output);
        auto meshes = readJson(output);
        if (!meshes.is_array())
            fail("Mesh helper returned an invalid document");
        for (auto &part : meshes) {
            std::string resource = part.value("texture", std::string());
            if (!resource.empty()) {
                if (resource.front() == '*')
                    fail("Embedded texture was not extracted by the mesh helper");
                part["texture"] = texture(textureSource(resource, path), temp.path);
            }
            auto dependencies = part.value("dependencies", Json::array());
            if (!dependencies.is_array())
                fail("Mesh dependencies must be a list");
            if (!dependencies.empty()) {
                Json converted = Json::array();
                for (const auto &dep : dependencies)
                    converted.push_back(
                        texture(textureSource(text(dep, "Texture dependency"), path), temp.path));
                part["dependencies"] = converted;
            }
        }
        validateMesh(meshes, stage);
        std::string relative = "assets/meshes/" + id() + ".json";
        fs::create_directories((stage / relative).parent_path());
        atomicJson(stage / relative, meshes);
        return meshCache[path] = relative;
    }
    void retainedResources(Element *tree) {
        walk(tree, [&](Element *element) {
            if (!resourceTags.count(element->Name()) || !element->GetText())
                return;
            std::string value = trim(element->GetText());
            if (value.empty() || starts(value, "__default__") || starts(value, "name://"))
                return;
            auto path = local(value);
            std::string relative = copySource(path);
            element->SetText(relative.substr(7).c_str());
            auto extension = lower(path.extension().string());
            if (extension == ".obj" || extension == ".dae" || extension == ".gltf" ||
                extension == ".glb" || extension == ".fbx" || extension == ".stl")
                mesh(path);
        });
        for (const auto &entry : fs::directory_iterator(root)) {
            std::string name = lower(entry.path().filename().string());
            if (entry.is_regular_file() &&
                (starts(name, "license") || starts(name, "copying") || starts(name, "notice")))
                copySource(entry.path());
        }
    }
};
Json sensor(Element *element, const std::string &parent, const Json &placement) {
    const std::map<std::string, std::string> aliases = {
        {"air_speed", "airspeed"}, {"navsat", "gps"},    {"air_pressure", "barometer"},
        {"gpu_lidar", "lidar"},    {"gpu_ray", "lidar"}, {"ray", "lidar"},
        {"depth", "depth_camera"}};
    std::string kind = attribute(element, "type");
    if (aliases.count(kind))
        kind = aliases.at(kind);
    Json spec;
    for (const auto &candidate : sensorCatalog().at("types"))
        if (candidate.at("id") == kind) {
            spec = candidate;
            break;
        }
    if (spec.is_null())
        return Json();
    Json parameters = spec.at("default_parameters");
    const std::map<std::string, std::map<std::string, std::string>> mappings = {
        {"camera",
         {{"width", "camera/image/width"},
          {"height", "camera/image/height"},
          {"horizontal_fov", "camera/horizontal_fov"},
          {"near", "camera/clip/near"},
          {"far", "camera/clip/far"}}},
        {"lidar",
         {{"horizontal_samples", "lidar/scan/horizontal/samples"},
          {"horizontal_min_angle", "lidar/scan/horizontal/min_angle"},
          {"horizontal_max_angle", "lidar/scan/horizontal/max_angle"},
          {"vertical_samples", "lidar/scan/vertical/samples"},
          {"vertical_min_angle", "lidar/scan/vertical/min_angle"},
          {"vertical_max_angle", "lidar/scan/vertical/max_angle"},
          {"min_range", "lidar/range/min"},
          {"max_range", "lidar/range/max"}}},
        {"barometer", {{"pressure_noise", "air_pressure/pressure/noise/stddev"}}},
        {"airspeed", {{"noise", "air_speed/pressure/noise/stddev"}}},
        {"imu",
         {{"accel_noise", "imu/linear_acceleration/x/noise/stddev"},
          {"gyro_noise", "imu/angular_velocity/x/noise/stddev"}}},
        {"magnetometer", {{"noise", "magnetometer/x/noise/stddev"}}},
        {"gps",
         {{"position_noise", "navsat/position_sensing/horizontal/noise/stddev"},
          {"velocity_noise", "navsat/velocity_sensing/horizontal/noise/stddev"}}},
        {"contact", {{"collision", "contact/collision"}}}};
    const auto &fields = mappings.at(kind == "depth_camera" ? "camera" : kind);
    for (const auto &[key, path] : fields) {
        auto *source = child(element, path);
        if (!source && starts(path, "lidar/"))
            source = child(element, "ray/" + path.substr(6));
        if (!source || !source->GetText())
            continue;
        std::string value = source->GetText();
        if (parameters[key].is_string())
            parameters[key] = value;
        else if (parameters[key].is_number_integer()) {
            std::size_t used = 0;
            auto clean = trim(value);
            long long n = std::stoll(clean, &used);
            if (used != clean.size())
                fail("Invalid integer SDF sensor parameter: " + value);
            parameters[key] = n;
        } else
            parameters[key] = scalar(value);
    }
    double rate = scalar(content(element, "update_rate", spec.at("default_rate_hz").dump()));
    if (rate <= 0)
        rate = spec.at("default_rate_hz").get<double>();
    std::string name = attribute(element, "name", kind);
    return {{"id", id()},
            {"name", name},
            {"source_name", name},
            {"source_parent", parent},
            {"type", kind},
            {"parent", parent},
            {"pose", placement},
            {"enabled", true},
            {"rate_hz", rate},
            {"source_rate_hz", rate},
            {"topic", content(element, "topic")},
            {"parameters", parameters},
            {"source_parameters", parameters},
            {"xml", xmlString(element)}};
}
Json importSdf(const fs::path &source, const fs::path &stage, const fs::path &helper) {
    Importer importer(source, stage, helper);
    auto tree = xml(readText(source, 16 * 1024 * 1024));
    if (std::string(tree->RootElement()->Name()) == "model") {
        auto *previous = tree->RootElement();
        auto *model = previous->DeepClone(tree.get());
        tree->DeleteChild(previous);
        auto *sdf = tree->NewElement("sdf");
        sdf->SetAttribute("version", "1.11");
        tree->InsertEndChild(sdf);
        sdf->InsertEndChild(model);
    }
    auto models = children(tree->RootElement(), "model");
    if (models.size() != 1)
        fail("Import requires exactly one top-level SDF model");
    auto *model = models.front();
    importer.modelName = attribute(model, "name", "Imported model");
    if (model->FirstChildElement("include") || model->FirstChildElement("model"))
        fail("Nested SDF models/includes are not supported; import a self-contained model");
    auto lowerName = lower(importer.modelName);
    std::string vehicle = lowerName.find("rover") != std::string::npos ? "rover"
                          : (lowerName.find("cessna") != std::string::npos ||
                             lowerName.find("plane") != std::string::npos ||
                             lowerName.find("aircraft") != std::string::npos)
                              ? "fixed-wing"
                              : "custom";
    Json document = newProject(importer.modelName, vehicle);
    for (const std::string kind : {"links", "visuals", "sensors", "plugins", "joints"})
        document[kind] = Json::array();
    std::vector<std::tuple<std::string, std::size_t, Element *>> preserved;
    std::map<std::string, std::pair<std::string, Matrix>> nodes;
    std::map<std::string, Matrix> resolved{{"__model__", matrix(Json::array({0, 0, 0, 0, 0, 0}))}};
    std::set<std::string> visiting;
    auto add = [&](const std::string &name, const std::pair<std::string, Matrix> &value) {
        if (name.empty() || name == "__model__" || !nodes.emplace(name, value).second)
            fail("Duplicate/invalid SDF frame name: " + name);
    };
    for (auto *link : children(model, "link"))
        add(attribute(link, "name"), sdfPose(link, "__model__"));
    for (auto *joint : children(model, "joint"))
        add(attribute(joint, "name"), sdfPose(joint, content(joint, "child", "__model__")));
    for (auto *frame : children(model, "frame"))
        add(attribute(frame, "name"), sdfPose(frame, attribute(frame, "attached_to", "__model__")));
    std::function<Matrix(std::string)> resolve = [&](std::string name) -> Matrix {
        if (name == importer.modelName && !nodes.count(name))
            name = "__model__";
        if (resolved.count(name))
            return resolved.at(name);
        if (!nodes.count(name))
            fail("Unknown SDF pose frame: " + name);
        if (!visiting.insert(name).second)
            fail("Cyclic SDF frame reference: " + name);
        auto node = nodes.at(name);
        auto result = multiply(resolve(node.first), node.second);
        visiting.erase(name);
        return resolved[name] = result;
    };
    auto localPose = [&](Element *element, const std::string &parent) {
        auto local = sdfPose(element, parent);
        return pose(
            multiply(inverse(resolve(parent)), multiply(resolve(local.first), local.second)));
    };
    for (const auto &node : nodes)
        resolve(node.first);
    for (auto *link : children(model, "link")) {
        std::string name = attribute(link, "name");
        Json inertia = Json::array();
        for (const std::string key : {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"})
            inertia.push_back(
                scalar(content(link, "inertial/inertia/" + key,
                               (key == "ixx" || key == "iyy" || key == "izz") ? "1" : "0")));
        Json record = {{"name", name},
                       {"pose", pose(resolve(name))},
                       {"mass", scalar(content(link, "inertial/mass", "1"))},
                       {"inertia", inertia}};
        if (auto *inertial = link->FirstChildElement("inertial"))
            record["inertial_pose"] = localPose(inertial, name);
        else
            document["warnings"].push_back(
                "Link " + name +
                " has no inertial block; editor defaults to 1 kg and unit inertia.");
        document["links"].push_back(record);
        for (auto *visual : children(link, "visual")) {
            auto *geometry = visual->FirstChildElement("geometry");
            if (!geometry)
                fail("Visual has no geometry: " + attribute(visual, "name"));
            Json shape, scale = Json::array({1, 1, 1});
            if (auto *mesh = geometry->FirstChildElement("mesh")) {
                if (mesh->FirstChildElement("submesh"))
                    fail("SDF mesh sub-selection is not supported");
                shape = {{"type", "mesh"},
                         {"path", importer.mesh(importer.local(content(mesh, "uri")))}};
                scale = numbers(content(mesh, "scale"), 3, "1 1 1");
            } else if (geometry->FirstChildElement("box"))
                shape = {{"type", "box"}, {"size", numbers(content(geometry, "box/size"), 3)}};
            else if (geometry->FirstChildElement("sphere"))
                shape = {{"type", "sphere"},
                         {"radius", scalar(content(geometry, "sphere/radius"))}};
            else if (geometry->FirstChildElement("cylinder"))
                shape = {{"type", "cylinder"},
                         {"radius", scalar(content(geometry, "cylinder/radius"))},
                         {"length", scalar(content(geometry, "cylinder/length"))}};
            else
                fail("Unsupported SDF visual geometry: " + xmlString(geometry).substr(0, 150));
            std::string visualName =
                attribute(visual, "name", "visual_" + std::to_string(document["visuals"].size()));
            document["visuals"].push_back(
                {{"id", id()},
                 {"name", visualName},
                 {"source_name", visualName},
                 {"source_link", name},
                 {"link", name},
                 {"pose", localPose(visual, name)},
                 {"scale", scale},
                 {"geometry", shape},
                 {"color", color(content(visual, "material/diffuse",
                                         shape["type"] == "mesh" ? "1 1 1 1" : "0.7 0.7 0.7 1"))}});
        }
        for (auto *element : children(link, "sensor")) {
            auto value = sensor(element, name, localPose(element, name));
            if (value.is_null())
                document["warnings"].push_back("Unsupported sensor " + attribute(element, "type") +
                                               " retained in source SDF only.");
            else {
                document["sensors"].push_back(value);
                preserved.emplace_back("sensors", document["sensors"].size() - 1, element);
            }
        }
    }
    std::vector<std::pair<std::string, Element *>> owners{{"__model__", model}};
    for (auto *link : children(model, "link"))
        owners.emplace_back(attribute(link, "name"), link);
    for (auto [parent, owner] : owners)
        for (auto *element : children(owner, "plugin")) {
            document["plugins"].push_back(
                {{"id", id()},
                 {"name", attribute(element, "name",
                                    "plugin_" + std::to_string(document["plugins"].size()))},
                 {"target", "gazebo"},
                 {"parent", parent},
                 {"filename", attribute(element, "filename")},
                 {"xml", xmlString(element)}});
            preserved.emplace_back("plugins", document["plugins"].size() - 1, element);
        }
    for (auto *joint : children(model, "joint")) {
        document["joints"].push_back({{"name", attribute(joint, "name")},
                                      {"type", attribute(joint, "type", "fixed")},
                                      {"parent", content(joint, "parent")},
                                      {"child", content(joint, "child")},
                                      {"pose", pose(resolve(attribute(joint, "name")))},
                                      {"xml", xmlString(joint)}});
        preserved.emplace_back("joints", document["joints"].size() - 1, joint);
    }
    importer.retainedResources(tree->RootElement());
    for (auto [kind, index, element] : preserved)
        document[kind][index]["xml"] = xmlString(element);
    fs::create_directories(stage / "assets");
    writeText(stage / "assets/source.sdf", xmlString(tree.get()));
    document["source_sdf"] = "assets/source.sdf";
    validateProject(document, stage);
    return document;
}
} // namespace
Json importProject(const fs::path &sourcePath, const fs::path &workspacePath,
                   const fs::path &meshHelper) {
    auto source = fs::absolute(sourcePath), workspace = fs::absolute(workspacePath);
    if (fs::is_symlink(source) || !fs::is_regular_file(source))
        fail("Import source must be a regular file");
    if (fs::is_symlink(workspace))
        fail("Workspace must not be a symlink");
    if (fs::exists(workspace)) {
        for (const auto &entry : fs::directory_iterator(workspace))
            if (entry.path().filename() != "project.json" && entry.path().filename() != "assets")
                fail("Import workspace contains unrelated files");
        if (!fs::is_empty(workspace) && !fs::is_regular_file(workspace / "project.json"))
            fail("Import workspace must be empty or an existing model project");
    }
    fs::path helper = meshHelper;
    if (helper.empty()) {
        if (const char *env = std::getenv("XOVIUM_MESH_IMPORT"))
            helper = env;
    }
    TempDir temp(workspace.parent_path(), ".xovmodel-import-");
    auto stage = temp.path / "workspace";
    fs::create_directories(stage / "assets");
    auto extension = lower(source.extension().string());
    Json document;
    if (extension == ".xovmodel") {
        unpack(source, stage);
        document = loadWorkspace(stage);
    } else if (extension == ".sdf")
        document = importSdf(source, stage, helper);
    else if (extension == ".obj" || extension == ".dae" || extension == ".stl" ||
             extension == ".fbx" || extension == ".gltf" || extension == ".glb") {
        Importer importer(source, stage, helper);
        document = newProject(source.stem().string(), "custom");
        document["visuals"][0]["name"] = source.stem().string();
        document["visuals"][0]["geometry"] = {{"type", "mesh"}, {"path", importer.mesh(source)}};
        document["visuals"][0]["color"] = Json::array({1, 1, 1, 1});
        document["warnings"].push_back(
            "Mesh import supplies visual geometry; configure mass, inertia, "
            "collision and sensors before simulation.");
    } else
        fail("Supported imports: .xovmodel, .sdf, .obj, .dae, .stl, .fbx, .gltf, .glb");
    writeWorkspace(document, stage);
    safeFiles(stage);
    auto backup = temp.path / "previous";
    if (fs::exists(workspace))
        fs::rename(workspace, backup);
    try {
        fs::rename(stage, workspace);
    } catch (...) {
        if (fs::exists(backup))
            fs::rename(backup, workspace);
        throw;
    }
    return document;
}
} // namespace xovium
