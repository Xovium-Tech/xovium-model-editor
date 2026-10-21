#include "project.hpp"
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#ifndef XOVIUM_SOURCE_DIR
#define XOVIUM_SOURCE_DIR "."
#endif
#ifndef XOVIUM_VERSION
#define XOVIUM_VERSION "0.1.0"
#endif
#ifndef XOVIUM_MESH_IMPORT_DEFAULT
#define XOVIUM_MESH_IMPORT_DEFAULT "mesh-import"
#endif
namespace {
const char *usage =
    "Xovium native model tool\n  [--root DIR] new --workspace DIR [--name NAME] [--vehicle "
    "fixed-wing|rover|custom]\n  [--root DIR] import FILE --workspace DIR [--mesh-helper "
    "EXE]\n  [--root DIR] save --workspace DIR --output FILE.xovmodel\n  [--root DIR] export "
    "--workspace DIR --target gazebo|isaac|sih|unreal|flightgear --output EMPTYDIR\n  "
    "[--root DIR] validate --workspace DIR\n  [--root DIR] catalog\n  --version\n";
[[noreturn]] void fail(const std::string &value) {
    throw std::runtime_error(value);
}
} // namespace
int main(int argc, char **argv) {
    using namespace xovium;
    try {
        std::vector<std::string> args;
        fs::path root = XOVIUM_SOURCE_DIR;
        bool rootGiven = false;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--version") {
                std::cout << Json({{"ok", true}, {"version", XOVIUM_VERSION}}).dump() << '\n';
                return 0;
            }
            if (arg == "--help" || arg == "-h") {
                std::cout << usage;
                return 0;
            }
            if (arg == "--root") {
                if (++i >= argc)
                    fail("--root requires a directory");
                root = argv[i];
                rootGiven = true;
            } else
                args.push_back(arg);
        }
        std::error_code error;
        auto executable = fs::read_symlink("/proc/self/exe", error);
        if (!rootGiven && !fs::is_regular_file(root / "backend/sensors.json") && !error) {
            auto installed = executable.parent_path().parent_path() / "share/xovium-model-editor";
            if (fs::is_regular_file(installed / "backend/sensors.json"))
                root = installed;
        }
        setRoot(root);
        if (args.empty())
            fail("A command is required; use --help");
        std::string command = args.front();
        const std::map<std::string, std::set<std::string>> allowed = {
            {"new", {"workspace", "name", "vehicle"}},
            {"import", {"workspace", "mesh-helper"}},
            {"save", {"workspace", "output"}},
            {"export", {"workspace", "target", "output"}},
            {"validate", {"workspace"}},
            {"catalog", {}}};
        if (!allowed.count(command))
            fail("Unknown command: " + command);
        std::map<std::string, std::string> options;
        std::vector<std::string> positional;
        bool literal = false;
        for (std::size_t i = 1; i < args.size(); ++i) {
            const auto &arg = args[i];
            if (arg == "--" && !literal) {
                literal = true;
                continue;
            }
            if (!literal && arg.rfind("--", 0) == 0) {
                auto equal = arg.find('=');
                std::string key =
                    arg.substr(2, equal == std::string::npos ? std::string::npos : equal - 2);
                if (!allowed.at(command).count(key))
                    fail("Unknown option: --" + key);
                std::string value;
                if (equal != std::string::npos)
                    value = arg.substr(equal + 1);
                else {
                    if (i + 1 >= args.size() || args[i + 1].rfind("--", 0) == 0)
                        fail("Missing value for --" + key);
                    value = args[++i];
                }
                options[key] = value;
            } else
                positional.push_back(arg);
        }
        auto required = [&](const std::string &name) {
            if (!options.count(name) || options.at(name).empty())
                fail("--" + name + " is required");
            return options.at(name);
        };
        if ((command == "import" && positional.size() != 1) ||
            (command != "import" && !positional.empty()))
            fail(command == "import" ? "Import requires exactly one source file"
                                     : "Unexpected positional argument");
        Json result = {{"ok", true}};
        if (command == "catalog")
            result["catalog"] = sensorCatalog();
        else {
            auto workspace = fs::absolute(required("workspace"));
            result["workspace"] = workspace.string();
            Json document;
            if (command == "new") {
                if (fs::exists(workspace) && !fs::is_empty(workspace))
                    fail("New project workspace must be empty");
                document = newProject(options.count("name") ? options["name"] : "Untitled model",
                                      options.count("vehicle") ? options["vehicle"] : "fixed-wing");
                writeWorkspace(document, workspace);
            } else if (command == "import") {
                fs::path helper;
                if (options.count("mesh-helper"))
                    helper = options["mesh-helper"];
                else if (const char *env = std::getenv("XOVIUM_MESH_IMPORT"))
                    helper = env;
                else if (!error && fs::is_regular_file(executable.parent_path() / "mesh-import"))
                    helper = executable.parent_path() / "mesh-import";
                else
                    helper = XOVIUM_MESH_IMPORT_DEFAULT;
                document = importProject(positional.front(), workspace, helper);
            } else if (command == "save") {
                auto output = saveProject(workspace, required("output"));
                document = loadWorkspace(workspace);
                result["output"] = output.string();
            } else if (command == "export") {
                auto target = required("target");
                if (!std::set<std::string>{"gazebo", "isaac", "sih", "unreal", "flightgear"}.count(
                        target))
                    fail("Unknown export target: " + target);
                auto output = fs::absolute(required("output"));
                document = loadWorkspace(workspace);
                result["target"] = target;
                result["output"] = output.string();
                result["export"] = exportProject(document, workspace, target, output);
            } else
                document = loadWorkspace(workspace);
            result["name"] = document.at("name");
            result["warnings"] = document.value("warnings", Json::array());
            for (const char *key : {"links", "visuals", "sensors", "plugins", "joints"})
                result["counts"][key] = document.value(key, Json::array()).size();
        }
        std::cout << result.dump() << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cout << xovium::Json({{"ok", false}, {"error", error.what()}}).dump() << '\n';
        return 1;
    }
}
