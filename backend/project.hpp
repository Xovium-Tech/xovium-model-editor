#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

namespace xovium {
namespace fs = std::filesystem;
using Json = nlohmann::json;
void setRoot(const fs::path &root);
const Json &sensorCatalog();
fs::path assetPath(const std::string &value, const fs::path &workspace = {}, bool mustExist = true);
void validateProject(const Json &document, const fs::path &workspace = {});
Json loadWorkspace(const fs::path &workspace);
Json newProject(const std::string &name, const std::string &vehicle);
void writeWorkspace(const Json &document, const fs::path &workspace);
Json importProject(const fs::path &source, const fs::path &workspace, const fs::path &helper = {});
fs::path saveProject(const fs::path &workspace, const fs::path &output);
Json exportProject(const Json &document, const fs::path &workspace, const std::string &target,
                   const fs::path &output);
} // namespace xovium
