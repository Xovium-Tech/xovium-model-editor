// Assimp normalizes mesh transforms and COLLADA units before document poses.
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <nlohmann/json.hpp>
#include <png.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Json = nlohmann::json;

static std::string compressedExtension(const aiTexture& texture) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(texture.pcData);
    if (texture.mWidth >= 8 && png_sig_cmp(bytes, 0, 8) == 0) return ".png";
    if (texture.mWidth >= 3 && bytes[0] == 0xff && bytes[1] == 0xd8 && bytes[2] == 0xff) return ".jpg";
    std::string hint(texture.achFormatHint, strnlen(texture.achFormatHint, sizeof(texture.achFormatHint)));
    std::transform(hint.begin(), hint.end(), hint.begin(), [](unsigned char c) { return std::tolower(c); });
    if (hint == "jpeg") hint = "jpg";
    for (const auto* supported : {"png", "jpg", "bmp", "tga", "dds", "webp", "gif"})
        if (hint == supported) return "." + hint;
    throw std::runtime_error("Unsupported embedded texture encoding; supply PNG or JPEG textures");
}

static std::string extractTexture(const aiTexture& texture, unsigned index, const fs::path& output) {
    constexpr std::size_t maximumBytes = 256u * 1024u * 1024u;
    if (!texture.pcData || texture.mWidth == 0) throw std::runtime_error("Embedded texture is empty");
    const auto directory = output.parent_path() / (output.stem().string() + "_textures");
    fs::create_directories(directory);
    const auto target = directory / ("embedded_" + std::to_string(index) +
        (texture.mHeight == 0 ? compressedExtension(texture) : ".png"));
    const auto temporary = fs::path(target.string() + ".part");
    if (texture.mHeight == 0) {
        if (texture.mWidth > maximumBytes) throw std::runtime_error("Embedded image exceeds 256 MiB");
        std::ofstream stream(temporary, std::ios::binary);
        stream.write(reinterpret_cast<const char*>(texture.pcData), texture.mWidth);
        stream.close();
        if (!stream) throw std::runtime_error("Cannot write embedded image: " + temporary.string());
    } else {
        if (texture.mWidth > 16384 || texture.mHeight > 16384 ||
            std::size_t(texture.mWidth) * texture.mHeight > maximumBytes / 4)
            throw std::runtime_error("Embedded texture dimensions exceed the import limit");
        std::vector<unsigned char> pixels(std::size_t(texture.mWidth) * texture.mHeight * 4);
        for (std::size_t i = 0; i < pixels.size() / 4; ++i) {
            pixels[i * 4] = texture.pcData[i].r;
            pixels[i * 4 + 1] = texture.pcData[i].g;
            pixels[i * 4 + 2] = texture.pcData[i].b;
            pixels[i * 4 + 3] = texture.pcData[i].a;
        }
        png_image image{};
        image.version = PNG_IMAGE_VERSION;
        image.width = texture.mWidth;
        image.height = texture.mHeight;
        image.format = PNG_FORMAT_RGBA;
        if (!png_image_write_to_file(&image, temporary.c_str(), 0, pixels.data(), 0, nullptr))
            throw std::runtime_error("Cannot encode embedded PNG: " + std::string(image.message));
    }
    fs::rename(temporary, target);
    return fs::absolute(target).lexically_normal().string();
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: mesh-import INPUT_MESH OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto input = fs::absolute(argv[1]);
        const auto output = fs::absolute(argv[2]);
        if (input == output) throw std::runtime_error("Output JSON must differ from the input mesh");
        Assimp::Importer importer;
        // Match SDF mesh conventions: keep the authored axes, including Z-up DAE.
        importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
        const auto* scene = importer.ReadFile(input.string(),
            aiProcess_Triangulate | aiProcess_PreTransformVertices | aiProcess_JoinIdenticalVertices |
            aiProcess_SortByPType | aiProcess_ValidateDataStructure);
        if (!scene) throw std::runtime_error(importer.GetErrorString());
        Json result = Json::array();
        std::map<const aiTexture*, std::string> textures;
        // Retain every material image, even though the viewport uses base color only.
        auto textureReference = [&](const aiString& reference) {
            if (const auto* embedded = scene->GetEmbeddedTexture(reference.C_Str())) {
                auto found = textures.find(embedded);
                if (found == textures.end())
                    found = textures.emplace(embedded, extractTexture(*embedded, textures.size(), output)).first;
                return found->second;
            }
            std::string path = reference.C_Str();
            std::replace(path.begin(), path.end(), '\\', '/');
            if (!path.empty() && path.front() == '*')
                throw std::runtime_error("Missing embedded texture: " + path);
            return path;
        };
        std::size_t vertexCount = 0, triangleCount = 0;
        for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
            const auto* mesh = scene->mMeshes[m];
            Json vertices = Json::array(), faces = Json::array(), uv = Json::array();
            for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
                const auto& v = mesh->mVertices[i];
                if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
                    throw std::runtime_error("Mesh contains non-finite coordinates");
                vertices.push_back({v.x, v.y, v.z});
                if (mesh->HasTextureCoords(0)) {
                    const auto& t = mesh->mTextureCoords[0][i];
                    if (!std::isfinite(t.x) || !std::isfinite(t.y))
                        throw std::runtime_error("Mesh contains non-finite texture coordinates");
                    uv.push_back({t.x, t.y});
                }
            }
            for (unsigned i = 0; i < mesh->mNumFaces; ++i) {
                const auto& face = mesh->mFaces[i];
                if (face.mNumIndices != 3) continue;
                for (unsigned j = 0; j < 3; ++j)
                    if (face.mIndices[j] >= mesh->mNumVertices) throw std::runtime_error("Invalid mesh index");
                faces.push_back({face.mIndices[0], face.mIndices[1], face.mIndices[2]});
            }
            if (vertices.empty() || faces.empty()) continue;
            aiColor4D color(.7f, .7f, .7f, 1.f);
            std::string texturePath;
            std::vector<std::string> dependencies;
            if (mesh->mMaterialIndex < scene->mNumMaterials) {
                const auto* material = scene->mMaterials[mesh->mMaterialIndex];
                if (material->Get(AI_MATKEY_BASE_COLOR, color) != AI_SUCCESS)
                    material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
                aiString texture;
                if (material->GetTexture(aiTextureType_BASE_COLOR, 0, &texture) != AI_SUCCESS)
                    material->GetTexture(aiTextureType_DIFFUSE, 0, &texture);
                if (texture.length) texturePath = textureReference(texture);
                // Read texture-file properties rather than a fixed texture type range;
                // this also preserves future Assimp material slots without rendering them.
                for (unsigned i = 0; i < material->mNumProperties; ++i) {
                    const auto* property = material->mProperties[i];
                    if (std::string(property->mKey.C_Str()) != _AI_MATKEY_TEXTURE_BASE) continue;
                    aiString reference;
                    if (material->GetTexture(static_cast<aiTextureType>(property->mSemantic),
                            property->mIndex, &reference) != AI_SUCCESS || !reference.length) continue;
                    const auto dependency = textureReference(reference);
                    if (std::find(dependencies.begin(), dependencies.end(), dependency) == dependencies.end())
                        dependencies.push_back(dependency);
                }
            }
            vertexCount += vertices.size();
            triangleCount += faces.size();
            result.push_back({{"vertices", std::move(vertices)}, {"faces", std::move(faces)},
                {"uv", std::move(uv)}, {"color", {color.r, color.g, color.b, color.a}}, {"texture", texturePath},
                {"dependencies", std::move(dependencies)}});
        }
        if (result.empty()) throw std::runtime_error("Model contains no triangle meshes");
        fs::create_directories(output.parent_path());
        const auto temporary = fs::path(output.string() + ".part");
        std::ofstream stream(temporary);
        stream << result.dump() << '\n';
        stream.close();
        if (!stream) throw std::runtime_error("Cannot write mesh JSON: " + output.string());
        fs::rename(temporary, output);
        std::cerr << "Imported " << result.size() << " meshes, " << vertexCount << " vertices, "
                  << triangleCount << " triangles, " << textures.size() << " embedded textures\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Mesh import failed: " << error.what() << '\n';
        return 1;
    }
}
