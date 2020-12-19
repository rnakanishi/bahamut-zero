#include <tiny_obj_loader.h>
#include <garuda_utils/obj_loader.hpp>
#include <iostream>

namespace Garuda {
void ObjLoader::loadFile(std::string filename, TriangleMesh& object) {
  // Check if the extension is obj

  // Get object position, normals, and uvs vector

  std::string warning, error;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::attrib_t attributes;

  std::vector<Eigen::Array3f> vertCoords;
  std::vector<Eigen::Array3f> vertNormals;

  std::string baseDir(filename);
  int lastSlash = baseDir.find_last_of("/\\");

  // Loaading data
  bool ret = tinyobj::LoadObj(
      &attributes, &shapes, &materials, &warning, &error, filename.c_str(),
      baseDir.substr(0, lastSlash).c_str(), true, false);

  bool hasTexture, hasMaterial, hasNormal;

  hasTexture = (!attributes.texcoords.empty()) ? true : false;
  hasMaterial = (!materials.empty()) ? true : false;
  hasNormal = (!attributes.normals.empty()) ? true : false;

  if (hasTexture && !materials.empty()) {
    // Set object texture filepath
    object.getTexture().setFilename(
        baseDir.substr(0, lastSlash + 1).append(materials[0].diffuse_texname));
  } else {
    hasTexture = false;
  }
  if (hasTexture) {
    vertCoords.resize(attributes.texcoords.size() / 2);
    vertNormals.resize(attributes.texcoords.size() / 2);
  } else {
    vertCoords.resize(attributes.vertices.size() / 3);
    vertNormals.resize(attributes.vertices.size() / 3);
  }

  std::cerr << "Vertices size: " << attributes.vertices.size() / 3
            << "\nTexcoords size " << attributes.texcoords.size() / 2
            << "\nNormals size: " << attributes.normals.size() / 3 << std::endl;
  std::vector<Eigen::ArrayXf> texCoords;
  for (int i = 0; i < attributes.texcoords.size(); i++) {
    texCoords.emplace_back(Eigen::Array2f(attributes.texcoords[2 * i + 0],
                                          attributes.texcoords[2 * i + 1]));
  }
  object.addVerticeProperty("textureCoordinates", texCoords);
}
}  // namespace Garuda
