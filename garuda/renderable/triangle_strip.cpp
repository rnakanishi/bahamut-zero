#include <glad/glad.h>

#include <renderable/triangle_strip.hpp>

namespace Garuda {

TriangleStrip::TriangleStrip() : TriangleStrip(std::vector<Eigen::Array3f>()) {}

TriangleStrip::TriangleStrip(std::vector<Eigen::Array3f> positions)
    : RenderObject() {
  vertexVectorLabels["positions"] = 0;
  vertexVectorPropeties.emplace_back(std::vector<float>());
  vectorPropertyDimension.emplace_back(3);
  for (auto&& data : positions) {
    vertexVectorPropeties[0].insert(vertexVectorPropeties[0].end(), data.data(),
                                    data.data() + data.size());
  }
}

TriangleStrip::~TriangleStrip() {}

int TriangleStrip::addVerticeProperty(std::string name,
                                      std::vector<Eigen::Array4f> dataVector) {
  unsigned int propertyId;
  // Check for existing property name
  if (vertexVectorLabels.find(name) == vertexVectorLabels.end()) {
    propertyId = vertexVectorPropeties.size();
    vertexVectorLabels[name] = propertyId;
    vertexVectorPropeties.emplace_back(std::vector<float>());
    vectorPropertyDimension.emplace_back(0);
  }
  propertyId = vertexVectorLabels[name];
  vertexVectorPropeties[propertyId].clear();
  // Linearize Eigen Array data
  for (auto&& data : dataVector) {
    vertexVectorPropeties[propertyId].insert(
        vertexVectorPropeties[propertyId].end(), data.data(),
        data.data() + data.size());
  }
  vectorPropertyDimension[propertyId] = 4;
  return propertyId;
}

int TriangleStrip::addVerticeProperty(std::string name,
                                      std::vector<Eigen::Array3f> dataVector) {
  unsigned int propertyId;
  // Check for existing property name
  if (vertexVectorLabels.find(name) == vertexVectorLabels.end()) {
    propertyId = vertexVectorPropeties.size();
    vertexVectorLabels[name] = propertyId;
    vertexVectorPropeties.emplace_back(std::vector<float>());
    vectorPropertyDimension.emplace_back(0);
  }
  propertyId = vertexVectorLabels[name];
  vertexVectorPropeties[propertyId].clear();
  // Linearize Eigen Array data
  for (auto&& data : dataVector) {
    vertexVectorPropeties[propertyId].insert(
        vertexVectorPropeties[propertyId].end(), data.data(),
        data.data() + data.size());
  }
  vectorPropertyDimension[propertyId] = 3;
  return propertyId;
}

int TriangleStrip::addVerticeProperty(std::string name,
                                      std::vector<Eigen::Array2f> dataVector) {
  unsigned int propertyId;
  // Check for existing property name
  if (vertexVectorLabels.find(name) == vertexVectorLabels.end()) {
    propertyId = vertexVectorPropeties.size();
    vertexVectorLabels[name] = propertyId;
    vertexVectorPropeties.emplace_back(std::vector<float>());
    vectorPropertyDimension.emplace_back(0);
  }
  propertyId = vertexVectorLabels[name];
  vertexVectorPropeties[propertyId].clear();
  // Linearize Eigen Array data
  for (auto&& data : dataVector) {
    vertexVectorPropeties[propertyId].insert(
        vertexVectorPropeties[propertyId].end(), data.data(),
        data.data() + data.size());
  }
  vectorPropertyDimension[propertyId] = 2;
  return propertyId;
}

int TriangleStrip::addFaceProperty(std::string name,
                                   std::vector<Eigen::Array3i> dataVector) {
  unsigned int propertyId;
  // Check for existing property name
  if (faceScalarLabels.find(name) == faceScalarLabels.end()) {
    propertyId = faceIntProperties.size();
    faceScalarLabels[name] = propertyId;
    faceIntProperties.emplace_back(std::vector<int>());
  }
  propertyId = faceScalarLabels[name];
  faceIntProperties[propertyId].clear();
  // Linearize Eigen Array data
  for (auto&& data : dataVector) {
    faceIntProperties[propertyId].insert(faceIntProperties[propertyId].end(),
                                         data.data(),
                                         data.data() + data.size());
  }
  return propertyId;
}

int TriangleStrip::addFaceProperty(std::string name,
                                   std::vector<Eigen::Array3f> dataVector) {
  return -1;
}

int TriangleStrip::addFaceProperty(std::string name,
                                   std::vector<Eigen::Array4f> dataVector) {
  return -1;
}

void TriangleStrip::sendDataToBuffer() {
  glBindVertexArray(_vao);
  glBindBuffer(GL_ARRAY_BUFFER, _vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertexVectorPropeties[0].size(),
               &vertexVectorPropeties[0][0], GL_STATIC_DRAW);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               sizeof(int) * faceIntProperties[0].size(),
               &faceIntProperties[0][0], GL_STATIC_DRAW);
  /**
   * glVErtexAttribPointer parameters:
   *   - Which vertex attrib to configure set in the shader program
   *   - Size of vertex attrib
   *   - Data type
   *   - Is data normalized?
   *   - Stride: space between consecutive vertex attrib
   *   - Offset to the beginning position data
   */
  // TODO: Fix size of vertex attrib to match more prpoerties
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

  _shader.compileShader();
}

void TriangleStrip::render() {
  glUseProgram(_shader.getShaderProgram());
  glBindVertexArray(_vao);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

}  // namespace Garuda
