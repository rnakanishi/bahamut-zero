#ifndef __GARUDA_TRIANGLE_STRIPS_HPP__
#define __GARUDA_TRIANGLE_STRIPS_HPP__

#include <Eigen/Dense>
#include <map>
#include <renderable/render_object.hpp>
#include <string>
#include <vector>

namespace Garuda {
class TriangleStrip : public RenderObject {
 public:
  /**
   * @brief Construct a new Triangle Strip object. This class' objects always
   * have a position property stored. Additionally, normals and any other vector
   * or scalar properties can be added.
   *
   * Vector properties always have 3 or 4 components (x, y, z) or (r, g, b, a)
   *   - If vector does not have all components (like 2D positions), then it
   * should be filled before adding to the object.
   *  Scalar properties are always float based values
   *
   */
  TriangleStrip();
  TriangleStrip(std::vector<Eigen::Array3f> positions);

  ~TriangleStrip();

  /**
   * @brief Adds a vector property with a propertyName. The full content of the
   * vector should be given as parameter as well. If the count of properties
   * does not match the vertices count (number of vertices), then a exception is
   * raised
   *
   * @param propertyName name of the added property
   * @param content vector containing the values for this multidimensiontal
   * property
   * @return int the index of the property stored in the class
   */
  int addVerticeProperty(std::string propertyName,
                         std::vector<Eigen::Array4f> content);
  int addVerticeProperty(std::string propertyName,
                         std::vector<Eigen::Array3f> content);
  int addVerticeProperty(std::string propertyName,
                         std::vector<Eigen::Array2f> content);

  int addFaceProperty(std::string propertyName,
                      std::vector<Eigen::Array3i> content);
  int addFaceProperty(std::string propertyName,
                      std::vector<Eigen::Array3f> content);
  int addFaceProperty(std::string propertyName,
                      std::vector<Eigen::Array4f> content);

  /**
   * @brief Adds a scalar property with a propertyName. The full content of the
   * vector should be given as parameter as well. If the count of properties
   * does not match the vertices count (number of vertices), then a exception is
   * raised
   *
   * @param propertyName name of the added property
   * @param content vector containing the values for this scalar property
   * @return int the index of the property stored in the class
   */
  int addScalarProperty(std::string propertyName, std::vector<float> content);

  void render() override;

  void sendDataToBuffer() override;

 private:
  std::map<std::string, unsigned int> vertexVectorLabels;
  std::map<std::string, unsigned int> faceVectorLabels;
  std::map<std::string, unsigned int> vertexScalarLabels;
  std::map<std::string, unsigned int> faceScalarLabels;

  std::vector<std::vector<float>>
      vertexVectorPropeties, /**! Vector properties are linearized in C array
                           format*/
      vertexScalarProperties;

  std::vector<std::vector<float>> faceVectorPropeties, faceScalarProperties;
  std::vector<std::vector<int>> faceIntProperties;

  std::vector<unsigned int> vectorPropertyDimension;
  std::vector<std::vector<float>> vertexSacalarPropeties;
};
}  // namespace Garuda

#endif