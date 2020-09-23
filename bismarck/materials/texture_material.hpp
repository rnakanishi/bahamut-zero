#ifndef __BISMARCK_TEXTURE_MATERIAL_HPP__
#define __BISMARCK_TEXTURE_MATERIAL_HPP__

#include <materials/material.hpp>
#include <materials/texture.hpp>
#include <string>

namespace Bismarck {
class TextureMaterial : public Material {
 public:
  TextureMaterial();

  void showUI() override;

 protected:
  Texture _albedo;
  float _diffuseIntensity;
  float _specularIntensity;
};
}  // namespace Bismarck

#endif