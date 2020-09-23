#include <glad/glad.h>
#include <iostream>
#include <materials/material.hpp>

namespace Bismarck {

Material::Material() {}

void Material::compileShader() {
  _shader.compileShader();
}

void Material::render() {
  _shader.render();
}

void Material::showUI() {}

}  // namespace Bismarck
