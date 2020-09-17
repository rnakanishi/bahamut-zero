#include <glad/glad.h>
#include <iostream>
#include <shaders/material.hpp>

namespace Bismarck {

Material::Material() {}

void Material::compileShader() {
  _shader.compileShader();
}

void Material::render() {
  _shader.render();
}

}  // namespace Bismarck
