#include <glad/glad.h>

#include <iostream>
#include <shaders/shader.hpp>
#include <vector>

namespace Bismarck {
Shader::Shader() {
  _vertexShader = glCreateShader(GL_VERTEX_SHADER);
  _fragShader = glCreateShader(GL_FRAGMENT_SHADER);
  _shaderProgram = glCreateProgram();

  _shaderName = "General Shader";
  _isCompiled = false;
}

Shader::~Shader() {}

unsigned int Shader::getShaderProgram() {
  return _shaderProgram;
}

void Shader::checkShaderErrors(unsigned int shaderId) {
  int success;
  char infoLog[512];
  glGetShaderiv(shaderId, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shaderId, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
  }
}

void Shader::checkProgramErrors(unsigned int programId) {
  int success;
  char infoLog[512];
  glGetProgramiv(programId, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(programId, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER_PROGRAM::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }
  success = 0;
  glGetProgramiv(programId, GL_LINK_STATUS, (int*)&success);
  if (!success) {
    GLint maxLength = 0;
    glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &maxLength);

    // The maxLength includes the NULL character
    GLchar linkLog[maxLength];
    glGetProgramInfoLog(programId, maxLength, &maxLength, linkLog);
    std::cerr << "ERROR::SHADER::PROGRAM::LINK_FAILED\n"
              << linkLog << std::endl;

    // We don't need the program anymore.
    glDeleteProgram(_shaderProgram);
  }
}

void Shader::compileShader() {
  std::string _vertexSource =
      "#version 330 core\n"
      "layout (location = 0) in vec3 aPos;\n"
      "void main()\n"
      "{\n"
      "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
      "}\0";

  std::string _fragSource =
      "#version 330 core\n"
      "out vec4 FragColor;\n"
      "void main()\n"
      "{\n"
      "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
      "}\n\0";

  const GLchar* vertexSource = _vertexSource.c_str();
  const GLchar* fragSource = _fragSource.c_str();
  glShaderSource(_vertexShader, 1, &vertexSource, NULL);
  glCompileShader(_vertexShader);
  checkShaderErrors(_vertexShader);

  glShaderSource(_fragShader, 1, &fragSource, NULL);
  glCompileShader(_fragShader);
  checkShaderErrors(_fragShader);

  glAttachShader(_shaderProgram, _vertexShader);
  glAttachShader(_shaderProgram, _fragShader);
  glLinkProgram(_shaderProgram);
  checkProgramErrors(_shaderProgram);

  // glDeleteShader(_vertexShader);
  // glDeleteShader(_fragShader);
  _isCompiled = true;
}

void Shader::render() {
  if (!_isCompiled) {
    compileShader();
  }
  glUseProgram(_shaderProgram);
}

}  // namespace Bismarck
