#include <glad/glad.h>

#include <renderable/render_object.hpp>

namespace Garuda {

RenderObject::RenderObject() {
  glGenVertexArrays(1, &_vao);
  glGenBuffers(1, &_vbo);
  glGenBuffers(1, &_ebo);

  glBindVertexArray(_vao);
  glBindBuffer(GL_ARRAY_BUFFER, _vbo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
}

RenderObject::~RenderObject() {
  glDeleteVertexArrays(1, &_vao);
  glDeleteBuffers(1, &_vbo);
}

}  // namespace Garuda
