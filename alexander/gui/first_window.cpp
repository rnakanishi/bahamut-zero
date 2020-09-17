#include <imgui/imgui.h>

#include <cmath>
#include <garuda_gui.hpp>
#include <iostream>
#include <renderable/triangle_mesh.hpp>
#include <shaders/shader.hpp>

#include "scene_object.hpp"

int main(int argc, char const* argv[]) {
  Garuda::GarudaGUI interface;
  Garuda::TriangleMesh triangles;
  Alexander::SceneObject object;

  triangles.addVerticeProperty("positions", object.loadObjectVertices());
  triangles.addFaceProperty("vertexId", object.loadObjectFaces());

  char* displayText;
  float value;
  bool demoWindow = true;

  triangles.sendDataToBuffer();

  while (!glfwWindowShouldClose(interface.getWindow())) {
    interface.newFrame();

    ImGui::ShowDemoWindow(&demoWindow);
    triangles.render();
    interface.renderFrame();
  }

  interface.finalizeContext();
  return 0;
}
