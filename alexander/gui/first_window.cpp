#include <imgui/imgui.h>

#include <cmath>
#include <garuda_gui.hpp>
#include <iostream>

int main(int argc, char const* argv[]) {
  Garuda::GarudaGUI interface;
  interface.inializeContext();

  char* displayText;
  float value;
  bool demoWindow = true;
  while (!glfwWindowShouldClose(interface.getWindow())) {
    interface.newFrame();

    ImGui::ShowDemoWindow(&demoWindow);
    interface.renderFrame();
  }

  interface.finalizeContext();
  return 0;
}
