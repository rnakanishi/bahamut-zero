#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <garuda_gui.hpp>
#include <iostream>

namespace Garuda {

GarudaGUI::GarudaGUI() {
  glsl_version = std::string("#version 130");
  _glfwMajor = 3;
  _glfwMinor = 0;
}

void GarudaGUI::errorCallback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void GarudaGUI::inializeContext() {
  glfwSetErrorCallback(GarudaGUI::errorCallback);
  if (!glfwInit())
    std::cerr << "Error initializing glfw\n";
  //   return 1;
  // TODO: Throw exception when glfw raises an error

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, _glfwMajor);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, _glfwMinor);

  // Create window with graphics context
  _window = glfwCreateWindow(1280, 720, "Dear ImGui GLFW+OpenGL3 example", NULL,
                             NULL);
  if (_window == NULL)
    std::cerr << "Error initializing window\n";
  //   return 1;
  glfwMakeContextCurrent(_window);
  glfwSwapInterval(1);  // Enable vsync

  bool err = gladLoadGL() == 0;
  // if (err) {
  //   fprintf(stderr, "Failed to initialize OpenGL loader!\n");
  //   return 1;
  // }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(_window, true);
  ImGui_ImplOpenGL3_Init(glsl_version.c_str());
}

void GarudaGUI::newFrame() {
  glfwPollEvents();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void GarudaGUI::renderFrame() {
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  // glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(_window);
}

void GarudaGUI::finalizeContext() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(_window);
  glfwTerminate();
}

GLFWwindow* GarudaGUI::getWindow() {
  return _window;
}

}  // namespace Garuda
