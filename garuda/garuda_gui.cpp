#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <garuda_gui.hpp>
#include <iostream>

namespace Garuda {
GarudaGUI::GarudaGUI() : GarudaGUI(4, 0) {}

GarudaGUI::GarudaGUI(int major, int minor)
    : _glfwMajor(major), _glfwMinor(minor) {
  glsl_version = std::string("#version 130");

  if (!glfwInit())
    std::cerr << "Error initializing glfw\n";
  glfwSetErrorCallback(GarudaGUI::errorCallback);
  // glfwSetFramebufferSizeCallback(_window,
  // GarudaGUI::framebufferResizeCallback);
  //   return 1;
  // TODO: Throw exception when glfw raises an error

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, _glfwMajor);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, _glfwMinor);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Create window with graphics context
  _window =
      glfwCreateWindow(800, 800, "Garuda Graphical Interface", NULL, NULL);
  if (_window == NULL)
    std::cerr << "Error initializing window\n";
  //   return 1;
  glfwMakeContextCurrent(_window);
  glfwSwapInterval(1);  // Enable vsync

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(_window, true);
  ImGui_ImplOpenGL3_Init(glsl_version.c_str());
}

void GarudaGUI::errorCallback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void GarudaGUI::framebufferResizeCallback(GLFWwindow* window,
                                          int width,
                                          int height) {
  glViewport(0, 0, width, height);
}

void GarudaGUI::newFrame() {
  int display_w, display_h;
  glfwGetFramebufferSize(_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void GarudaGUI::renderFrame() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwPollEvents();
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
