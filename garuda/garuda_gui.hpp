
#include <glad/glad.h>
// Glad should be included first
#include <GLFW/glfw3.h>

#include <memory>
#include <string>

namespace Garuda {

/**
 * @brief Class for setting all the GLFW and ImGUI environment.
 * The usual calls for the methods inside this class are described as follows:
 *
 * - First call:
 * initializeContext() -> sets the environmemt
 *
 * - Inside the application loop:
 *    newFrame(); --> creates a frame to render the objects
 *    ... Application loop
 *    renderFrame() --> render the frame to the screen
 *
 * - After finishing the application loop:
 * finalizeContext() --> Cleans GLFW and ImGui environments
 *
 */
class GarudaGUI {
 public:
  /**
   * @brief Construct a new Garuda G U I object
   * It is responsible for setting up all GLFW and OpenGl context. After that,
   * ImGUI environment is set.
   *
   */
  GarudaGUI();
  GarudaGUI(int glfwMajor, int glfwMinor);

  /**
   * @brief This method should be the first thing called in the rendering loop.
   * It is responsible for setting the environment Frame. All the scene is
   * rendered in this frame so it can be showed after in the screens
   *
   */
  void newFrame();

  /**
   * @brief This method is the last thing called inside the rendering loop. It
   * is responsible for showing the objects on the Frame.
   *
   */
  void renderFrame();

  /**
   * @brief This methhod is the last thing called in the gui application. It is
   * responsible for cleaning up the glfw environment and freeing its variables.
   *
   */
  void finalizeContext();

  /**
   * @brief Get the Window object. This is useful when using methods that
   * require direct manipulation of the glfw window
   *
   * USE THIS METHOD WITH CAUTION
   *
   * @return GLFWwindow*
   */
  GLFWwindow* getWindow();

  /**
   * @brief Default error callback for the GLFW
   *
   * @param error
   * @param description
   */
  static void errorCallback(int error, const char* description);

  /**
   * @brief This callback is used for GLFW to resize objects inside window
   *
   * @param window
   * @param width
   * @param height
   */
  static void framebufferResizeCallback(GLFWwindow* window,
                                        int width,
                                        int height);

 private:
  GLFWwindow* _window;
  // GL 3.0 + GLSL 130
  std::string glsl_version;
  int _glfwMajor;
  int _glfwMinor;
};

}  // namespace Garuda
