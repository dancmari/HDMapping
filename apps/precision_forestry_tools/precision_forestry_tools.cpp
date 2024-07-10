#include <GL/glew.h>

#include <GL/freeglut.h>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include <HDMapping/Version.hpp>

#include <portable-file-dialogs.h>

#include <session.h>

const unsigned int window_width = 800;
const unsigned int window_height = 600;
double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_mode_ortho_z_center_h = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
bool is_ortho = false;
bool show_axes = true;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;

float m_ortho_projection[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

bool is_decimate = true;
double bucket_x = 0.05;
double bucket_y = 0.05;
double bucket_z = 0.05;
bool calculate_offset = false;
Session session;
int viewer_decmiate_point_cloud = 100;
std::vector<int> lowest_points_indexes;

namespace fs = std::filesystem;

unsigned long long int get_index_2D(const int16_t x, const int16_t y /*, const int16_t z*/)
{
  // return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
  //        ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
  //        ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
  return ((static_cast<unsigned long long int>(x) << 16) & (0x00000000FFFF0000ull)) |
         ((static_cast<unsigned long long int>(y) << 0) & (0x000000000000FFFFull));
}

unsigned long long int get_rgd_index_2D(const Eigen::Vector3d p, const Eigen::Vector2d b)
{
  int16_t x = static_cast<int16_t>(p.x() / b.x());
  int16_t y = static_cast<int16_t>(p.y() / b.y());
  // int16_t z = static_cast<int16_t>(p.z() / b.z());
  return get_index_2D(x, y);
}

std::vector<int> get_lowest_points_indexes(const PointCloud &pc, Eigen::Vector2d bucket_dim_xy)
{
  std::vector<int> indexes;

  std::vector<std::tuple<unsigned long long int, unsigned int, double>> indexes_tuple;

  for (size_t i = 0; i < pc.points_local.size(); i++)
  {
    unsigned long long int index = get_rgd_index_2D(pc.points_local[i], bucket_dim_xy);
    indexes_tuple.emplace_back(index, i, pc.points_local[i].z());
  }

  std::sort(indexes_tuple.begin(), indexes_tuple.end(),
            [](const std::tuple<unsigned long long int, unsigned int, unsigned int> &a, const std::tuple<unsigned long long int, unsigned int, unsigned int> &b)
            { return (std::get<0>(a) == std::get<0>(b)) ? (std::get<2>(a) > std::get<2>(b)) : (std::get<0>(a) < std::get<0>(b)); });

  // for (const auto &t : indexes_tuple){
  //   std::cout << std::get<0>(t) << " " << std::get<1>(t) << " " << std::get<2>(t) << std::endl;
  // }

  for (int i = 0; i < indexes_tuple.size(); i++)
  {
    if (i == 0)
    {
      indexes.push_back(std::get<1>(indexes_tuple[i]));
    }
    else
    {
      if (std::get<0>(indexes_tuple[i - 1]) != std::get<0>(indexes_tuple[i]))
      {
        indexes.push_back(std::get<1>(indexes_tuple[i]));
      }
    }
  }
  return indexes;
}

void reshape(int w, int h)
{
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (!is_ortho)
  {
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
  }
  else
  {
    ImGuiIO &io = ImGui::GetIO();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
            -camera_ortho_xy_view_zoom / ratio,
            camera_ortho_xy_view_zoom / ratio, -100000, 100000);
    // glOrtho(-translate_z, translate_z, -translate_z * (float)h / float(w),
    // translate_z * float(h) / float(w), -10000, 10000);
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void motion(int x, int y)
{
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)x, (float)y);

  if (!io.WantCaptureMouse)
  {
    float dx, dy;
    dx = (float)(x - mouse_old_x);
    dy = (float)(y - mouse_old_y);

    if (is_ortho)
    {
      if (mouse_buttons & 1)
      {
        float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
        Eigen::Vector3d v(
            dx * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.x * 2),
            dy * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.y * 2 /
                  ratio),
            0);
        TaitBryanPose pose_tb;
        pose_tb.px = 0.0;
        pose_tb.py = 0.0;
        pose_tb.pz = 0.0;
        pose_tb.om = 0.0;
        pose_tb.fi = 0.0;
        pose_tb.ka = camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
        auto m = affine_matrix_from_pose_tait_bryan(pose_tb);
        Eigen::Vector3d v_t = m * v;
        camera_ortho_xy_view_shift_x += v_t.x();
        camera_ortho_xy_view_shift_y += v_t.y();
      }
    }
    else
    {
      gui_mouse_down = mouse_buttons > 0;
      if (mouse_buttons & 1)
      {
        rotate_x += dy * 0.2f; // * mouse_sensitivity;
        rotate_y += dx * 0.2f; // * mouse_sensitivity;
      }
      if (mouse_buttons & 4)
      {
        translate_x += dx * 0.05f * mouse_sensitivity;
        translate_y -= dy * 0.05f * mouse_sensitivity;
      }
    }

    mouse_old_x = x;
    mouse_old_y = y;
  }
  glutPostRedisplay();
}

void project_gui()
{
  if (ImGui::Begin("main gui window"))
  {
    ImGui::Checkbox("is_decimate", &is_decimate);
    if (is_decimate)
    {
      ImGui::InputDouble("bucket_x", &bucket_x);
      ImGui::InputDouble("bucket_y", &bucket_y);
      ImGui::InputDouble("bucket_z", &bucket_z);
    }
    ImGui::Checkbox("calculate_offset", &calculate_offset);

    ImGui::InputInt("viewer_decmiate_point_cloud", &viewer_decmiate_point_cloud);
    if (viewer_decmiate_point_cloud < 1)
    {
      viewer_decmiate_point_cloud = 1;
    }

    if (ImGui::Button("Load *.laz file"))
    {
      static std::shared_ptr<pfd::open_file> open_file;
      std::vector<std::string> input_file_names;
      ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
      const auto t = [&]()
      {
        std::vector<std::string> filters;
        auto sel = pfd::open_file("Load laz files", "C:\\", filters, true).result();
        for (int i = 0; i < sel.size(); i++)
        {
          input_file_names.push_back(sel[i]);
        }
      };
      std::thread t1(t);
      t1.join();

      if (input_file_names.size() == 1)
      {
        // std::cout << "Las/Laz file (only 1):" << std::endl;
        for (size_t i = 0; i < input_file_names.size(); i++)
        {
          std::cout << input_file_names[i] << std::endl;
        }

        session.working_directory = fs::path(input_file_names[0]).parent_path().string();

        if (!session.point_clouds_container.load_whu_tls(input_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset))
        {
          std::cout << "check input files laz/las" << std::endl;
        }
        else
        {
          std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
        }
      }
      else
      {
        std::cout << "please mark only 1 file" << std::endl;
      }
    }

    if (ImGui::Button("get indexes lowest points"))
    {
      if (session.point_clouds_container.point_clouds.size() > 0)
      {
        lowest_points_indexes = get_lowest_points_indexes(session.point_clouds_container.point_clouds[0], Eigen::Vector2d(0.3, 0.3));
      }
      else
      {
        std::cout << "point cloud data is empty, please load data" << std::endl;
      }
    }

    ImGui::End();
  }
  return;
}

void display()
{
  ImGuiIO &io = ImGui::GetIO();
  glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

  if (is_ortho)
  {

    glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
            -camera_ortho_xy_view_zoom / ratio,
            camera_ortho_xy_view_zoom / ratio, -100000, 100000);

    glm::mat4 proj = glm::orthoLH_ZO<float>(
        -camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
        -camera_ortho_xy_view_zoom / ratio, camera_ortho_xy_view_zoom / ratio,
        -100, 100);

    std::copy(&proj[0][0], &proj[3][3], m_ortho_projection);

    Eigen::Vector3d v_eye_t(-camera_ortho_xy_view_shift_x,
                            camera_ortho_xy_view_shift_y,
                            camera_mode_ortho_z_center_h + 10);
    Eigen::Vector3d v_center_t(-camera_ortho_xy_view_shift_x,
                               camera_ortho_xy_view_shift_y,
                               camera_mode_ortho_z_center_h);
    Eigen::Vector3d v(0, 1, 0);

    TaitBryanPose pose_tb;
    pose_tb.px = 0.0;
    pose_tb.py = 0.0;
    pose_tb.pz = 0.0;
    pose_tb.om = 0.0;
    pose_tb.fi = 0.0;
    pose_tb.ka = -camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
    auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

    Eigen::Vector3d v_t = m * v;

    gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(), v_center_t.x(),
              v_center_t.y(), v_center_t.z(), v_t.x(), v_t.y(), v_t.z());
    glm::mat4 lookat =
        glm::lookAt(glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
                    glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
                    glm::vec3(v_t.x(), v_t.y(), v_t.z()));
    std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);
  }

  glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
               clear_color.z * clear_color.w, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  if (!is_ortho)
  {
    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    Eigen::Affine3f viewTranslation = Eigen::Affine3f::Identity();
    viewTranslation.translate(rotation_center);
    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();
    viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
    viewLocal.rotate(
        Eigen::AngleAxisf(M_PI * rotate_x / 180.f, Eigen::Vector3f::UnitX()));
    viewLocal.rotate(
        Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f viewTranslation2 = Eigen::Affine3f::Identity();
    viewTranslation2.translate(-rotation_center);

    Eigen::Affine3f result = viewTranslation * viewLocal * viewTranslation2;

    glLoadMatrixf(result.matrix().data());
    /*      glTranslatef(translate_x, translate_y, translate_z);
          glRotatef(rotate_x, 1.0, 0.0, 0.0);
          glRotatef(rotate_y, 0.0, 0.0, 1.0);*/
  }
  else
  {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }

  if (ImGui::GetIO().KeyCtrl)
  {
    glBegin(GL_LINES);
    glColor3f(1.f, 1.f, 1.f);
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x() + 1.f, rotation_center.y(),
               rotation_center.z());
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x() - 1.f, rotation_center.y(),
               rotation_center.z());
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x(), rotation_center.y() - 1.f,
               rotation_center.z());
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x(), rotation_center.y() + 1.f,
               rotation_center.z());
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x(), rotation_center.y(),
               rotation_center.z() - 1.f);
    glVertex3fv(rotation_center.data());
    glVertex3f(rotation_center.x(), rotation_center.y(),
               rotation_center.z() + 1.f);
    glEnd();
  }

  if (show_axes)
  {
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(100, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 100, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 100);
    glEnd();
  }

  for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
  {
    session.point_clouds_container.point_clouds[i].render(false, ObservationPicking(), viewer_decmiate_point_cloud);
  }

  if (session.point_clouds_container.point_clouds.size() == 1){
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(3);
    glBegin(GL_POINTS);
    for (int i = 0; i < lowest_points_indexes.size(); i++)
    {
      glVertex3f(session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].x(),
                 session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].y(),
                 session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].z());

      //std::cout << session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].x() << " " <<
      //    session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].y() << " " <<
      //    session.point_clouds_container.point_clouds[0].points_local[lowest_points_indexes[i]].z() << std::endl;
    }
    glEnd();
    glPointSize(1);
  }
  // void render(bool show_with_initial_pose, const ObservationPicking &observation_picking, int viewer_decmiate_point_cloud);

  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();

  project_gui();

  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

  glutSwapBuffers();
  glutPostRedisplay();
}

bool initGL(int *argc, char **argv)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(window_width, window_height);

  glutCreateWindow("precision_forestry_tools " HDMAPPING_VERSION_STRING);

  glutDisplayFunc(display);
  glutMotionFunc(motion);

  // default initialization
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glEnable(GL_DEPTH_TEST);

  // viewport
  glViewport(0, 0, window_width, window_height);

  // projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01,
                 10000.0);
  glutReshapeFunc(reshape);
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable
  // Keyboard Controls

  ImGui::StyleColorsDark();
  ImGui_ImplGLUT_Init();
  ImGui_ImplGLUT_InstallFuncs();
  ImGui_ImplOpenGL2_Init();
  return true;
}

void mouse(int glut_button, int state, int x, int y)
{
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)x, (float)y);
  int button = -1;
  if (glut_button == GLUT_LEFT_BUTTON)
    button = 0;
  if (glut_button == GLUT_RIGHT_BUTTON)
    button = 1;
  if (glut_button == GLUT_MIDDLE_BUTTON)
    button = 2;
  if (button != -1 && state == GLUT_DOWN)
    io.MouseDown[button] = true;
  if (button != -1 && state == GLUT_UP)
    io.MouseDown[button] = false;

  if (!io.WantCaptureMouse)
  {
    if (glut_button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
    {
    }

    if (state == GLUT_DOWN)
    {
      mouse_buttons |= 1 << glut_button;
    }
    else if (state == GLUT_UP)
    {
      mouse_buttons = 0;
    }
    mouse_old_x = x;
    mouse_old_y = y;
  }
}

void wheel(int button, int dir, int x, int y)
{
  if (dir > 0)
  {
    if (is_ortho)
    {
      camera_ortho_xy_view_zoom -= 0.1f * camera_ortho_xy_view_zoom;

      if (camera_ortho_xy_view_zoom < 0.1)
      {
        camera_ortho_xy_view_zoom = 0.1;
      }
    }
    else
    {
      translate_z -= 0.05f * translate_z;
    }
  }
  else
  {
    if (is_ortho)
    {
      camera_ortho_xy_view_zoom += 0.1 * camera_ortho_xy_view_zoom;
    }
    else
    {
      translate_z += 0.05f * translate_z;
    }
  }

  return;
}

int main(int argc, char *argv[])
{
  initGL(&argc, argv);
  glutDisplayFunc(display);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutMouseWheelFunc(wheel);
  glutMainLoop();

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGLUT_Shutdown();

  ImGui::DestroyContext();
  return 0;
}