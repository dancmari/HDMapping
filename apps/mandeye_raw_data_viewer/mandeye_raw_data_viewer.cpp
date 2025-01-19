#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include <portable-file-dialogs.h>

#include <filesystem>
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"

#include <HDMapping/Version.hpp>

#include <mutex>

#include <export_laz.h>

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

const unsigned int window_width = 800;
const unsigned int window_height = 600;
double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_mode_ortho_z_center_h = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
bool is_ortho = false;
bool show_axes = true;
ImVec4 clear_color = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
ImVec4 pc_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);

int point_size = 1;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

struct AllData
{
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Eigen::Affine3d> poses;
    std::vector<Point3Di> points_local;
    std::vector<int> lidar_ids;
};

std::vector<AllData> all_data;

std::vector<std::string> laz_files;
std::vector<std::string> csv_files;
std::vector<std::string> sn_files;
std::string working_directory = "";
std::string imuSnToUse;
std::string working_directory_preview;
double filter_threshold_xy = 0.1;
bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
int number_of_points_threshold = 20000;
bool is_init = true;
int index_rendered_points_local = -1;
// std::vector<std::vector<Point3Di>> all_points_local;
// std::vector<std::vector<Point3Di>> all_points_local;
// std::vector<std::vector<int>> all_lidar_ids;
std::vector<int> indexes_to_filename;
std::vector<std::string> all_file_names;
double ahrs_gain = 0.5;
// bool is_slerp = false;

const std::vector<std::string> LAS_LAZ_filter = {"LAS file (*.laz)", "*.laz", "LASzip file (*.las)", "*.las", "All files", "*"};

double rgd_x = 0.5;
double rgd_y = 5;
double rgd_z = 5;

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rgd_nn;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> mean_cov;
bool show_mean_cov = false;
bool show_rgd_nn = false;

void optimize();
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> get_nn();
void draw_ellipse(const Eigen::Matrix3d &covar, Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd);
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> get_mean_cov();

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
        // glOrtho(-translate_z, translate_z, -translate_z * (float)h / float(w), translate_z * float(h) / float(w), -10000, 10000);
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
                Eigen::Vector3d v(dx * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.x * 2),
                                  dy * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.y * 2 / ratio), 0);
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
        ImGui::ColorEdit3("clear color", (float *)&clear_color);
        ImGui::ColorEdit3("pc_color_lidar_1", (float *)&pc_color);
        ImGui::ColorEdit3("pc_color_lidar_2", (float *)&pc_color2);

        ImGui::InputInt("point_size", &point_size);
        if (point_size < 1)
        {
            point_size = 1;
        }

        if (is_init)
        {
            ImGui::InputInt("number_of_points_threshold", &number_of_points_threshold);
            if (number_of_points_threshold < 0)
            {
                number_of_points_threshold = 0;
            }
        }

        ImGui::InputDouble("ahrs_gain", &ahrs_gain);
        // ImGui::Checkbox("is_slerp", &is_slerp);

        ImGui::InputInt("index_rendered_points_local", &index_rendered_points_local);
        if (index_rendered_points_local < 0)
        {
            index_rendered_points_local = 0;
        }
        if (index_rendered_points_local >= all_data.size())
        {
            index_rendered_points_local = all_data.size() - 1;
        }

        if (is_init)
        {
            if (ImGui::Button("load data"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                    }
                };
                std::thread t1(t);
                t1.join();

                std::sort(std::begin(input_file_names), std::end(input_file_names));

                std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string &fileName)
                              {
                    if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
                    {
                        laz_files.push_back(fileName);
                        all_file_names.push_back(fileName);
                    }
                    if (fileName.ends_with(".csv"))
                    {
                        csv_files.push_back(fileName);
                    }
                    if (fileName.ends_with(".sn"))
                    {
                        sn_files.push_back(fileName);
                    } });

                if (input_file_names.size() > 0 && laz_files.size() == csv_files.size())
                {
                    working_directory = fs::path(input_file_names[0]).parent_path().string();

                    const auto calibrationFile = (fs::path(working_directory) / "calibration.json").string();
                    const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
                    imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
                    if (!preloadedCalibration.empty())
                    {
                        std::cout << "Loaded calibration for: \n";
                        for (const auto &[sn, _] : preloadedCalibration)
                        {
                            std::cout << " -> " << sn << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "There is no calibration.json file in folder (check comment in source code) file: " << __FILE__ << " line: " << __LINE__ << std::endl;
                        // example file for 2x livox";
                        /*
                        {
                            "calibration" : {
                                "47MDL9T0020193" : {
                                    "identity" : "true"
                                },
                                "47MDL9S0020300" :
                                    {
                                        "order" : "ROW",
                                        "inverted" : "TRUE",
                                        "data" : [
                                            0.999824, 0.00466397, -0.0181595, -0.00425984,
                                            -0.0181478, -0.00254457, -0.999832, -0.151599,
                                            -0.0047094, 0.999986, -0.00245948, -0.146408,
                                            0, 0, 0, 1
                                        ]
                                    }
                            },
                                            "imuToUse" : "47MDL9T0020193"
                        }*/
                    }

                    fs::path wdp = fs::path(input_file_names[0]).parent_path();
                    wdp /= "preview";
                    if (!fs::exists(wdp))
                    {
                        fs::create_directory(wdp);
                    }

                    working_directory_preview = wdp.string();

                    for (size_t i = 0; i < input_file_names.size(); i++)
                    {
                        std::cout << input_file_names[i] << std::endl;
                    }
                    std::cout << "loading imu" << std::endl;
                    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

                    for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
                    {
                        const std::string &imufn = csv_files.at(fileNo);
                        const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
                        const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
                        // GetId of Imu to use
                        int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
                        std::cout << "imuNumberToUse  " << imuNumberToUse << " at" << imufn << std::endl;
                        auto imu = load_imu(imufn.c_str(), imuNumberToUse);
                        std::cout << imufn << " with mapping " << snFn << std::endl;
                        imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu));
                    }

                    std::cout << "loading points" << std::endl;
                    std::vector<std::vector<Point3Di>> pointsPerFile;
                    pointsPerFile.resize(laz_files.size());
                    // std::vector<std::vector<int>> indexesPerFile;

                    std::mutex mtx;
                    std::cout << "start std::transform" << std::endl;

                    std::transform(std::execution::par_unseq, std::begin(laz_files), std::end(laz_files), std::begin(pointsPerFile), [&](const std::string &fn)
                                   {
                                       // Load mapping from id to sn
                                       fs::path fnSn(fn);
                                       fnSn.replace_extension(".sn");

                                       // GetId of Imu to use
                                       const auto idToSn = MLvxCalib::GetIdToSnMapping(fnSn.string());
                                       auto calibration = MLvxCalib::CombineIntoCalibration(idToSn, preloadedCalibration);
                                       auto data = load_point_cloud(fn.c_str(), true, filter_threshold_xy, calibration);

                                       if (fn == laz_files.front())
                                       {
                                           fs::path calibrationValidtationFile = wdp / "calibrationValidation.asc";

                                           std::ofstream testPointcloud{calibrationValidtationFile.c_str()};
                                           for (const auto &p : data)
                                           {
                                               testPointcloud << p.point.x() << "\t" << p.point.y() << "\t" << p.point.z() << "\t" << p.intensity << "\t" << (int)p.lidarid << "\n";
                                           }
                                       }

                                       std::unique_lock lck(mtx);
                                       for (const auto &[id, calib] : calibration)
                                       {
                                           std::cout << " id : " << id << std::endl;
                                           std::cout << calib.matrix() << std::endl;
                                       }
                                       return data;
                                       // std::cout << fn << std::endl;
                                       //
                                   });
                    std::cout << "std::transform finished" << std::endl;

                    FusionAhrs ahrs;
                    FusionAhrsInitialise(&ahrs);

                    if (fusionConventionNwu)
                    {
                        ahrs.settings.convention = FusionConventionNwu;
                    }
                    if (fusionConventionEnu)
                    {
                        ahrs.settings.convention = FusionConventionEnu;
                    }
                    if (fusionConventionNed)
                    {
                        ahrs.settings.convention = FusionConventionNed;
                    }
                    ahrs.settings.gain = ahrs_gain;

                    std::map<double, std::pair<Eigen::Matrix4d, double>> trajectory;

                    int counter = 1;
                    for (const auto &[timestamp_pair, gyr, acc] : imu_data)
                    {
                        const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
                        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

                        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

                        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

                        Eigen::Quaterniond d{quat.element.w, quat.element.x, quat.element.y, quat.element.z};
                        Eigen::Affine3d t{Eigen::Matrix4d::Identity()};
                        t.rotate(d);

                        trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
                        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                        counter++;
                        if (counter % 100 == 0)
                        {
                            printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
                        }
                    }

                    std::vector<std::pair<double, double>> timestamps;
                    std::vector<Eigen::Affine3d> poses;
                    for (const auto &t : trajectory)
                    {
                        timestamps.emplace_back(t.first, t.second.second);
                        Eigen::Affine3d m;
                        m.matrix() = t.second.first;
                        poses.push_back(m);
                    }

                    int number_of_points = 0;
                    for (const auto &pp : pointsPerFile)
                    {
                        number_of_points += pp.size();
                    }
                    std::cout << "number of points: " << number_of_points << std::endl;

                    std::cout << "start indexing points" << std::endl;

                    // std::vector<Point3Di> points_global;
                    std::vector<Point3Di> points_local;
                    std::vector<int> lidar_ids;

                    Eigen::Affine3d m_prev;
                    Eigen::Affine3d m_next;

                    Eigen::Quaterniond q_prev;
                    Eigen::Quaterniond q_next;
                    Eigen::Quaterniond q;

                    double time_prev;
                    double time_next;
                    double curr_time;

                    double t;

                    for (int i = 0; i < pointsPerFile.size(); i++)
                    {
                        std::cout << "indexed: " << i + 1 << " of " << pointsPerFile.size() << " files" << std::endl;
                        for (const auto &pp : pointsPerFile[i])
                        {
                            auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), pp.timestamp,
                                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                                          { return lhs.first < rhs; });

                            int index_pose = std::distance(timestamps.begin(), lower);

                            if (index_pose >= 0 && index_pose < poses.size())
                            {
                                auto ppp = pp;
                                // Eigen::Affine3d m = poses[index_pose];
                                /*if (is_slerp)
                                {
                                    if (index_pose > 0)
                                    {
                                        m_prev = poses[index_pose - 1];
                                        m_next = poses[index_pose];

                                        q_prev = Eigen::Quaterniond(m_prev.rotation());
                                        q_next = Eigen::Quaterniond(m_next.rotation());

                                        time_prev = timestamps[index_pose - 1].first;
                                        time_next = timestamps[index_pose].first;
                                        curr_time = ppp.timestamp;

                                        t = (curr_time - time_prev) / (time_next - time_prev);
                                        q = q_prev.slerp(t, q_next);

                                        m.linear() = q.toRotationMatrix();
                                    }
                                }*/

                                points_local.push_back(ppp);

                                // ppp.point = m * ppp.point;
                                lidar_ids.push_back(pp.lidarid);
                                // points_global.push_back(ppp);
                            }

                            if (points_local.size() > number_of_points_threshold)
                            {
                                // all_points_local.push_back(points_global);
                                indexes_to_filename.push_back(i);
                                // points_global.clear();
                                // all_lidar_ids.push_back(lidar_ids);

                                ///////////////////////////////////////
                                AllData data;
                                data.points_local = points_local;
                                data.lidar_ids = lidar_ids;

                                for (int i = 0; i < timestamps.size(); i++)
                                {
                                    if (timestamps[i].first >= points_local[0].timestamp && timestamps[i].first <= points_local[points_local.size() - 1].timestamp)
                                    {
                                        data.timestamps.push_back(timestamps[i]);
                                        data.poses.push_back(poses[i]);
                                    }
                                }
                                all_data.push_back(data);

                                points_local.clear();
                                lidar_ids.clear();
                                //////////////////////////////////////
                            }
                        }
                    }

                    std::cout << "indexing points finished" << std::endl;

                    if (all_data.size() > 0)
                    {
                        is_init = false;
                        index_rendered_points_local = 0;
                    }
                }
                else
                {
                    std::cout << "please select files correctly" << std::endl;
                    std::cout << "input_file_names.size(): " << input_file_names.size() << std::endl;
                    std::cout << "laz_files.size(): " << laz_files.size() << std::endl;
                    std::cout << "csv_files.size(): " << csv_files.size() << std::endl;

                    std::cout << "condition: input_file_names.size() > 0 && laz_files.size() == csv_files.size() NOT SATISFIED!!!" << std::endl;
                }
            }
        }

        if (all_file_names.size() > 0)
        {
            if (index_rendered_points_local >= 0 && index_rendered_points_local < indexes_to_filename.size())
            {
                std::string fn = all_file_names[indexes_to_filename[index_rendered_points_local]];
                ImGui::Text(fn.c_str());
            }
        }

        if (ImGui::Button("save point cloud"))
        {
            std::shared_ptr<pfd::save_file> save_file;
            std::string output_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
            const auto t = [&]()
            {
                auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                output_file_name = sel;
                std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
            };
            std::thread t1(t);
            t1.join();

            if (output_file_name.size() > 0)
            {
                std::vector<Eigen::Vector3d> pointcloud;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;

                /*if (index_rendered_points_local >= 0 && index_rendered_points_local < all_points_local.size())
                {
                    for (int i = 0; i < all_points_local[index_rendered_points_local].size(); i++)
                    {
                        pointcloud.emplace_back(all_points_local[index_rendered_points_local][i].point.x(), all_points_local[index_rendered_points_local][i].point.y(), all_points_local[index_rendered_points_local][i].point.z());
                        intensity.push_back(all_points_local[index_rendered_points_local][i].intensity);
                        timestamps.push_back(all_points_local[index_rendered_points_local][i].timestamp);
                    }
                }*/

                if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, 0, 0, 0))
                {
                    std::cout << "problem with saving file: " << output_file_name << std::endl;
                }
            }
        }

        if (ImGui::Button("optimize"))
        {
            optimize();
        }

        ImGui::InputDouble("rgd_x", &rgd_x);
        ImGui::InputDouble("rgd_y", &rgd_y);
        ImGui::InputDouble("rgd_z", &rgd_z);
        ImGui::Checkbox("show show_rgd_nn", &show_rgd_nn);

        if (ImGui::Button("get_nn"))
        {
            rgd_nn = get_nn();
        }

        ImGui::Checkbox("show_mean_cov", &show_mean_cov);

        if (ImGui::Button("get_mean_cov"))
        {
            mean_cov = get_mean_cov();
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

        glm::mat4 proj = glm::orthoLH_ZO<float>(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                                                -camera_ortho_xy_view_zoom / ratio,
                                                camera_ortho_xy_view_zoom / ratio, -100, 100);

        std::copy(&proj[0][0], &proj[3][3], m_ortho_projection);

        Eigen::Vector3d v_eye_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h + 10);
        Eigen::Vector3d v_center_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h);
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

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(),
                  v_center_t.x(), v_center_t.y(), v_center_t.z(),
                  v_t.x(), v_t.y(), v_t.z());
        glm::mat4 lookat = glm::lookAt(glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
                                       glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
                                       glm::vec3(v_t.x(), v_t.y(), v_t.z()));
        std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);
    }

    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

        Eigen::Affine3f viewTranslation = Eigen::Affine3f::Identity();
        viewTranslation.translate(rotation_center);
        Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();
        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
        viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_x / 180.f, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));

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
        glVertex3f(rotation_center.x() + 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x() - 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() - 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() + 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() - 1.f);
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() + 1.f);
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

    //
    /* glPointSize(point_size);
     glBegin(GL_POINTS);
     if (index_rendered_points_local >= 0 && index_rendered_points_local < all_points_local.size())
     {
         for (int i = 0; i < all_points_local[index_rendered_points_local].size(); i++)
         {
             if (all_lidar_ids[index_rendered_points_local][i] == 0)
             {
                 glColor3f(pc_color.x, pc_color.y, pc_color.z);
             }
             else
             {
                 glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
             }
             glVertex3f(all_points_local[index_rendered_points_local][i].point.x(), all_points_local[index_rendered_points_local][i].point.y(), all_points_local[index_rendered_points_local][i].point.z());
         }
     }
     glEnd();
     glPointSize(1);*/

    glPointSize(point_size);
    glBegin(GL_POINTS);
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        for (int i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(all_data[index_rendered_points_local].timestamps.begin(), all_data[index_rendered_points_local].timestamps.end(), all_data[index_rendered_points_local].points_local[i].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });

            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower);

            if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
            {
                Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                Eigen::Vector3d p = m * all_data[index_rendered_points_local].points_local[i].point;

                if (all_data[index_rendered_points_local].lidar_ids[i] == 0)
                {
                    glColor3f(pc_color.x, pc_color.y, pc_color.z);
                }
                else
                {
                    glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
                }
                glVertex3f(p.x(), p.y(), p.z());
            }
        }
    }
    glEnd();
    glPointSize(1);

    if (show_rgd_nn)
    {
        glColor3f(0, 0, 0);
        glBegin(GL_LINES);
        for (const auto &nn : rgd_nn)
        {
            glVertex3f(nn.first.x(), nn.first.y(), nn.first.z());
            glVertex3f(nn.second.x(), nn.second.y(), nn.second.z());
        }
        glEnd();
    }

    if (show_mean_cov)
    {
        for (const auto &mc : mean_cov)
        {
            draw_ellipse(mc.second, mc.first, Eigen::Vector3f(1, 0, 0), 1);
        }
    }

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
    glutCreateWindow("mandeye raw data viewer " HDMAPPING_VERSION_STRING);
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
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01, 10000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

void wheel(int button, int dir, int x, int y);

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

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) &&
        glutMajorVersion < 3)
    {
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);
    }

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

void optimize()
{
    //if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    //{
    //    auto worker_data = all_data[index_rendered_points_local];

    //    auto tr = worker_data.intermediate_trajectory;
    //    auto trmm = worker_data.intermediate_trajectory_motion_model;
    //}

#if 0
    if (params.use_robust_and_accurate_lidar_odometry)
    {
        

        auto firstm = tr[0];

        for (auto &t : tr)
        {
            t.translation() -= firstm.translation();
        }
        for (auto &t : trmm)
        {
            t.translation() -= firstm.translation();
        }

        NDT::GridParameters rgd_params_sc;

        rgd_params_sc.resolution_X = params.distance_bucket;
        rgd_params_sc.resolution_Y = params.polar_angle_deg;
        rgd_params_sc.resolution_Z = params.azimutal_angle_deg;

        // for (int iter = 0; iter < 10; iter++)
        //{
        //     optimize_sf(worker_data[i].intermediate_points, tr, trmm,
        //                 rgd_params_sc, params.buckets, /*params.useMultithread*/ false);
        // }
        std::vector<Point3Di> points_local_sf;
        std::vector<Point3Di> points_local;

        ///
        for (int ii = 0; ii < worker_data[i].intermediate_points.size(); ii++)
        {
            double r_l = worker_data[i].intermediate_points[ii].point.norm();
            if (r_l > 0.5 && worker_data[i].intermediate_points[ii].index_pose != -1 && r_l < params.max_distance_lidar)
            {
                double polar_angle_deg_l = atan2(worker_data[i].intermediate_points[ii].point.y(), worker_data[i].intermediate_points[ii].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data[i].intermediate_points[ii].point.z() / r_l) / M_PI * 180.0;

                points_local.push_back(worker_data[i].intermediate_points[ii]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data[i].intermediate_points[ii];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sf.push_back(p_sl);
            }
        }
        ///
        std::cout << "optimize_sf2" << std::endl;
        for (int iter = 0; iter < params.robust_and_accurate_lidar_odometry_iterations; iter++)
        {
            optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, params.useMultithread);
        }

        for (auto &t : tr)
        {
            t.translation() += firstm.translation();
        }

        for (auto &t : trmm)
        {
            t.translation() += firstm.translation();
        }

        worker_data[i].intermediate_trajectory = tr;
        worker_data[i].intermediate_trajectory_motion_model = tr;
    }
#endif
#if 0
    bool multithread = false;
    std::cout << "optimize" << std::endl;
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // for (int i = 0; i < all_data[index_rendered_points_local].poses.size(); i++){
        //     all_data[index_rendered_points_local].poses[i](0, 3) += 1.0;
        // }

        

        // index data
        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            auto lower = std::lower_bound(worker_data.timestamps.begin(), worker_data.timestamps.end(), worker_data.points_local[i].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });

            int index_pose = std::distance(worker_data.timestamps.begin(), lower);

            if (index_pose >= 0 && index_pose < worker_data.poses.size())
            {
                worker_data.points_local[i].index_pose = index_pose;
            }
            else
            {
                worker_data.points_local[i].index_pose = -1;
            }
        }

        NDT::GridParameters rgd_params;
        // rgd_params.resolution_X = 0.3; // distance bucket
        // rgd_params.resolution_Y = 0.3; // polar angle deg
        // rgd_params.resolution_Z = 0.3; // azimutal angle deg

        rgd_params.resolution_X = rgd_x; // distance bucket
        rgd_params.resolution_Y = rgd_y; // polar angle deg
        rgd_params.resolution_Z = rgd_z; // azimutal angle deg

        std::vector<Point3Di> point_cloud_global;
        std::vector<Point3Di> points_local;

        std::vector<Eigen::Vector3d> point_cloud_global_sc;
        std::vector<Point3Di> points_local_sc;

        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            double r_l = worker_data.points_local[i].point.norm();
            if (r_l > 0.5 && worker_data.points_local[i].index_pose != -1 && r_l < 10)
            {
                double polar_angle_deg_l = atan2(worker_data.points_local[i].point.y(), worker_data.points_local[i].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data.points_local[i].point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d pp = worker_data.points_local[i].point;
                // pps.x() = r;
                // pps.y() = polar_angle_deg;
                // pps.z() = azimutal_angle_deg;
                // point_cloud_spherical_coordinates.push_back(pps);

                Eigen::Affine3d pose = worker_data.poses[worker_data.points_local[i].index_pose];

                pp = pose * pp;

                Point3Di pg = worker_data.points_local[i];
                pg.point = pp;

                point_cloud_global.push_back(pg);
                points_local.push_back(worker_data.points_local[i]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data.points_local[i];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sc.push_back(p_sl);
                //
                double r_g = pg.point.norm();
                double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_g = acos(pg.point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d p_sg = worker_data.points_local[i].point;
                p_sg.x() = r_g;
                p_sg.y() = polar_angle_deg_g;
                p_sg.z() = azimutal_angle_deg_g;

                point_cloud_global_sc.push_back(p_sg);
            }
        }

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc, {0, 0, 0});
        // update_rgd(rgd_params, buckets, point_cloud_global, {0, 0, 0});
        std::cout << "buckets.size(): " << buckets.size() << std::endl;

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

        const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
        {
            int ir = tripletListB.size();
            double delta_x;
            double delta_y;
            double delta_z;

            Eigen::Affine3d m_pose = worker_data.poses[intermediate_points_i.index_pose];
            Eigen::Vector3d point_local(intermediate_points_i.point.x(), intermediate_points_i.point.y(), intermediate_points_i.point.z());
            Eigen::Vector3d point_global = m_pose * point_local;

            ///////////////
            double r = point_global.norm();
            double polar_angle_deg = atan2(point_global.y(), point_global.x()) / M_PI * 180.0;
            double azimutal_angle_deg = acos(point_global.z() / r) / M_PI * 180.0;
            ///////////////

            auto index_of_bucket = get_rgd_index({r, polar_angle_deg, azimutal_angle_deg}, b);
            // auto index_of_bucket = get_rgd_index(point_global, b);

            auto bucket_it = buckets.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets.end())
            {
                return;
            }
            auto &this_bucket = bucket_it->second;

            Eigen::Vector3d mean(this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

            point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                          pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                          point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                   pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                   point_local.x(), point_local.y(), point_local.z());

            int c = intermediate_points_i.index_pose * 6;
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    if (jacobian(row, col) != 0.0)
                    {
                        tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
                    }
                }
            }

            Eigen::Matrix3d infm = this_bucket.cov.inverse();

            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            tripletListP.emplace_back(ir, ir, infm(0, 0));
            tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
            tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
            tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
            tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
            tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
            tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
            tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
            tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));
        };

        if (points_local.size() > 100)
        {
            std::cout << "start adding lidar observations" << std::endl;
            if (multithread)
            {
                std::for_each(std::execution::par_unseq, std::begin(points_local), std::end(points_local), hessian_fun);
            }
            else
            {
                std::for_each(std::begin(points_local), std::end(points_local), hessian_fun);
            }
            std::cout << "adding lidar observations finished" << std::endl;
        }

        std::vector<std::pair<int, int>> odo_edges;
        for (size_t i = 1; i < worker_data.poses.size(); i++)
        {
            odo_edges.emplace_back(i - 1, i);
        }

        std::vector<TaitBryanPose> poses;
        std::vector<TaitBryanPose> poses_desired;

        for (size_t i = 0; i < worker_data.poses.size(); i++)
        {
            poses.push_back(pose_tait_bryan_from_affine_matrix(worker_data.poses[i]));
        }
        for (size_t i = 0; i < worker_data.poses.size(); i++)
        {
            poses_desired.push_back(pose_tait_bryan_from_affine_matrix(worker_data.poses[i]));
        }

        for (size_t i = 0; i < odo_edges.size(); i++)
        {
            /*Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
            relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                                              poses_desired[odo_edges[i].first].px,
                                              poses_desired[odo_edges[i].first].py,
                                              poses_desired[odo_edges[i].first].pz,
                                              poses_desired[odo_edges[i].first].om,
                                              poses_desired[odo_edges[i].first].fi,
                                              poses_desired[odo_edges[i].first].ka,
                                              poses_desired[odo_edges[i].second].px,
                                              poses_desired[odo_edges[i].second].py,
                                              poses_desired[odo_edges[i].second].pz,
                                              poses_desired[odo_edges[i].second].om,
                                              poses_desired[odo_edges[i].second].fi,
                                              poses_desired[odo_edges[i].second].ka);*/

            Eigen::Matrix<double, 6, 1> delta;
            relative_pose_obs_eq_tait_bryan_wc_case1(
                delta,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka,
                0, 0, 0, 0, 0, 0);
            // relative_pose_measurement_odo(0, 0),
            // relative_pose_measurement_odo(1, 0),
            // relative_pose_measurement_odo(2, 0),
            // relative_pose_measurement_odo(3, 0),
            // relative_pose_measurement_odo(4, 0),
            // relative_pose_measurement_odo(5, 0));

            Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
            relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                              poses[odo_edges[i].first].px,
                                                              poses[odo_edges[i].first].py,
                                                              poses[odo_edges[i].first].pz,
                                                              poses[odo_edges[i].first].om,
                                                              poses[odo_edges[i].first].fi,
                                                              poses[odo_edges[i].first].ka,
                                                              poses[odo_edges[i].second].px,
                                                              poses[odo_edges[i].second].py,
                                                              poses[odo_edges[i].second].pz,
                                                              poses[odo_edges[i].second].om,
                                                              poses[odo_edges[i].second].fi,
                                                              poses[odo_edges[i].second].ka);

            int ir = tripletListB.size();

            int ic_1 = odo_edges[i].first * 6;
            int ic_2 = odo_edges[i].second * 6;

            for (size_t row = 0; row < 6; row++)
            {
                tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
                tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
                tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
                tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
                tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
                tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

                tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
                tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
                tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
                tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
                tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
                tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
            }

            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
            tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
            tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
            tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
            tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

            tripletListP.emplace_back(ir, ir, 1000000);
            tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
            tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
            tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
            tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
            tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
        }

        int ic = 0;
        int ir = tripletListB.size();
        tripletListA.emplace_back(ir, ic * 6 + 0, 1);
        tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
        tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
        tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
        tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
        tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);
        tripletListP.emplace_back(ir + 2, ir + 2, 1);
        tripletListP.emplace_back(ir + 3, ir + 3, 1);
        tripletListP.emplace_back(ir + 4, ir + 4, 1);
        tripletListP.emplace_back(ir + 5, ir + 5, 1);

        tripletListB.emplace_back(ir, 0, 0);
        tripletListB.emplace_back(ir + 1, 0, 0);
        tripletListB.emplace_back(ir + 2, 0, 0);
        tripletListB.emplace_back(ir + 3, 0, 0);
        tripletListB.emplace_back(ir + 4, 0, 0);
        tripletListB.emplace_back(ir + 5, 0, 0);

        Eigen::SparseMatrix<double> matA(tripletListB.size(), worker_data.poses.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(worker_data.poses.size() * 6, worker_data.poses.size() * 6);
        Eigen::SparseMatrix<double> AtPB(worker_data.poses.size() * 6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        // AtPA += AtPAndt.sparseView();
        // AtPB += AtPBndt.sparseView();

        // Eigen::SparseMatrix<double> AtPA_I(intrinsics.size() * 6, intrinsics.size() * 6);
        // AtPA_I.setIdentity();
        // AtPA_I *= 1;
        // AtPA += AtPA_I;

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
            solver(AtPA);
        std::cout << "start solving" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);
        std::cout << "start finished" << std::endl;
        std::vector<double> h_x;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                // std::cout << it.value() << " ";
                h_x.push_back(it.value());
            }
            // std::cout << std::endl;
        }

        if (h_x.size() == 6 * worker_data.poses.size())
        {
            int counter = 0;

            for (size_t i = 0; i < worker_data.poses.size(); i++)
            {
                TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(worker_data.poses[i]);
                auto prev_pose = pose;
                pose.px += h_x[counter++];
                pose.py += h_x[counter++];
                pose.pz += h_x[counter++];
                pose.om += h_x[counter++];
                pose.fi += h_x[counter++];
                pose.ka += h_x[counter++];

                worker_data.poses[i] = affine_matrix_from_pose_tait_bryan(pose);
                all_data[index_rendered_points_local].poses[i] = worker_data.poses[i];
            }
        }
        else
        {
            std::cout << "optimization failed" << std::endl;
        }
    }

#endif
    return;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> get_nn()
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nn;
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // for (int i = 0; i < all_data[index_rendered_points_local].poses.size(); i++){
        //     all_data[index_rendered_points_local].poses[i](0, 3) += 1.0;
        // }

        auto worker_data = all_data[index_rendered_points_local];

        // index data
        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            auto lower = std::lower_bound(worker_data.timestamps.begin(), worker_data.timestamps.end(), worker_data.points_local[i].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });

            int index_pose = std::distance(worker_data.timestamps.begin(), lower);

            if (index_pose >= 0 && index_pose < worker_data.poses.size())
            {
                worker_data.points_local[i].index_pose = index_pose;
            }
            else
            {
                worker_data.points_local[i].index_pose = -1;
            }
        }

        NDT::GridParameters rgd_params;
        // rgd_params.resolution_X = 0.3; // distance bucket
        // rgd_params.resolution_Y = 0.3; // polar angle deg
        // rgd_params.resolution_Z = 0.3; // azimutal angle deg

        rgd_params.resolution_X = rgd_x; // distance bucket
        rgd_params.resolution_Y = rgd_y; // polar angle deg
        rgd_params.resolution_Z = rgd_z; // azimutal angle deg

        std::vector<Point3Di> point_cloud_global;
        std::vector<Point3Di> points_local;

        std::vector<Eigen::Vector3d> point_cloud_global_sc;
        std::vector<Point3Di> points_local_sc;

        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            double r_l = worker_data.points_local[i].point.norm();
            if (r_l > 0.5 && worker_data.points_local[i].index_pose != -1)
            {
                double polar_angle_deg_l = atan2(worker_data.points_local[i].point.y(), worker_data.points_local[i].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data.points_local[i].point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d pp = worker_data.points_local[i].point;
                // pps.x() = r;
                // pps.y() = polar_angle_deg;
                // pps.z() = azimutal_angle_deg;
                // point_cloud_spherical_coordinates.push_back(pps);

                Eigen::Affine3d pose = worker_data.poses[worker_data.points_local[i].index_pose];

                pp = pose * pp;

                Point3Di pg = worker_data.points_local[i];
                pg.point = pp;

                point_cloud_global.push_back(pg);
                points_local.push_back(worker_data.points_local[i]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data.points_local[i];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sc.push_back(p_sl);
                //
                double r_g = pg.point.norm();
                double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_g = acos(pg.point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d p_sg = worker_data.points_local[i].point;
                p_sg.x() = r_g;
                p_sg.y() = polar_angle_deg_g;
                p_sg.z() = azimutal_angle_deg_g;

                point_cloud_global_sc.push_back(p_sg);
            }
        }

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc, {0, 0, 0});

        /////////////
        // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nn;
        Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

        for (int i = 0; i < point_cloud_global_sc.size(); i++)
        {
            auto index_of_bucket = get_rgd_index(point_cloud_global_sc[i], b);

            auto bucket_it = buckets.find(index_of_bucket);

            if (bucket_it != buckets.end())
            {
                auto &this_bucket = bucket_it->second;
                this_bucket.number_of_points++;
                // const auto &curr_mean = points_global[i].point;
                const auto &mean = this_bucket.mean;

                nn.emplace_back(point_cloud_global[i].point, mean);

                //////////////
            }
        }
    }
    return nn;
}

void draw_ellipse(const Eigen::Matrix3d &covar, Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd = 3)
{

    Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di * 2 * pi;
    const double dv = dj * pi;
    glColor3f(color.x(), color.y(), color.z());

    for (double i = 0; i < 1.0; i += di) // horizonal
    {
        for (double j = 0; j < 1.0; j += dj) // vertical
        {
            double u = i * 2 * pi;     // 0     to  2pi
            double v = (j - 0.5) * pi; //-pi/2 to pi/2

            const Eigen::Vector3d pp0(cos(v) * cos(u), cos(v) * sin(u), sin(v));
            const Eigen::Vector3d pp1(cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v));
            const Eigen::Vector3d pp2(cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv));
            const Eigen::Vector3d pp3(cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv));
            Eigen::Vector3d tp0 = transform * (nstd * pp0) + mean;
            Eigen::Vector3d tp1 = transform * (nstd * pp1) + mean;
            Eigen::Vector3d tp2 = transform * (nstd * pp2) + mean;
            Eigen::Vector3d tp3 = transform * (nstd * pp3) + mean;

            glBegin(GL_LINE_LOOP);
            glVertex3dv(tp0.data());
            glVertex3dv(tp1.data());
            glVertex3dv(tp2.data());
            glVertex3dv(tp3.data());
            glEnd();
        }
    }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> get_mean_cov()
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> mc;
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // for (int i = 0; i < all_data[index_rendered_points_local].poses.size(); i++){
        //     all_data[index_rendered_points_local].poses[i](0, 3) += 1.0;
        // }

        auto worker_data = all_data[index_rendered_points_local];

        // index data
        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            auto lower = std::lower_bound(worker_data.timestamps.begin(), worker_data.timestamps.end(), worker_data.points_local[i].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });

            int index_pose = std::distance(worker_data.timestamps.begin(), lower);

            if (index_pose >= 0 && index_pose < worker_data.poses.size())
            {
                worker_data.points_local[i].index_pose = index_pose;
            }
            else
            {
                worker_data.points_local[i].index_pose = -1;
            }
        }

        NDT::GridParameters rgd_params;
        // rgd_params.resolution_X = 0.3; // distance bucket
        // rgd_params.resolution_Y = 0.3; // polar angle deg
        // rgd_params.resolution_Z = 0.3; // azimutal angle deg

        rgd_params.resolution_X = rgd_x; // distance bucket
        rgd_params.resolution_Y = rgd_y; // polar angle deg
        rgd_params.resolution_Z = rgd_z; // azimutal angle deg

        std::vector<Point3Di> point_cloud_global;
        std::vector<Point3Di> points_local;

        std::vector<Eigen::Vector3d> point_cloud_global_sc;
        std::vector<Point3Di> points_local_sc;

        for (int i = 0; i < worker_data.points_local.size(); i++)
        {
            double r_l = worker_data.points_local[i].point.norm();
            if (r_l > 0.5 && worker_data.points_local[i].index_pose != -1)
            {
                double polar_angle_deg_l = atan2(worker_data.points_local[i].point.y(), worker_data.points_local[i].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data.points_local[i].point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d pp = worker_data.points_local[i].point;
                // pps.x() = r;
                // pps.y() = polar_angle_deg;
                // pps.z() = azimutal_angle_deg;
                // point_cloud_spherical_coordinates.push_back(pps);

                Eigen::Affine3d pose = worker_data.poses[worker_data.points_local[i].index_pose];

                pp = pose * pp;

                Point3Di pg = worker_data.points_local[i];
                pg.point = pp;

                point_cloud_global.push_back(pg);
                points_local.push_back(worker_data.points_local[i]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data.points_local[i];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sc.push_back(p_sl);
                //
                double r_g = pg.point.norm();
                double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_g = acos(pg.point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d p_sg = worker_data.points_local[i].point;
                p_sg.x() = r_g;
                p_sg.y() = polar_angle_deg_g;
                p_sg.z() = azimutal_angle_deg_g;

                point_cloud_global_sc.push_back(p_sg);
            }
        }

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc, {0, 0, 0});

        /////////////
        Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

        for (const auto &b : buckets)
        {
            auto &this_bucket = b.second;

            mc.emplace_back(this_bucket.mean, this_bucket.cov);
        }
    }

    return mc;
}