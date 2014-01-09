#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "baxter_cams1/transMatrix.h"

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_operations.h"
#include "shape_tools/shape_to_marker.h"
#include "shape_msgs/Mesh.h"
#include "shape_msgs/MeshTriangle.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/CollisionObject.h"
#include "visualization_msgs/Marker.h"

#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/joint_state_group.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/planning_scene/planning_scene.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "vector"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"

#include "eigen3/Eigen/Eigenvalues"

#include "baxter_move.h"
#include "geometry_functions.h"
#include "boost/thread.hpp"

#include "signal.h"
#include "exception"
#include "ctime"
#include "iostream"

int                             g_shutdown;
tf::TransformListener*          transform_listener;
baxter::BaxterMove              *robotBaxter;
std::vector<cv::Point3f>        g_object_pts_wrt_cam(100);
int                             g_number_of_object_pts = 0;

void shutdown(int sig);
double cost_function(int number_of_intersecting_points, int number_of_overlapping_pts, cv::Point3f object_wrt_base, cv::Point3f cam_wrt_base, robot_state::JointStateGroup *joint_state_group, int coi);
void random_sampling();
void reconstruct_probable_object_space();
void compute_field_of_view(cv::Mat camera_matrix, double &fovx, double &fovy, double &focal_length);
int checkVertexVisibility(std::vector<Triangle> &triangles1, std::vector<Triangle> &fov_triangles, cv::Point3f &box_pt,
                          cv::Point3f &cam_pt, std::vector<Triangle> &triangles2, std::vector<Triangle> &triangles3, std::vector<Triangle> &triangles4);
int rayTracingCollisions(std::vector<Ray> &r, std::vector<Triangle> &triangles1, std::vector<Triangle> &triangles2, std::vector<Triangle> &triangles3);
int rayTracing(std::vector<Ray> &r, std::vector<Triangle> &triangles);
void printTriangle(Triangle T);
cv::Mat getTransform(geometry_msgs::PoseStamped rand_pose);
Point getTransformedPoint(cv::Mat rot_matrix, Point pt);
void constructObstacleTriangles(std::vector<Triangle> &triangles, shapes::Mesh *obstacle_mesh, geometry_msgs::Pose &obstacle_pose);
void constructCameraViewTriangles(std::vector<Ray> &r, std::vector<Triangle> &triangles, std::vector<cv::Point3f> box_pts_wrt_base,
                                  geometry_msgs::PoseStamped cam_pt_wrt_base, cv::Point3f fov_point, double fovx, double fovy, ros::Publisher &marker_pub);
void trans_mat_callback(visualization_msgs::Marker msg);
