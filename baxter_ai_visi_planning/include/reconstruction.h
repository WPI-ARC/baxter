#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "baxter_cams1/transMatrix.h"

#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/joint_state_group.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "vector"
//#include "Eigen/Dense"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"

#include "baxter_move.h"
#include "boost/thread.hpp"

#include "signal.h"
#include "exception"

std::vector<cv::Point2f>        g_box_coordinates;
std::vector<cv::Point3f>        g_box;
int                             coordinates_entered;
int                             g_shutdown;
baxter::BaxterCam*              cam_baxter;

void shutdown(int sig);
void threadFunc();

void getCorners(cv::Mat &gray_image, std::vector<cv::Point2f> &corners);
void mouseCallback(int event, int x, int y, int flags, void* param);
void setBoxCoords(void);
