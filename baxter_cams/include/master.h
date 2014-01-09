#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "baxter_cams/transMatrix.h"

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

baxter::BaxterMove              *robotBaxter;                   // Use MoveIt's IK solver to get to the desired position
baxter::BaxterStateControl*     state_control_baxter;           // This is to control the robot states like enable, disable
baxter::BaxterJointControl*     joint_control_baxter;           // This is to control each of the joints (like asking a particular joint to move to a desired angle)
bool                            g_shutdown;                     // Global shutdown variable set to true when the program receives a SIGINT signal
tf::TransformListener*          transform_listener;             // Transform listener to get the desired transform from the current transform tree

void shutdown(int sig);
void threadFunc();
void setBoxCoords(void);
