#include "ros/ros.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/joint_state_group.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "baxter_msgs/AssemblyState.h"
#include "baxter_msgs/JointPositions.h"
#include "baxter_msgs/JointCommandMode.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "boost/thread.hpp"

// openCV includes
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "Eigen/Dense"
#include <vector>
#include <exception>

#define LEFT_ARM    1
#define RIGHT_ARM   2
#define BOTH_ARMS   3

#define CAMERA_1            0
#define CAMERA_2            1
#define CAMERA_SUB_FULL     2
#define CAMERA_SUB_ZERO     5

namespace baxter_camera
{
    const int left_hand_camera  = 0;
    const int right_hand_camera = 1;
    const int head_camera       = 2;

    const std::string camera_topic[3] = { "/cameras/left_hand_camera/image", "/cameras/right_hand_camera/image", "/cameras/head_camera/image" };

    class intrinsic_matrix
    {
        // Published by the topic /cameras/right_hand_camera/camera_info_std
        // Published by the topic /cameras/left_hand_camera/camera_info_std
        // Published by the topic /cameras/head_camera/camera_info_std
        public:
            static const double right_hand_camera[];
            static const double left_hand_camera[];
            static const double head_camera[];
    };

    class distortion_coeffs
    {
        public:
            static const double right_hand_camera[];
            static const double left_hand_camera[];
            static const double head_camera[];
    };
}

namespace baxter_links
{
    const int base                          = 0;
    const int collision_head_link_1         = 1;
    const int collision_head_link_2         = 2;
    const int torso                         = 3;
    const int head                          = 4;
    const int dummyhead1                    = 5;
    const int head_camera                   = 6;
    const int screen                        = 7;
    const int left_arm_mount                = 8;
    const int left_upper_shoulder           = 9;
    const int left_lower_shoulder           = 10;
    const int left_upper_elbow              = 11;
    const int left_lower_elbow              = 12;
    const int left_upper_forearm            = 13;
    const int left_arm_itb                  = 14;
    const int left_lower_forearm            = 15;
    const int left_wrist                    = 16;
    const int left_hand                     = 17;
    const int left_gripper                  = 18;
    const int left_hand_camera              = 19;
    const int left_hand_camera_axis         = 20;
    const int left_hand_range               = 21;
    const int left_upper_forearm_visual     = 22;
    const int left_upper_elbow_visual       = 23;
    const int left_torso_itb                = 24;
    const int pedestal                      = 25;
    const int right_arm_mount               = 26;
    const int right_upper_shoulder          = 27;
    const int right_lower_shoulder          = 28;
    const int right_upper_elbow             = 29;
    const int right_lower_elbow             = 30;
    const int right_upper_forearm           = 31;
    const int right_arm_itb                 = 32;
    const int right_lower_forearm           = 33;
    const int right_wrist                   = 34;
    const int right_hand                    = 35;
    const int right_gripper                 = 36;
    const int right_hand_camera             = 37;
    const int right_hand_camera_axis        = 38;
    const int right_hand_range              = 39;
    const int right_upper_forearm_visual    = 40;
    const int right_upper_elbow_visual      = 41;
    const int right_torso_itb               = 42;
    const int sonar_ring                    = 43;

    const std::string link_names[44] = { "/base", "/collision_head_link_1", "/collision_head_link_2", "/torso", "/head", "/dummyhead1", "/head_camera", "/screen", "/left_arm_mount",
                                         "/left_upper_shoulder", "/left_lower_shoulder", "/left_upper_elbow", "/left_lower_elbow", "/left_upper_forearm", "/left_arm_itb", "/left_lower_forearm",
                                         "/left_wrist", "/left_hand", "/left_gripper", "/left_hand_camera", "/left_hand_camera_axis", "/left_hand_range", "/left_upper_forearm_visual",
                                         "/left_upper_elbow_visual", "/left_torso_itb", "/pedestal", "/right_arm_mount", "/right_upper_shoulder", "/right_lower_shoulder", "/right_upper_elbow",
                                         "/right_lower_elbow", "/right_upper_forearm", "/right_arm_itb", "/right_lower_forearm", "/right_wrist", "/right_hand", "/right_gripper",
                                         "/right_hand_camera", "/right_hand_camera_axis", "/right_hand_range", "/right_upper_forearm_visual", "/right_upper_elbow_visual", "/right_torso_itb",
                                         "/sonar_ring" };
}

namespace baxter_joints
{
    const int head_nod                      = 0;
    const int head_pan                      = 1;
    const int left_e0                       = 2;
    const int left_e1                       = 3;
    const int left_s0                       = 4;
    const int left_s1                       = 5;
    const int left_w0                       = 6;
    const int left_w1                       = 7;
    const int left_w2                       = 8;
    const int right_e0                      = 9;
    const int right_e1                      = 10;
    const int right_s0                      = 11;
    const int right_s1                      = 12;
    const int right_w0                      = 13;
    const int right_w1                      = 14;
    const int right_w2                      = 15;
    const int torso_t0                      = 16;

    const std::string joint_names[17] = {"head_nod", "head_pan", "left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2", "right_e0", "right_e1", "right_s0",
                                         "right_s1", "right_w0", "right_w1", "right_w2", "torso_t0"};
}

namespace baxter_joint_command_mode
{
    const int POSITION                      = 1;
    const int VELOCITY                      = 2;
    const int TORQUE                        = 3;
}

namespace math_functions
{
    double absolute(double X);
    int absolute(int X);
}

namespace baxter
{
    // MoveIt Control
    class BaxterMove
    {
        // Public variables
        public:
            ros::Subscriber                         joint_state_subscriber;
            static sensor_msgs::JointState          joint_state_msg;
            move_group_interface::MoveGroup         *right_arm, *left_arm;
            geometry_msgs::PoseStamped              currentPos_wrt_baseframe;
            tf::TransformListener*                  transform_listener;
            robot_model_loader::RobotModelLoader*   robot_model_loader;
            robot_model::RobotModelPtr              kinematic_model;
            robot_state::RobotStatePtr              kinematic_state;
            robot_state::JointStateGroup*           right_arm_joint_state_group;
            robot_state::JointStateGroup*           left_arm_joint_state_group;

            BaxterMove();
            ~BaxterMove();

            static void getJointStates(sensor_msgs::JointState jsm);
            geometry_msgs::PoseStamped getCurrentPose(int link_index);
            bool checkIKexists(geometry_msgs::PoseStamped ik_check_point_pose, int arm_group);
            void moveToTarget(geometry_msgs::PoseStamped target, int arm);
            bool subscribeJointStates(ros::NodeHandle current_node_handle);
    };

    // Individual joint angle control
    class BaxterJointControl
    {
        public:
            ros::Subscriber                         joint_state_subscriber;
            static sensor_msgs::JointState          joint_state_msg;

            BaxterJointControl();

            static void getJointStates(sensor_msgs::JointState jsm);
            bool subscribeJointStates(ros::NodeHandle current_node_handle);
            bool moveJoint(std::vector<int> names, std::vector<double> angle, ros::NodeHandle nHandle);
            bool setJointCommandMode(int mode, ros::NodeHandle nHandle);
    };

    // Enable/Disable/Reset Robot
    class BaxterStateControl
    {
        public:
            // New functions added today
            static baxter_msgs::AssemblyState       assembly_state_msg;
            ros::Subscriber                         assembly_state_subscriber;

            static void getAssemblyState(baxter_msgs::AssemblyState bax_asm);
            bool subscribeAssemblyState(ros::NodeHandle current_node_handle);
            baxter_msgs::AssemblyState enableRobot(ros::NodeHandle current_node_handle);
            baxter_msgs::AssemblyState disableRobot(ros::NodeHandle current_node_handle);
            baxter_msgs::AssemblyState resetRobot(ros::NodeHandle current_node_handle);
    };

    // Camera control
    class BaxterCam
    {
        public:
            ros::Subscriber                         camera1_subscriber, camera2_subscriber;
            static sensor_msgs::Image               camera1_image, camera2_image;

            BaxterCam();
            ~BaxterCam();

            static void getCameraImage_1(sensor_msgs::Image image_1);
            static void getCameraImage_2(sensor_msgs::Image image_2);
            cv_bridge::CvImagePtr readImageFromCamera(int cameraIndex, cv_bridge::CvImagePtr cv_camera_image);
            bool subscribeCameraImage(int cameraIndex, ros::NodeHandle current_node_handle, int replace_camera_index = baxter_camera::head_camera, bool replace_if_full = true);

        private:
            char                                    cameras_subscribed;
            int                                     camera1, camera2;

            bool subscribeCameraImage_1(int cameraIndex, ros::NodeHandle current_node_handle);
            bool subscribeCameraImage_2(int cameraIndex, ros::NodeHandle current_node_handle);
    };
}

// ######################################################################################################################
//                                      BOOST SHARED POINTER CLASS NULL POINTER DEFINITION
// ######################################################################################################################

// Defining the NULL pointer for the boost::shared_ptr class
namespace pointer
{
    class boostPointer
    {
        public:
            template<typename T>
            operator boost::shared_ptr<T>() { return boost::shared_ptr<T>(); }
    } nullPtr;
}
