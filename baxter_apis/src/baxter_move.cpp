#include "baxter_move.h"

sensor_msgs::JointState                 baxter::BaxterMove::joint_state_msg;
baxter_msgs::AssemblyState              baxter::BaxterStateControl::assembly_state_msg;
sensor_msgs::Image                      baxter::BaxterCam::camera1_image, baxter::BaxterCam::camera2_image;
sensor_msgs::JointState                 baxter::BaxterJointControl::joint_state_msg;

baxter::BaxterMove::BaxterMove()
{
    
    this->right_arm = new move_group_interface::MoveGroup("right_arm");
    
    this->left_arm  = new move_group_interface::MoveGroup("left_arm");
    
    this->transform_listener = new tf::TransformListener();
    
    this->robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
    
    this->kinematic_model = robot_model_loader->getModel();
    
    this->kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(this->kinematic_model));
    
    this->left_arm_joint_state_group = this->kinematic_state->getJointStateGroup("left_arm");
    
    this->right_arm_joint_state_group = this->kinematic_state->getJointStateGroup("right_arm");
    
}

void baxter::BaxterMove::getJointStates(sensor_msgs::JointState jsm)
{
    baxter::BaxterMove::joint_state_msg = jsm;
}

bool baxter::BaxterMove::checkIKexists(geometry_msgs::PoseStamped ik_check_point_pose, int arm_group)
{
    geometry_msgs::Pose tPos = ik_check_point_pose.pose;

    if (arm_group == LEFT_ARM)
        return(this->left_arm_joint_state_group->setFromIK(tPos, 5, 0.0, kinematics::KinematicsQueryOptions()));
    else if (arm_group == RIGHT_ARM)
        return(right_arm_joint_state_group->setFromIK(tPos, 5, 0.0, kinematics::KinematicsQueryOptions()));
    else
        ROS_INFO("ERROR: Wrong identifier!");

    return false;
}

geometry_msgs::PoseStamped baxter::BaxterMove::getCurrentPose(int link_index)
{
    geometry_msgs::PoseStamped tPos_base, tPos_link;

    tPos_base.header.frame_id = "/reference" + baxter_links::link_names[link_index];
    tPos_base.pose.position.x = 0.0;
    tPos_base.pose.position.y = 0.0;
    tPos_base.pose.position.z = 0.0;
    tPos_base.pose.orientation.w = 1.0;
    tPos_base.pose.orientation.x = 0.0;
    tPos_base.pose.orientation.y = 0.0;
    tPos_base.pose.orientation.z = 0.0;

    transform_listener->waitForTransform(baxter_links::link_names[baxter_links::base], "/reference" + baxter_links::link_names[link_index], ros::Time::now(), ros::Duration(1.0));
    transform_listener->transformPose("/reference/base", tPos_base, tPos_link);

    return tPos_link;
}

void baxter::BaxterMove::moveToTarget(geometry_msgs::PoseStamped target, int arm)
{
    if (arm == RIGHT_ARM)
    {
        this->right_arm->setPoseTarget(target);
        this->right_arm->move();
    }
    else if (arm == LEFT_ARM)
    {
        this->left_arm->setPoseTarget(target);
        this->left_arm->move();
    }
    else
        ROS_INFO("ERROR: Wrong arm identifier!");
}

bool baxter::BaxterMove::subscribeJointStates(ros::NodeHandle current_node_handle)
{
    this->joint_state_subscriber = current_node_handle.subscribe("/robot/joint_states", 50, this->getJointStates);
}

// **********************************************************************************************************************
//                                           BaxterJointControl CLASS Definitions
// **********************************************************************************************************************

baxter::BaxterJointControl::BaxterJointControl()
{}

bool baxter::BaxterJointControl::subscribeJointStates(ros::NodeHandle current_node_handle)
{
    this->joint_state_subscriber = current_node_handle.subscribe("/robot/joint_states", 50, this->getJointStates);
}

void baxter::BaxterJointControl::getJointStates(sensor_msgs::JointState jsm)
{
    baxter::BaxterJointControl::joint_state_msg = jsm;
}

bool baxter::BaxterJointControl::moveJoint(std::vector<int> names, std::vector<double> angle, ros::NodeHandle nHandle)
{
    try
    {
        int i;
        double last_angle;
        ros::Publisher move_joint_pub = nHandle.advertise<baxter_msgs::JointPositions>("/robot/limb/right/command_joint_angles", 0, true);
        baxter_msgs::JointPositions msg;

        while(1)
            if (baxter::BaxterJointControl::joint_state_msg.position.size() == 0)
                continue;
            else
                break;

        for (i = 0; i < names.size(); i++)
        {
            msg.angles.clear();
            msg.names.clear();
            msg.angles.push_back(angle[i] + baxter::BaxterJointControl::joint_state_msg.position[names[i]]);
            msg.names.push_back(baxter_joints::joint_names[names[i]]);
            //ROS_INFO("Name - %s, Initial val - %f, Final value - %f", baxter_joints::joint_names[names[i]], baxter::BaxterJointControl::joint_state_msg.position[names[i]],
            //         msg.angles.at(i));
            //ROS_INFO_STREAM("Name - " << baxter_joints::joint_names[names[i]] << " Initial val - " << baxter::BaxterJointControl::joint_state_msg.position[names[i]] <<
            //                " Final val - " << msg.angles.at(i) << " delta - " << angle[i]);
            //ROS_INFO("%s - %f", msg.names[0].c_str(), msg.angles[0]);

            ros::Rate loop_rate(100);
            int j = 0;

            while(1)
            {
                double cur_angle;

                if (j == 100)
                    break;

                if (baxter::BaxterJointControl::joint_state_msg.position.size() == 0)
                    continue;
                cur_angle = baxter::BaxterJointControl::joint_state_msg.position[names[i]];
                //ROS_INFO("%f, Cur - %f, Last - %f", baxter::BaxterJointControl::joint_state_msg.position[names[i]], cur_angle - msg.angles[0], msg.angles[0]);
                move_joint_pub.publish(msg);
                if (math_functions::absolute(cur_angle - msg.angles[0]) < 0.1)
                    j++;
                loop_rate.sleep();
            }

        }

        return true;
    }
    catch (std::exception ex)
    {
        return false;
    }
}

bool baxter::BaxterJointControl::setJointCommandMode(int mode, ros::NodeHandle nHandle)
{
    try
    {
        int i;
        ros::Publisher command_joint_pub = nHandle.advertise<baxter_msgs::JointCommandMode>("/robot/limb/right/joint_command_mode", 0, true);
        baxter_msgs::JointCommandMode msg;

        msg.mode = baxter_joint_command_mode::POSITION;

        ros::Rate loop_rate(1);
        for (i = 0; i < 2; i++)
        {
            command_joint_pub.publish(msg);
            loop_rate.sleep();
        }
    }
    catch (std::exception ex)
    {
        return false;
    }
}

// **********************************************************************************************************************
//                                           BaxterStateControl CLASS Definitions
// **********************************************************************************************************************

void baxter::BaxterStateControl::getAssemblyState(baxter_msgs::AssemblyState bax_asm)
{
    baxter::BaxterStateControl::assembly_state_msg = bax_asm;
}

bool baxter::BaxterStateControl::subscribeAssemblyState(ros::NodeHandle current_node_handle)
{
    this->assembly_state_subscriber = current_node_handle.subscribe("/sdk/robot/state", 10, this->getAssemblyState);
}

baxter_msgs::AssemblyState baxter::BaxterStateControl::enableRobot(ros::NodeHandle current_node_handle)
{
    int i;
    ros::Publisher enable_pub = current_node_handle.advertise<std_msgs::Bool>("/robot/set_super_enable", 0, true);
    std_msgs::Bool msg;

    ros::Rate loop_rate(4);
    for (i = 0; i < 10; i++)
    {
        msg.data = true;
        enable_pub.publish(msg);
        loop_rate.sleep();
        if (baxter::BaxterStateControl::assembly_state_msg.enabled == true)
            break;
    }

    return baxter::BaxterStateControl::assembly_state_msg;
}

baxter_msgs::AssemblyState baxter::BaxterStateControl::disableRobot(ros::NodeHandle current_node_handle)
{
    int i;
    ros::Publisher enable_pub = current_node_handle.advertise<std_msgs::Bool>("/robot/set_super_enable", 0, true);
    std_msgs::Bool msg;

    ros::Rate loop_rate(4);
    for (i = 0; i < 10; i++)
    {
        msg.data = false;
        enable_pub.publish(msg);
        loop_rate.sleep();
        if (baxter::BaxterStateControl::assembly_state_msg.enabled == false)
            break;
    }

    return baxter::BaxterStateControl::assembly_state_msg;
}

baxter_msgs::AssemblyState baxter::BaxterStateControl::resetRobot(ros::NodeHandle current_node_handle)
{
    int i;
    ros::Publisher enable_pub = current_node_handle.advertise<std_msgs::Bool>("/robot/set_super_reset", 0, true);
    std_msgs::Bool msg;

    ros::Rate loop_rate(4);
    for (i = 0; i < 10; i++)
    {
        msg.data = true;
        enable_pub.publish(msg);
        loop_rate.sleep();
    }

    return baxter::BaxterStateControl::assembly_state_msg;
}

// **********************************************************************************************************************
//                                              BaxterCam CLASS Definitions
// **********************************************************************************************************************

const double baxter_camera::intrinsic_matrix::right_hand_camera[] = {395.36091716822295f, 0.0f, 582.3449018156452f, 0.0f, 396.70598889677996f, 404.4833884355011f, 0.0f, 0.0f, 1.0f};
const double baxter_camera::intrinsic_matrix::left_hand_camera[] = {};
const double baxter_camera::intrinsic_matrix::head_camera[] = {};

const double baxter_camera::distortion_coeffs::right_hand_camera[] = {0.008832567434430496, -0.012992061388991628, -0.003303704587738239, -0.0032571418720307188, -0.035931164201818674};
const double baxter_camera::distortion_coeffs::left_hand_camera[] = {};
const double baxter_camera::distortion_coeffs::head_camera[] = {};

baxter::BaxterCam::BaxterCam()
{
    this->camera1 = CAMERA_SUB_ZERO;
    this->camera2 = CAMERA_SUB_ZERO;
    this->cameras_subscribed = 0;
}

bool baxter::BaxterCam::subscribeCameraImage_1(int cameraIndex, ros::NodeHandle current_node_handle)
{
    this->camera1_subscriber = current_node_handle.subscribe(baxter_camera::camera_topic[cameraIndex], 1, this->getCameraImage_1);
    this->camera1 = cameraIndex;
    return true;
}

bool baxter::BaxterCam::subscribeCameraImage_2(int cameraIndex, ros::NodeHandle current_node_handle)
{
    this->camera2_subscriber = current_node_handle.subscribe(baxter_camera::camera_topic[cameraIndex], 5, this->getCameraImage_2);
    this->camera2 = cameraIndex;
    return true;
}

bool baxter::BaxterCam::subscribeCameraImage(int cameraIndex, ros::NodeHandle current_node_handle, int replace_camera_index, bool replace_if_full)
{
    bool returnValue;

    if ((this->camera1 == cameraIndex) || (this->camera2 == cameraIndex))
        return true;

    if (replace_if_full == false)
    {
        if (this->cameras_subscribed == CAMERA_SUB_FULL)
            return false;

        if (!((this->camera1 == baxter_camera::head_camera) || (this->camera1 == baxter_camera::left_hand_camera) || (this->camera1 == baxter_camera::right_hand_camera)))
            returnValue = this->subscribeCameraImage_1(cameraIndex, current_node_handle);
        else if (!((this->camera2 == baxter_camera::head_camera) || (this->camera2 == baxter_camera::left_hand_camera) || (this->camera2 == baxter_camera::right_hand_camera)))
            returnValue = this->subscribeCameraImage_1(cameraIndex, current_node_handle);
    }
    else
    {
        if (this->cameras_subscribed == CAMERA_SUB_FULL)
        {
            if (this->camera1 == replace_camera_index)
                returnValue = this->subscribeCameraImage_1(cameraIndex, current_node_handle);
            else if (this->camera2 == replace_camera_index)
                returnValue = this->subscribeCameraImage_2(cameraIndex, current_node_handle);
            else
                return false;
        }
        else
        {
            if (!((this->camera1 == baxter_camera::head_camera) || (this->camera1 == baxter_camera::left_hand_camera) || (this->camera1 == baxter_camera::right_hand_camera)))
                returnValue = this->subscribeCameraImage_1(cameraIndex, current_node_handle);
            else if (!((this->camera2 == baxter_camera::head_camera) || (this->camera2 == baxter_camera::left_hand_camera) || (this->camera2 == baxter_camera::right_hand_camera)))
                returnValue = this->subscribeCameraImage_2(cameraIndex, current_node_handle);
        }
    }

    if ((cameras_subscribed < 2) && (returnValue == true))
        cameras_subscribed++;

    return returnValue;
}

cv_bridge::CvImagePtr baxter::BaxterCam::readImageFromCamera(int cameraIndex, cv_bridge::CvImagePtr cv_camera_image)
{
    sensor_msgs::Image      raw_camera_image;

    cv_camera_image.reset();

    // Get the raw_camera_image from the corresponding camera
    if (this->camera1 == cameraIndex)
        raw_camera_image = this->camera1_image;
    else if (this->camera2 == cameraIndex)
        raw_camera_image = this->camera2_image;
    else
        return pointer::nullPtr;

    try
    {
        // Convert raw_camera_image to cv_camera_image
        cv_camera_image = cv_bridge::toCvCopy(raw_camera_image, raw_camera_image.encoding);
    }
    catch(std::exception& ex)
    {
        return pointer::nullPtr;
    }

    return cv_camera_image;
}

void baxter::BaxterCam::getCameraImage_1(sensor_msgs::Image image_1)
{
    static int i = 0;

    i++;

    if (i < 3)
    {
        ROS_INFO("Height - %d", image_1.height);
        ROS_INFO("Width - %d", image_1.width);
        ROS_INFO("Encoding - %s", image_1.encoding.c_str());
    }
    //cv_bridge::CvImagePtr   cv_camera_image;
    //sensor_msgs::Image      raw_camera_image;

    baxter::BaxterCam::camera1_image = image_1;

    //raw_camera_image = baxter::BaxterMove::camera1_image;
    //cv_camera_image = cv_bridge::toCvCopy(raw_camera_image, raw_camera_image.encoding);

    //cv::imshow("camera1", cv_camera_image->image);
    //cv::waitKey(30);
}

void baxter::BaxterCam::getCameraImage_2(sensor_msgs::Image image_2)
{
    baxter::BaxterCam::camera2_image = image_2;
}

double math_functions::absolute(double X)
{
    if (X < 0)
        return -X;
    else
        return X;
}

int math_functions::absolute(int X)
{
    if (X < 0)
        return -X;
    else
        return X;
}

// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
