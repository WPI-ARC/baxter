// Master node -- receive image from cams node and perform online estimation of the Jacobian
// Perform the Visual Sevoing tasks here like computation of the Jacobian and all of that
// By - Nandan Banerjee

#include "master.h"
#include "cmath"

std::vector<cv::Point3f>        g_box_coordinates;
cv::Mat                         g_transformation_matrix(4, 4, cv::DataType<double>::type);
bool                            g_get_transform;
robot_state::JointStateGroup*   g_joint_state_group;

void threadFunc()
{
    ros::Rate r(100);    // Nyquist Sampling frequency f >= 2 BW = 2 * frame_rate = 2 * 24 = 48
    while (ros::ok())
    {
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
    }
}

void shutdown(int sig)
{
    g_shutdown = true;
}

void trans_mat_callback(baxter_cams1::transMatrix msg)
{
    if (g_get_transform == true)
    {
        for (int k = 0; k < 4; k++)
            for (int l = 0; l < 4; l++)
                g_transformation_matrix.at<double>(k, l) = msg.tMat[k*4 + l];

        for (int k = 0; k < 4; k++)
            ROS_INFO("%10.6f       %10.6f       %10.6f       %10.6f",
                     g_transformation_matrix.at<double>(k, 0), g_transformation_matrix.at<double>(k, 1),
                     g_transformation_matrix.at<double>(k, 2), g_transformation_matrix.at<double>(k, 3));
        g_get_transform = false;
    }
}

// Set the box coordinates in the global variable g_box_coordinates
void setBoxCoords(void)
{
    g_box_coordinates.clear();

    g_box_coordinates.push_back(cv::Point3f(-0.256, 0.183, 0.0));         // Coords for AD
    g_box_coordinates.push_back(cv::Point3f(0.0, 0.183, 0.0));           // Coords for BD
    g_box_coordinates.push_back(cv::Point3f(-0.256, 0.0, 0.0));          // Coords for CD
    g_box_coordinates.push_back(cv::Point3f(0.0, 0.0, 0.0));            // Coords for DD
    g_box_coordinates.push_back(cv::Point3f(-0.256, 0.0, -0.095));         // Coords for ED
    g_box_coordinates.push_back(cv::Point3f(0.0, 0.0, -0.095));           // Coords for FD
}

// Check this thoroughly! Figure out the implications that it might have.
// Should not have a problem
void transform_coords(cv::Mat &transformation_matrix, cv::Point3f &point, cv::Point3f &transformed_point)
{
    transformed_point.x = (transformation_matrix.at<double>(0, 0) * point.x) + (transformation_matrix.at<double>(0, 1) * point.y) +
                          (transformation_matrix.at<double>(0, 2) * point.z) + (transformation_matrix.at<double>(0, 3));
    transformed_point.y = (transformation_matrix.at<double>(1, 0) * point.x) + (transformation_matrix.at<double>(1, 1) * point.y) +
                          (transformation_matrix.at<double>(1, 2) * point.z) + (transformation_matrix.at<double>(1, 3));
    transformed_point.z = (transformation_matrix.at<double>(2, 0) * point.x) + (transformation_matrix.at<double>(2, 1) * point.y) +
                          (transformation_matrix.at<double>(2, 2) * point.z) + (transformation_matrix.at<double>(2, 3));
}

// TODO: Should return the FK(theta) in the camera frame
// FK seems to be working fine.
cv::Point3f forward_kinematics(sensor_msgs::JointState &jsm, tf::TransformListener &transform_listener, sensor_msgs::JointState &start_state)
{
    cv::Mat                     translation_mat(4, 4, cv::DataType<double>::type);
    cv::Point3f                 camera_point, start_point;
    geometry_msgs::PoseStamped  tPos_base, tPos_camera;

    g_joint_state_group->setVariableValues(jsm);
    const Eigen::Affine3d &end_effector_state = g_joint_state_group->getRobotState()->getLinkState("right_hand_camera")->getGlobalLinkTransform();

    for (int cntr = 0; cntr < 4; cntr++)
    {
        translation_mat.at<double>(cntr, 0) = end_effector_state(cntr, 0);
        translation_mat.at<double>(cntr, 1) = end_effector_state(cntr, 1);
        translation_mat.at<double>(cntr, 2) = end_effector_state(cntr, 2);
        translation_mat.at<double>(cntr, 3) = end_effector_state(cntr, 3);
        ROS_INFO_STREAM("NANDAN BANERJEE: " << end_effector_state(cntr, 0) << "  " << end_effector_state(cntr, 1) << "  " <<  end_effector_state(cntr, 2) << "  " <<  end_effector_state(cntr, 3));
        ROS_INFO_STREAM("NANDAN BANERJEE: " << translation_mat.at<double>(cntr, 0) << "  " << translation_mat.at<double>(cntr, 1) << "  "
                        <<  translation_mat.at<double>(cntr, 2) << "  " <<  translation_mat.at<double>(cntr, 3));
    }

    camera_point.x = translation_mat.at<double>(0, 3);
    camera_point.y = translation_mat.at<double>(1, 3);
    camera_point.z = translation_mat.at<double>(2, 3);

    g_joint_state_group->setVariableValues(start_state);
    const Eigen::Affine3d &end_effector_state2 = g_joint_state_group->getRobotState()->getLinkState("right_hand_camera")->getGlobalLinkTransform();

    for (int cntr = 0; cntr < 4; cntr++)
    {
        translation_mat.at<double>(cntr, 0) = end_effector_state2(cntr, 0);
        translation_mat.at<double>(cntr, 1) = end_effector_state2(cntr, 1);
        translation_mat.at<double>(cntr, 2) = end_effector_state2(cntr, 2);
        translation_mat.at<double>(cntr, 3) = end_effector_state2(cntr, 3);
        ROS_INFO_STREAM("NANDAN BANERJEE: " << end_effector_state2(cntr, 0) << "  " << end_effector_state2(cntr, 1) << "  " <<  end_effector_state2(cntr, 2) << "  " <<  end_effector_state2(cntr, 3));
        ROS_INFO_STREAM("NANDAN BANERJEE: " << translation_mat.at<double>(cntr, 0) << "  " << translation_mat.at<double>(cntr, 1) << "  "
                        <<  translation_mat.at<double>(cntr, 2) << "  " <<  translation_mat.at<double>(cntr, 3));
    }

    start_point.x = translation_mat.at<double>(0, 3);
    start_point.y = translation_mat.at<double>(1, 3);
    start_point.z = translation_mat.at<double>(2, 3);

    camera_point.x = camera_point.x - start_point.x;
    camera_point.y = camera_point.y - start_point.y;
    camera_point.z = camera_point.z - start_point.z;

    ROS_INFO_STREAM("CAMERA POINT: " << camera_point.x << " " << camera_point.y << " " << camera_point.z);
    ROS_INFO_STREAM("CAMERA DISTANCE: " << (std::pow(camera_point.x, 2) + std::pow(camera_point.y, 2) + std::pow(camera_point.z, 2)));

    return camera_point;
}

// Should be working fine
double computeError(cv::Point3f &A, cv::Point3f &B, cv::Point3f &A_B)
{

    A_B.x = B.x - A.x;
    A_B.y = B.y - A.y;
    A_B.z = B.z - A.z;

    ROS_INFO_STREAM("ERROR: " << std::sqrt((std::pow(A.x - B.x, 2)) + (std::pow(A.y - B.y, 2)) + (std::pow(A.z - B.z, 2))));
    return std::sqrt((std::pow(A.x - B.x, 2)) + (std::pow(A.y - B.y, 2)) + (std::pow(A.z - B.z, 2)));
}

// TODO: Need to check
void computeJointAngles(cv::Mat pseudo_inverse_jacobian, std::vector<double> &joint_angles, cv::Point3f position)
{
    joint_angles.clear();
    for (int i = 0; i < 7; i++)
    {
        joint_angles.push_back(     (pseudo_inverse_jacobian.at<double>(i, 0) * position.x) +
                                    (pseudo_inverse_jacobian.at<double>(i, 1) * position.y) +
                                    (pseudo_inverse_jacobian.at<double>(i, 2) * position.z)     );
    }
}

// Does the iterative IK in the camera frame as the Jacobian was computed in the camera frame
bool iterative_ik_solve_and_move(cv::Point3f target_point, cv::Point3f target_orientation, sensor_msgs::JointState &start_state, cv::Mat pseudo_inverse_jacobian,
                                 baxter::BaxterJointControl &joint_control_baxter, ros::NodeHandle nHandle, tf::TransformListener &transform_listener)
{
    std::vector<double>         joint_angles[20];   // 20 is the max number of allowed iterations
    double                      epsilon = 0.01;     // epsilon is the minimum allowable error
    std::vector<cv::Point3f>    X;                  // start points at every iteration
    cv::Point3f                 error;              // Distance error that is required to be minimized
    std::vector<double>         temp_angles;
    sensor_msgs::JointState     jsm;
    int                         last_entry;

    for (int j = 0; j < 7; j++)
        joint_angles[0].push_back(start_state.position[baxter_joints::right_e0 + j]);

    jsm = start_state;

    // Iterative IK solver
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 7; j++)
            jsm.position[baxter_joints::right_e0 + j] = joint_angles[i].at(j);
        X.push_back(forward_kinematics(jsm, transform_listener, start_state));      // X[i] = FK(theta(i))
        if (computeError(X[i], target_point, error) < epsilon)                      // E    = X[target] - X[i]
        {
            break;                                                                  // If E < epsilon, then start moving
            last_entry = i;
        }
        temp_angles.clear();
        computeJointAngles(pseudo_inverse_jacobian, temp_angles, error);            // d_theta = J_inv * error

        // Print the temp_angles
        for (int j = 0; j < 7; j++)
            ROS_INFO_STREAM("Temp angle " << i << " - " << temp_angles[j]);

        for (int j = 0; j < 7; j++)
            temp_angles.at(j) = (joint_angles[i].at(j) + temp_angles.at(j));        // theta(i + 1) = theta(i) + d_theta
        joint_angles[i + 1] = temp_angles;
        last_entry = i;
    }

    // Move the robot arm according to the joint angles
    std::vector<double> d_theta;
    std::vector<int>    joint_id;

    for (int j = 0; j < 7; j++)
        ROS_INFO_STREAM("Joint " << j << " - " << joint_angles[last_entry].at(j));

    char c;
    std::cin >> c;

    for (int i = 1; i < 4; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            joint_id.clear();
            d_theta.clear();
            joint_id.push_back(baxter_joints::right_e0 + j);
            d_theta.push_back(joint_angles[i].at(j) - joint_angles[0].at(j));
            ROS_INFO("dtheta %d - %f", baxter_joints::right_e0 + j, joint_angles[i].at(j) - joint_angles[0].at(j));
            joint_control_baxter.moveJoint(joint_id, d_theta, nHandle);
        }
    }
}

// **********************************************************************************************************************
//                                                   Simulation code
// **********************************************************************************************************************

// Get the box coordinates wrt the camera frame from RViz
geometry_msgs::PoseStamped getBoxCoordsWrtCamFrame(int baxter_link, tf::TransformListener *transform)
{
    geometry_msgs::PoseStamped  box, tPos_camera, tPos_link;

    box.header.frame_id = "/reference/base";
    box.pose.position.x = 0.8;
    box.pose.position.y = 0.0;
    box.pose.position.z = (0.025 / 2) + (0.095 / 2) - 0.14;
    box.pose.orientation.x = 0.0;
    box.pose.orientation.y = 0.0;
    box.pose.orientation.z = 0.0;
    box.pose.orientation.w = 1.0;

    //transform->waitForTransform("/reference" + baxter_links::link_names[baxter_link], "/reference/base", ros::Time::now(), ros::Duration(1.0));
    //transform->transformPose("/reference" + baxter_links::link_names[baxter_link], box, tPos_link);

    tPos_link.header.frame_id = "/reference" + baxter_links::link_names[baxter_link];
    tPos_link.pose.position.x = 0.0;
    tPos_link.pose.position.y = 0.0;
    tPos_link.pose.position.z = 0.0;
    tPos_link.pose.orientation.x = 0.0;
    tPos_link.pose.orientation.y = 0.0;
    tPos_link.pose.orientation.z = 0.0;
    tPos_link.pose.orientation.w = 1.0;

    tf::StampedTransform stamped_t;
    transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_link], ros::Time(0), stamped_t);

    //transform->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_link], ros::Time::now(), ros::Duration(1.0));
    transform->transformPose("/reference/base", tPos_link, tPos_camera);

    box.pose.position.x = box.pose.position.x - tPos_camera.pose.position.x;
    box.pose.position.y = box.pose.position.y - tPos_camera.pose.position.y;
    box.pose.position.z = box.pose.position.z - tPos_camera.pose.position.z;

    //transform->waitForTransform("/reference" + baxter_links::link_names[baxter_link], "/reference/base", ros::Time::now(), ros::Duration(1.0));
    //transform_listener->transformPose("/reference" + baxter_links::link_names[baxter_link], box, tPos_camera);

    //transform->waitForTransform("/reference" + baxter_links::link_names[baxter_link], "/base", ros::Time::now(), ros::Duration(1.0));
    //transform->transformPose("/reference" + baxter_links::link_names[baxter_link], box, tPos_camera);

    return box;
}

geometry_msgs::PoseStamped getCamCoordsWrtBaseFrame(int camera_link, tf::TransformListener *transform)
{
    geometry_msgs::PoseStamped box, tPos_cam;

    box.header.frame_id = "/reference" + baxter_links::link_names[camera_link];
    box.pose.position.x = 0.0;
    box.pose.position.y = 0.0;
    box.pose.position.z = 0.0;
    box.pose.orientation.x = 0.0;
    box.pose.orientation.y = 0.0;
    box.pose.orientation.z = 0.0;
    box.pose.orientation.w = 1.0;

    tf::StampedTransform stamped_t;
    transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[camera_link], ros::Time(0), stamped_t);

    //transform->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[camera_link], ros::Time::now(), ros::Duration(1.0));
    transform->transformPose("/reference/base", box, tPos_cam);

    return tPos_cam;
}

// Convert the box coordinates wrt the camera frame in the base frame which is always static
geometry_msgs::PoseStamped getCoordsWrtBaseFrame(int baxter_link, tf::TransformListener *transform, cv::Point3f pt)
{
    geometry_msgs::PoseStamped tPos_camera, tPos_pt_base, tPos_pt;

    tPos_pt.header.frame_id = "/reference" + baxter_links::link_names[baxter_link];
    tPos_pt.pose.position.y = pt.x;
    tPos_pt.pose.position.x = pt.y;
    tPos_pt.pose.position.z = pt.z;
    tPos_pt.pose.orientation.x = 0.0;
    tPos_pt.pose.orientation.y = 0.0;
    tPos_pt.pose.orientation.z = 0.0;
    tPos_pt.pose.orientation.w = 1.0;

    tf::StampedTransform stamped_t;
    transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_link], ros::Time(0), stamped_t);

    transform->transformPose("/reference/base", tPos_pt, tPos_pt_base);

    return tPos_pt_base;
}

// Use this instead of the above function in places of the above function!
geometry_msgs::PoseStamped getCoordsWrtCamInBaseFrame(int baxter_link, tf::TransformListener *transform, cv::Point3f pt)
{
    geometry_msgs::PoseStamped tPos_camera, tPos_pt_base, tPos_pt;

    tPos_pt.header.frame_id = "/reference" + baxter_links::link_names[baxter_link];
    tPos_pt.pose.position.y = pt.x;
    tPos_pt.pose.position.x = pt.y;
    tPos_pt.pose.position.z = pt.z;
    tPos_pt.pose.orientation.x = 0.0;
    tPos_pt.pose.orientation.y = 0.0;
    tPos_pt.pose.orientation.z = 0.0;
    tPos_pt.pose.orientation.w = 1.0;

    tf::StampedTransform stamped_t;
    transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_link], ros::Time(0), stamped_t);

    transform->transformPose("/reference/base", tPos_pt, tPos_pt_base);

    tPos_camera = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform);

    tPos_pt.pose.position.x = tPos_pt_base.pose.position.x - tPos_camera.pose.position.x;
    tPos_pt.pose.position.y = tPos_pt_base.pose.position.y - tPos_camera.pose.position.y;
    tPos_pt.pose.position.z = tPos_pt_base.pose.position.z - tPos_camera.pose.position.z;

    return tPos_pt;
}

// ***************************************************************************************************************************
//                                                Taking Orientation into account
// ***************************************************************************************************************************

geometry_msgs::PoseStamped getOrientation(cv::Point3f object_pos_wrt_cam, cv::Point3f target_pt_wrt_cam, tf::TransformListener *transform)
{
    geometry_msgs::PoseStamped  ret_pose;
    geometry_msgs::PoseStamped  t_Pos, v_cam_object;
    geometry_msgs::PoseStamped  cam_pos_wrt_base, target_pt_wrt_base, object_pos_wrt_base;

    // For object_pos_wrt_base, we don't really care about the orientation
    // For cam_pos_wrt_base, we are interested in finding the orientation

    try
    {
        tf::StampedTransform stamped_t;
        transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time(0), stamped_t);
    }
    catch (std::exception ex)
    {
        transform->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time::now(), ros::Duration(1.0));
    }

    ret_pose.header.frame_id = "/reference/base";

    t_Pos.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
    t_Pos.pose.position.x = 0.0;
    t_Pos.pose.position.y = 0.0;
    t_Pos.pose.position.z = 0.0;
    t_Pos.pose.orientation.w = 1.0;
    t_Pos.pose.orientation.x = 0.0;
    t_Pos.pose.orientation.y = 0.0;
    t_Pos.pose.orientation.z = 0.0;

    transform->transformPose("/reference/base", t_Pos, cam_pos_wrt_base);

    t_Pos.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
    t_Pos.pose.position.x = target_pt_wrt_cam.x;
    t_Pos.pose.position.y = target_pt_wrt_cam.y;
    t_Pos.pose.position.z = target_pt_wrt_cam.z;
    t_Pos.pose.orientation.w = 1.0;
    t_Pos.pose.orientation.x = 0.0;
    t_Pos.pose.orientation.y = 0.0;
    t_Pos.pose.orientation.z = 0.0;

    transform->transformPose("/reference/base", t_Pos, target_pt_wrt_base);

    t_Pos.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
    t_Pos.pose.position.x = object_pos_wrt_cam.x;
    t_Pos.pose.position.y = object_pos_wrt_cam.y;
    t_Pos.pose.position.z = object_pos_wrt_cam.z;
    t_Pos.pose.orientation.w = 1.0;
    t_Pos.pose.orientation.x = 0.0;
    t_Pos.pose.orientation.y = 0.0;
    t_Pos.pose.orientation.z = 0.0;

    ROS_INFO("%10.5f, %10.5f, %10.5f --- Camera Pose wrt Base", cam_pos_wrt_base.pose.position.x, cam_pos_wrt_base.pose.position.y, cam_pos_wrt_base.pose.position.z);

    transform->transformPose("/reference/base", t_Pos, object_pos_wrt_base);

    ROS_INFO("%10.5f, %10.5f, %10.5f --- Object Pose wrt Base", object_pos_wrt_base.pose.position.x, object_pos_wrt_base.pose.position.y, object_pos_wrt_base.pose.position.z);

    char c;
    std::cin >> c;


    double                  angle_of_rotation, magnitude_v_cam_object;
    cv::Point3f             axis_of_rotation_xyz;

    v_cam_object.pose.position.x = -(object_pos_wrt_base.pose.position.x - cam_pos_wrt_base.pose.position.x);
    v_cam_object.pose.position.y = -(object_pos_wrt_base.pose.position.y - cam_pos_wrt_base.pose.position.y);
    v_cam_object.pose.position.z = (object_pos_wrt_base.pose.position.z - cam_pos_wrt_base.pose.position.z);
    v_cam_object.pose.orientation.w = 1.0;
    v_cam_object.pose.orientation.x = 0.0;
    v_cam_object.pose.orientation.y = 0.0;
    v_cam_object.pose.orientation.z = 0.0;

    // Normal of v_cam_object
    magnitude_v_cam_object = std::sqrt(std::pow(v_cam_object.pose.position.x, 2) + std::pow(v_cam_object.pose.position.y, 2) + std::pow(v_cam_object.pose.position.z, 2));
    ROS_INFO("Magnitude - %10.5f", magnitude_v_cam_object);
    v_cam_object.pose.position.x = v_cam_object.pose.position.x / magnitude_v_cam_object;
    v_cam_object.pose.position.y = v_cam_object.pose.position.y / magnitude_v_cam_object;
    v_cam_object.pose.position.z = v_cam_object.pose.position.z / magnitude_v_cam_object;

    ROS_INFO("v_cam_object - %10.5f, %10.5f, %10.5f", v_cam_object.pose.position.x, v_cam_object.pose.position.y, v_cam_object.pose.position.z);

    // Cross product gives the axis of rotation
    // Two vectors - v_cam_object(txi + tyj + tzk) and z_unit_vector(k)
    axis_of_rotation_xyz.x = (v_cam_object.pose.position.y);
    axis_of_rotation_xyz.y = (-v_cam_object.pose.position.x);
    axis_of_rotation_xyz.z = 0.0;

    // Dot product
    angle_of_rotation = std::acos(v_cam_object.pose.position.z);
    ROS_INFO("Angle of rotation - %10.5f", angle_of_rotation);

    magnitude_v_cam_object = std::sqrt(std::pow(axis_of_rotation_xyz.x, 2) + std::pow(axis_of_rotation_xyz.y, 2));
    axis_of_rotation_xyz.x = axis_of_rotation_xyz.x / magnitude_v_cam_object;
    axis_of_rotation_xyz.y = axis_of_rotation_xyz.y / magnitude_v_cam_object;

    ret_pose.pose.position.x = target_pt_wrt_base.pose.position.x;
    ret_pose.pose.position.y = target_pt_wrt_base.pose.position.y;
    ret_pose.pose.position.z = target_pt_wrt_base.pose.position.z;

    // Create quaternion
    ret_pose.pose.orientation.w = std::cos(angle_of_rotation / 2);//cam_pos_wrt_base.pose.orientation.w;//std::cos(angle_of_rotation / 2);
    ret_pose.pose.orientation.x = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.x;//cam_pos_wrt_base.pose.orientation.x;//std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.x;
    ret_pose.pose.orientation.y = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.y;//cam_pos_wrt_base.pose.orientation.y;//std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.y;
    ret_pose.pose.orientation.z = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.z;//cam_pos_wrt_base.pose.orientation.z;//std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.z;

    ret_pose.header.frame_id = "/base";

    return ret_pose;
}

void checkIKAndMove()
{

}

void sendBoxToRviz(std::vector<cv::Point3f> box_coords, ros::Publisher &marker_pub)
{
    visualization_msgs::Marker  box_marker;
    geometry_msgs::Point        temp_pt;

    box_marker.header.frame_id = "/reference/base";// + baxter_links::link_names[baxter_links::right_hand_camera];
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "reconstructed_box";
    box_marker.id = 2;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.type = visualization_msgs::Marker::POINTS;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 0.02;
    box_marker.scale.y = 0.02;
    box_marker.scale.z = 0.02;
    box_marker.color.g = 1.0;
    box_marker.color.a = 1.0;

    for (int i = 0; i < box_coords.size(); i++)
    {
        temp_pt.x = ((cv::Point3f) box_coords.at(i)).x;
        temp_pt.y = ((cv::Point3f) box_coords.at(i)).y;
        temp_pt.z = ((cv::Point3f) box_coords.at(i)).z;
        box_marker.points.push_back(temp_pt);

        marker_pub.publish(box_marker);
    }
}

// ***************************************************************************************************************************

void iterative_ik_simulation(cv::Point3f target_point, sensor_msgs::JointState &start_state, cv::Point3f &start_point, cv::Mat pseudo_inverse_jacobian,
                             baxter::BaxterJointControl &joint_control_baxter, ros::NodeHandle nHandle, tf::TransformListener &transform_listener)
{
    std::vector<double>         joint_angles[2];    // 20 is the max number of allowed iterations
    cv::Point3f                 X0;                 // start points at every iteration
    cv::Point3f                 error;              // Distance error that is required to be minimized
    std::vector<double>         temp_angles;
    sensor_msgs::JointState     jsm;

    for (int j = 0; j < 7; j++)
        joint_angles[0].push_back(start_state.position[baxter_joints::right_e0 + j]);

    X0 = start_point;

    computeError(target_point, X0, error);                                      // E    = X[target] - X[i]
    temp_angles.clear();
    computeJointAngles(pseudo_inverse_jacobian, temp_angles, error);            // d_theta = J_inv * error

    // Print the temp_angles
    for (int j = 0; j < 7; j++)
        ROS_INFO_STREAM("Temp angle " << 0 << " - " << temp_angles[j]);

    for (int j = 0; j < 7; j++)
        temp_angles.at(j) = (joint_angles[0].at(j) + temp_angles.at(j));        // theta(i + 1) = theta(i) + d_theta
    joint_angles[1] = temp_angles;

    // Move the robot arm according to the joint angles
    std::vector<double> d_theta;
    std::vector<int>    joint_id;

    for (int j = 0; j < 7; j++)
        ROS_INFO_STREAM("Joint " << j << " - " << joint_angles[1].at(j));

    for (int j = 0; j < 7; j++)
    {
        joint_id.clear();
        d_theta.clear();
        joint_id.push_back(baxter_joints::right_e0 + j);
        d_theta.push_back(joint_angles[1].at(j) - joint_angles[0].at(j));
        ROS_INFO("dtheta %d - %f", baxter_joints::right_e0 + j, joint_angles[1].at(j) - joint_angles[0].at(j));
        joint_control_baxter.moveJoint(joint_id, d_theta, nHandle);
    }
}

cv::Point3f normalOfAPoint(cv::Point3f pt)
{
    cv::Point3f normal;
    double      amplitude;

    amplitude = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2) + std::pow(pt.z, 2));
    normal.x = pt.x / amplitude;
    normal.y = pt.y / amplitude;
    normal.z = pt.z / amplitude;

    return normal;
}

bool dotProductCheck(cv::Point3f a, cv::Point3f b, double epsilon)
{
    double dotP, mag, angle;

    dotP = a.x * b.x + a.y * b.y + a.z * b.z;
    mag = std::sqrt(std::pow(a.x, 2) + std::pow(a.y, 2) + std::pow(a.z, 2)) * std::sqrt(std::pow(b.x, 2) + std::pow(b.y, 2) + std::pow(b.z, 2));

    angle = std::acos(dotP / mag);

    ROS_INFO("ANGLE: - %f   ---  dotP = %f, -----     mag = %f", angle, dotP, mag);

    if (angle < 0)
        angle = -angle;

    if (angle > epsilon)
        return false;

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "master");

    ros::NodeHandle             nHandle;
    std::vector<cv::Point3f>    start_transformed_point;
    std::vector<cv::Point3f>    transformed_point;
    cv::Mat                     transformation_matrix(4, 4, cv::DataType<double>::type);

    std::vector<double>         d_theta;
    std::vector<int>            joint_id;
    double                      theta;
    sensor_msgs::JointState     jsm;
    cv::Mat                     jacobian_3x6(3, 7, cv::DataType<double>::type);
    cv::Mat                     pseudo_inverse_6x3(3, 7, cv::DataType<double>::type);
    cv::Mat                     test_jacobian_3x6(3, 7, cv::DataType<double>::type);
    cv::Mat                     test_pseudo_inverse_6x3(3, 7, cv::DataType<double>::type);
    cv::Mat                     jacobian(6, 7, cv::DataType<double>::type);
    cv::Mat                     pseudo_inverse(6, 7, cv::DataType<double>::type);
    cv::Point3f                 target_point, target_orientation;

    robot_model_loader::RobotModelLoader    robot_model_loader("robot_description");
    robot_model::RobotModelPtr              kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr              kinematic_state(new robot_state::RobotState(kinematic_model));

    kinematic_state->setToDefaultValues();
    robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("right_arm");

    g_joint_state_group = joint_state_group;

    signal(SIGINT, shutdown);

    transform_listener = new tf::TransformListener();

    robotBaxter = new baxter::BaxterMove();
    robotBaxter->kinematic_state->setStateValues(baxter::BaxterMove::joint_state_msg);
    state_control_baxter = new baxter::BaxterStateControl;
    joint_control_baxter = new baxter::BaxterJointControl;

    robotBaxter->subscribeJointStates(nHandle);
    // Get the current joint state values and set them in the current Robot State

    ROS_INFO("Here!");

    state_control_baxter->subscribeAssemblyState(nHandle);
    joint_control_baxter->subscribeJointStates(nHandle);

    g_shutdown = false;
    g_get_transform = false;

    setBoxCoords();

    ros::Subscriber sub = nHandle.subscribe("baxter_topic/right_hand/trans_matrix", 1, trans_mat_callback);

    boost::thread spinning_thread(threadFunc);

    robotBaxter->kinematic_state->setStateValues(baxter::BaxterMove::joint_state_msg);

    joint_control_baxter->setJointCommandMode(baxter_joint_command_mode::POSITION, nHandle);

    ros::Publisher marker_pub = nHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    for (int i = 0; i < 7; i++)
    {
        jacobian.at<double>(3, i) = 0;
        jacobian.at<double>(4, i) = 0;
        jacobian.at<double>(5, i) = 1;
    }

    bool recompute = true;

    geometry_msgs::PoseStamped tPosBox_cam;
    cv::Point3f start_pt, end_pt, init_pt_box_cam, init_pt_cam_base;

    while (g_shutdown == false)
    {
/*        if (recompute == true)
        {
            // Compute Jacobian
            for (int i = 0; i < 7; i++)
            {
                // Get initial coordinates
                // BEGIN
                g_get_transform = true;
                while (g_get_transform == true);
                g_transformation_matrix.copyTo(transformation_matrix);
                start_transformed_point.clear();
                for (int j = 0; j < 6; j++)
                {
                    start_transformed_point.push_back(cv::Point3f(0.0, 0.0, 0.0));
                    transform_coords(transformation_matrix, g_box_coordinates[j], start_transformed_point[j]);
                    tPosBox_cam = getCoordsWrtCamInBaseFrame(baxter_links::right_hand_camera, transform_listener, start_transformed_point[j]); // Gives the start box coords wrt the cam origin and base frame orientation
                    start_transformed_point[j].x = tPosBox_cam.pose.position.x;
                    start_transformed_point[j].y = tPosBox_cam.pose.position.y;
                    start_transformed_point[j].z = tPosBox_cam.pose.position.z;

                    ROS_INFO("Starting box pt: X - %10.5f, Y - %10.5f, Z - %10.5f", start_transformed_point[j].x, start_transformed_point[j].y, start_transformed_point[j].z);
                }
                // END

                // Move joint 'i' by 0.1 radians
                // BEGIN
                double prev_theta = baxter::BaxterJointControl::joint_state_msg.position[baxter_joints::right_e0 + i];
                d_theta.clear();
                joint_id.clear();
                d_theta.push_back(0.1);
                joint_id.push_back(baxter_joints::right_e0 + i);
                joint_control_baxter->moveJoint(joint_id, d_theta, nHandle);              // Revert back
                // END

                // Get current coordinates
                // BEGIN
                g_get_transform = true;
                while (g_get_transform == true);
                g_transformation_matrix.copyTo(transformation_matrix);
                transformed_point.clear();
                for (int j = 0; j < 6;j++)
                {
                    transformed_point.push_back(cv::Point3f(0.0, 0.0, 0.0));
                    transform_coords(transformation_matrix, g_box_coordinates[j], transformed_point[j]);
                    tPosBox_cam = getCoordsWrtCamInBaseFrame(baxter_links::right_hand_camera, transform_listener, transformed_point[j]);  // Gives the current box coords wrt the cam origin and base frame orientation
                    transformed_point[j].x = tPosBox_cam.pose.position.x;
                    transformed_point[j].y = tPosBox_cam.pose.position.y;
                    transformed_point[j].z = tPosBox_cam.pose.position.z;

                    ROS_INFO("Current box pt: X - %10.5f, Y - %10.5f, Z - %10.5f", transformed_point[j].x, transformed_point[j].y, transformed_point[j].z);
                }
                // END

                //sendBoxToRviz(transformed_point, marker_pub);

                double cur_theta = baxter::BaxterJointControl::joint_state_msg.position[baxter_joints::right_e0 + i];
                if (cur_theta - prev_theta > 0.05)
                {
                    jacobian_3x6.at<double>(0, i) = (transformed_point[3].x - start_transformed_point[3].x) / (cur_theta - prev_theta);//(start_transformed_point[3].x - transformed_point[3].x) / (cur_theta - prev_theta);
                    jacobian_3x6.at<double>(1, i) = (transformed_point[3].y - start_transformed_point[3].y) / (cur_theta - prev_theta);//(start_transformed_point[3].y - transformed_point[3].y) / (cur_theta - prev_theta);
                    jacobian_3x6.at<double>(2, i) = (transformed_point[3].z - start_transformed_point[3].z) / (cur_theta - prev_theta);//(start_transformed_point[3].z - transformed_point[3].z) / (cur_theta - prev_theta);

                    //jacobian.at<double>(0, i) = (transformed_point[3].x - start_transformed_point[3].x) / (cur_theta - prev_theta);//(start_transformed_point[3].x - transformed_point[3].x) / (prev_theta - cur_theta);//
                    //jacobian.at<double>(1, i) = (transformed_point[3].y - start_transformed_point[3].y) / (cur_theta - prev_theta);//(start_transformed_point[3].y - transformed_point[3].y) / (prev_theta - cur_theta);//
                    //jacobian.at<double>(2, i) = (transformed_point[3].z - start_transformed_point[3].z) / (cur_theta - prev_theta);//(start_transformed_point[3].z - transformed_point[3].z) / (prev_theta - cur_theta);//
                    //jacobian.at<double>(3, i) = ((transformed_point[3].x - transformed_point[5].x) - (start_transformed_point[3].x - start_transformed_point[5].x)) / (cur_theta - prev_theta);
                    //jacobian.at<double>(4, i) = ((transformed_point[3].y - transformed_point[5].y) - (start_transformed_point[3].y - start_transformed_point[5].y)) / (cur_theta - prev_theta);
                    //jacobian.at<double>(5, i) = ((transformed_point[3].z - transformed_point[5].z) - (start_transformed_point[3].z - start_transformed_point[5].z)) / (cur_theta - prev_theta);
                }
                else
                {
                    ROS_INFO("D_THETA = %f", cur_theta - prev_theta);
                    i--;
                }

                d_theta.clear();
                d_theta.push_back(-0.1);
                joint_control_baxter->moveJoint(joint_id, d_theta, nHandle);              // Revert back
            }

            g_get_transform = true;
            while (g_get_transform == true);
            g_transformation_matrix.copyTo(transformation_matrix);
            transform_coords(transformation_matrix, g_box_coordinates[3], init_pt_box_cam);
            tPosBox_cam = getCoordsWrtCamInBaseFrame(baxter_links::right_hand_camera, transform_listener, init_pt_box_cam);
            // The init_pt_box_cam stores the values wrt the
            init_pt_box_cam.x = tPosBox_cam.pose.position.x;
            init_pt_box_cam.y = tPosBox_cam.pose.position.y;
            init_pt_box_cam.z = tPosBox_cam.pose.position.z;

            target_point.x = init_pt_box_cam.x;
            target_point.y = init_pt_box_cam.y;
            target_point.z = init_pt_box_cam.z;

            tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
            init_pt_cam_base.x = tPosBox_cam.pose.position.x;
            init_pt_cam_base.y = tPosBox_cam.pose.position.y;
            init_pt_cam_base.z = tPosBox_cam.pose.position.z;
        }
        else
        {
            recompute = true;
            continue;
            g_get_transform = true;
            while (g_get_transform == true);
            g_transformation_matrix.copyTo(transformation_matrix);

        }

        ROS_INFO("Jacobian - \n\n");

        for (int i = 0; i < 3; i++)
        {
            ROS_INFO("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f", jacobian_3x6.at<double>(i, 0), jacobian_3x6.at<double>(i, 1), jacobian_3x6.at<double>(i, 2),
                     jacobian_3x6.at<double>(i, 3), jacobian_3x6.at<double>(i, 4), jacobian_3x6.at<double>(i, 5), jacobian_3x6.at<double>(i, 6));
            ROS_INFO("");
            ROS_INFO("");
        }

        //pseudo_inverse = jacobian.inv(cv::DECOMP_SVD);
        pseudo_inverse_6x3 = jacobian_3x6.inv(cv::DECOMP_SVD);

        ROS_INFO("Jacobian Pseudo Inverse - \n\n");

        for (int i = 0; i < 7; i++)
        {
            ROS_INFO("%10.5f %10.5f %10.5f", pseudo_inverse_6x3.at<double>(i, 0), pseudo_inverse_6x3.at<double>(i, 1), pseudo_inverse_6x3.at<double>(i, 2));//,
                     //pseudo_inverse_6x3.at<double>(i, 3));//, pseudo_inverse.at<double>(i, 4), pseudo_inverse.at<double>(i, 5));
            ROS_INFO("");
            ROS_INFO("");
        }

        cv::Point3f A_B, test, A_B2;

        char c;
        std::cin >> c;

        test = normalOfAPoint(target_point);
        ROS_INFO("X - %f, Y - %f, Z - %f", test.x, test.y, test.z);
        target_point.x = test.x * 0.05;
        target_point.y = test.y * 0.05;
        target_point.z = test.z * 0.05;
        ROS_INFO("Target - X - %f, Y - %f, Z - %f", target_point.x, target_point.y, target_point.z);

        test.x = 0.0;
        test.y = 0.0;
        test.z = 0.0;

        iterative_ik_simulation(target_point, baxter::BaxterJointControl::joint_state_msg, test, pseudo_inverse_6x3, *joint_control_baxter, nHandle, *transform_listener);

        //geometry_msgs::PoseStamped targetPos;

        //targetPos = getOrientation(start_transformed_point[3], target_point, transform_listener);
        //if (robotBaxter->checkIKexists(targetPos, RIGHT_ARM))
        //{
        //    ROS_INFO("Moving now - \n\n");
        //    robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        //    robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        //    robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        //    std::cin >> c;
        //}
        //else
        //{
        //    ROS_INFO("No IK!");
        //    char y;
        //    std::cin >> y;
        //}

        //baxter_moveit_move(*transform_listener, target_point, target_orientation);
        //iterative_ik_solve_and_move(target_point, target_orientation, baxter::BaxterJointControl::joint_state_msg, pseudo_inverse_6x3, *joint_control_baxter, nHandle, *transform_listener);
        //iterative_ik_solve_and_move(target_point, target_orientation, baxter::BaxterJointControl::joint_state_msg, pseudo_inverse, *joint_control_baxter, nHandle, *transform_listener);

*/
        // SIMULATION IK

        if (recompute == true)
        {
            tPosBox_cam = getBoxCoordsWrtCamFrame(baxter_links::right_hand_camera, transform_listener);
            init_pt_box_cam.x = tPosBox_cam.pose.position.x;
            init_pt_box_cam.y = tPosBox_cam.pose.position.y;
            init_pt_box_cam.z = tPosBox_cam.pose.position.z;

            tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
            init_pt_cam_base.x = tPosBox_cam.pose.position.x;
            init_pt_cam_base.y = tPosBox_cam.pose.position.y;
            init_pt_cam_base.z = tPosBox_cam.pose.position.z;

            // Compute Jacobian
            for (int i = 0; i < 7; i++)
            {
                // Check the starting coordinates at the beginning
                start_transformed_point.clear();
                tPosBox_cam = getBoxCoordsWrtCamFrame(baxter_links::right_hand_camera, transform_listener);
                start_transformed_point.push_back(cv::Point3f(tPosBox_cam.pose.position.x, tPosBox_cam.pose.position.y, tPosBox_cam.pose.position.z));
                ROS_INFO("pt before: X - %10.5f, Y - %10.5f, Z - %10.5f", start_transformed_point[0].x, start_transformed_point[0].y, start_transformed_point[0].z);

                tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
                start_pt.x = tPosBox_cam.pose.position.x;
                start_pt.y = tPosBox_cam.pose.position.y;
                start_pt.z = tPosBox_cam.pose.position.z;

                ROS_INFO("pt before: X - %10.5f, Y - %10.5f, Z - %10.5f", start_pt.x, start_pt.y, start_pt.z);

                // Move Joint 'i' by 0.1 radians
                double prev_theta = baxter::BaxterJointControl::joint_state_msg.position[baxter_joints::right_e0 + i];
                d_theta.clear();
                joint_id.clear();
                d_theta.push_back(0.1);
                joint_id.push_back(baxter_joints::right_e0 + i);
                joint_control_baxter->moveJoint(joint_id, d_theta, nHandle);

                tPosBox_cam = getBoxCoordsWrtCamFrame(baxter_links::right_hand_camera, transform_listener);
                start_transformed_point.push_back(cv::Point3f(tPosBox_cam.pose.position.x, tPosBox_cam.pose.position.y, tPosBox_cam.pose.position.z));
                ROS_INFO("pt after: X - %10.5f, Y - %10.5f, Z - %10.5f", start_transformed_point[1].x, start_transformed_point[1].y, start_transformed_point[1].z);

                tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
                end_pt.x = tPosBox_cam.pose.position.x;
                end_pt.y = tPosBox_cam.pose.position.y;
                end_pt.z = tPosBox_cam.pose.position.z;

                ROS_INFO("pt after: X - %10.5f, Y - %10.5f, Z - %10.5f", end_pt.x, end_pt.y, end_pt.z);

                double cur_theta = baxter::BaxterJointControl::joint_state_msg.position[baxter_joints::right_e0 + i];
                if (cur_theta - prev_theta > 0.05)
                {
                    cv::Point3f A_B;
                    jacobian_3x6.at<double>(0, i) = (start_transformed_point[1].x - start_transformed_point[0].x) / (cur_theta - prev_theta);
                    jacobian_3x6.at<double>(1, i) = (start_transformed_point[1].y - start_transformed_point[0].y) / (cur_theta - prev_theta);
                    jacobian_3x6.at<double>(2, i) = (start_transformed_point[1].z - start_transformed_point[0].z) / (cur_theta - prev_theta);

                    ROS_INFO("%f <- Error", computeError(start_transformed_point[0], start_transformed_point[1], A_B));

                    test_jacobian_3x6.at<double>(0, i) = (end_pt.x - start_pt.x) / (cur_theta - prev_theta);
                    test_jacobian_3x6.at<double>(1, i) = (end_pt.y - start_pt.y) / (cur_theta - prev_theta);
                    test_jacobian_3x6.at<double>(2, i) = (end_pt.z - start_pt.z) / (cur_theta - prev_theta);

                    ROS_INFO("%f <- Error", computeError(start_pt, end_pt, A_B));
                }
                else
                {
                    // Use a repeat until loop and move the cur_theta to a better pos
                    ROS_INFO("D_THETA = %f", cur_theta - prev_theta);
                    i--;
                }

                // Move back Joint 'i' to its original position by moving back by 0.1 radians
                d_theta.clear();
                d_theta.push_back(-0.1);
                joint_control_baxter->moveJoint(joint_id, d_theta, nHandle);
            }

            ROS_INFO("Jacobian - \n\n");

            for (int i = 0; i < 3; i++)
            {
                ROS_INFO("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f", jacobian_3x6.at<double>(i, 0), jacobian_3x6.at<double>(i, 1), jacobian_3x6.at<double>(i, 2),
                         jacobian_3x6.at<double>(i, 3), jacobian_3x6.at<double>(i, 4), jacobian_3x6.at<double>(i, 5), jacobian_3x6.at<double>(i, 6));
                ROS_INFO("");
                ROS_INFO("");
            }

            ROS_INFO(" Test Jacobian - \n\n");

            for (int i = 0; i < 3; i++)
            {
                ROS_INFO("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f", test_jacobian_3x6.at<double>(i, 0), test_jacobian_3x6.at<double>(i, 1), test_jacobian_3x6.at<double>(i, 2),
                         test_jacobian_3x6.at<double>(i, 3), test_jacobian_3x6.at<double>(i, 4), test_jacobian_3x6.at<double>(i, 5), test_jacobian_3x6.at<double>(i, 6));
                ROS_INFO("");
                ROS_INFO("");
            }

            pseudo_inverse = jacobian.inv(cv::DECOMP_SVD);
            pseudo_inverse_6x3 = jacobian_3x6.inv(cv::DECOMP_SVD);
            test_pseudo_inverse_6x3 = test_jacobian_3x6.inv(cv::DECOMP_SVD);

            ROS_INFO("Jacobian Pseudo Inverse - \n\n");

            for (int i = 0; i < 7; i++)
            {
                ROS_INFO("%10.5f %10.5f %10.5f", pseudo_inverse_6x3.at<double>(i, 0), pseudo_inverse_6x3.at<double>(i, 1), pseudo_inverse_6x3.at<double>(i, 2));//,
                         //pseudo_inverse_6x3.at<double>(i, 3));//, pseudo_inverse.at<double>(i, 4), pseudo_inverse.at<double>(i, 5));
                ROS_INFO("");
                ROS_INFO("");
            }

            ROS_INFO("Test Jacobian Pseudo Inverse - \n\n");

            for (int i = 0; i < 7; i++)
            {
                ROS_INFO("%10.5f %10.5f %10.5f", test_pseudo_inverse_6x3.at<double>(i, 0), test_pseudo_inverse_6x3.at<double>(i, 1), test_pseudo_inverse_6x3.at<double>(i, 2));//,
                         //pseudo_inverse_6x3.at<double>(i, 3));//, pseudo_inverse.at<double>(i, 4), pseudo_inverse.at<double>(i, 5));
                ROS_INFO("");
                ROS_INFO("");
            }

            ROS_INFO("Target point - %f, %f, %f", start_transformed_point[0].x, start_transformed_point[0].y, start_transformed_point[0].z);

            target_point.x = start_transformed_point[0].x;
            target_point.y = start_transformed_point[0].y;
            target_point.z = start_transformed_point[0].z;

            //char c;
            //std::cin >> c;
        }
        else
        {
            tPosBox_cam = getBoxCoordsWrtCamFrame(baxter_links::right_hand_camera, transform_listener);

            target_point.x = tPosBox_cam.pose.position.x;
            target_point.y = tPosBox_cam.pose.position.y;
            target_point.z = tPosBox_cam.pose.position.z;
        }

        cv::Point3f A_B, test, A_B2;

        test.x = target_point.x;
        test.y = target_point.y;
        test.z = target_point.z;

        A_B.x = 0.0;
        A_B.y = 0.0;
        A_B.z = 0.0;

        if (computeError(A_B, test, A_B2) < 0.15)
            return 1;
        else
            ROS_INFO("Exit error - %f", computeError(A_B, test, A_B2));

        test = normalOfAPoint(target_point);
        ROS_INFO("X - %f, Y - %f, Z - %f", test.x, test.y, test.z);
        target_point.x = test.x * 0.05;
        target_point.y = test.y * 0.05;
        target_point.z = test.z * 0.05;

        tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
        start_pt.x = tPosBox_cam.pose.position.x;
        start_pt.y = tPosBox_cam.pose.position.y;
        start_pt.z = tPosBox_cam.pose.position.z;

        test.x = 0.0;
        test.y = 0.0;
        test.z = 0.0;

        iterative_ik_simulation(target_point, baxter::BaxterJointControl::joint_state_msg, test, pseudo_inverse_6x3, *joint_control_baxter, nHandle, *transform_listener);

        tPosBox_cam = getCamCoordsWrtBaseFrame(baxter_links::right_hand_camera, transform_listener);
        test.x = tPosBox_cam.pose.position.x - init_pt_cam_base.x;
        test.y = tPosBox_cam.pose.position.y - init_pt_cam_base.y;
        test.z = tPosBox_cam.pose.position.z - init_pt_cam_base.z;

        ROS_INFO("Init pt - %f, %f, %f  ---  Cur pt - %f, %f, %f", init_pt_cam_base.x, init_pt_cam_base.y, init_pt_cam_base.z, test.x, test.y, test.z);

        if (dotProductCheck(init_pt_box_cam, test, 0.8) == true)
            recompute = false;
        else
            recompute = true;

    }

    return 0;
}