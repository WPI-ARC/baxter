#include "reconstruction.h"

void shutdown(int sig)
{
    ros::shutdown();
}

void threadFunc()
{
    ros::Rate r(50);    // Nyquist Sampling frequency f >= 2 BW = 2 * frame_rate = 2 * 24 = 48
    while (ros::ok())
    {
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
    }
}

void mouseCallback(int event, int x, int y, int flags, void* param)
{
    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
                std::cout << "Mouse clicked at x = " << x << " y = " << y << std::endl;
                if (coordinates_entered == 0)
                    g_box_coordinates.push_back(cv::Point2f((float) x, (float) y));
                else
                    g_box_coordinates.clear();
                break;
    }
}

void setBoxCoords(void)
{
    g_box.clear();

    g_box.push_back(cv::Point3f(-0.256, 0.183, 0.0));         // Coords for AD
    g_box.push_back(cv::Point3f(0.0, 0.183, 0.0));           // Coords for BD
    g_box.push_back(cv::Point3f(-0.256, 0.0, 0.0));          // Coords for CD
    g_box.push_back(cv::Point3f(0.0, 0.0, 0.0));            // Coords for DD
    g_box.push_back(cv::Point3f(-0.256, 0.0, -0.095));         // Coords for ED
    g_box.push_back(cv::Point3f(0.0, 0.0, -0.095));           // Coords for FD
}

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

// Inputs  - Intrinsic Matrix, Distortion Matrix, Detected/Selected Corners
// Outputs - Transformation Matrix, RViz Corners
void detectCornerSet(cv::Mat &intrinsic_mat, cv::Mat &distortion_mat, std::vector<cv::Point2f> &corners, cv::Mat &transformation_matrix, std::vector<cv::Point3f> &rviz_corners)
{
    cv::Mat                     rvec(3, 1, cv::DataType<double>::type);
    cv::Mat                     tvec(3, 1, cv::DataType<double>::type);
    cv::Mat                     rmatrix(3, 3, cv::DataType<double>::type);
    cv::Point3f                 temp_pt;

    cv::solvePnP(g_box, corners, intrinsic_mat, distortion_mat, rvec, tvec);
    cv::Rodrigues(rvec, rmatrix);

    transformation_matrix.at<double>(0, 3) = tvec.at<double>(0, 0);
    transformation_matrix.at<double>(1, 3) = tvec.at<double>(1, 0);
    transformation_matrix.at<double>(2, 3) = tvec.at<double>(2, 0);
    transformation_matrix.at<double>(3, 0) = 0.0;
    transformation_matrix.at<double>(3, 1) = 0.0;
    transformation_matrix.at<double>(3, 2) = 0.0;
    transformation_matrix.at<double>(3, 3) = 1.0;

    for (int k = 0; k < 3; k++)
        for (int l = 0; l < 3; l++)
            transformation_matrix.at<double>(k, l) = rmatrix.at<double>(k, l);

    g_box.push_back(cv::Point3f(-0.256, 0.183, -0.095));    // Coords for GD
    g_box.push_back(cv::Point3f(0.0, 0.183, -0.095));       // Coords for HD

    rviz_corners.clear();
    for (int i = 0; i < g_box.size(); i++)
    {
        transform_coords(transformation_matrix, g_box.at(i), temp_pt);
        temp_pt.x = temp_pt.x + 0.3;
        rviz_corners.push_back(cv::Point3f(temp_pt.x, temp_pt.y, temp_pt.z));
    }

    setBoxCoords();
}

void sendBoxToRviz(std::vector<cv::Point3f> box_coords, ros::Publisher &marker_pub)
{
    visualization_msgs::Marker  box_marker;
    geometry_msgs::Point        temp_pt;

    box_marker.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reconstruction");

    ros::NodeHandle             nHandle;
    cv_bridge::CvImagePtr       cam_image;
    cv::Mat                     src_img, gray_img, binary_img, smoothed_binary_img, hsv_img, imgA, imgB;
    std::vector<cv::Point3f>    rviz_corners;
    cv::Mat                     right_intrinsic_mat(3, 3, cv::DataType<double>::type);
    cv::Mat                     right_distortion_mat(1, 5, cv::DataType<double>::type);
    cv::Mat                     transformation_matrix(4, 4, cv::DataType<double>::type);

    for (int i = 0; i < 3; i++)
    {
        right_intrinsic_mat.at<double>(i, 0) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3)];
        right_intrinsic_mat.at<double>(i, 1) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 1];
        right_intrinsic_mat.at<double>(i, 2) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 2];
    }

    for (int i = 0; i < 5; i++)
        right_distortion_mat.at<double>(0, i) = 0;

    g_box_coordinates.clear();
    coordinates_entered = 0;
    setBoxCoords();

    signal(SIGINT, shutdown);
    g_shutdown = false;

    cam_baxter = new baxter::BaxterCam();
    cam_baxter->subscribeCameraImage(baxter_camera::right_hand_camera, nHandle);

    boost::thread spinning_thread(threadFunc);
    ros::Rate loop_rate(25);

    cv::namedWindow("cam1", cv::WINDOW_NORMAL);
    cv::createTrackbar("Coords entered: ", "cam1", &coordinates_entered, 1);// &switch_callback);
    cv::setMouseCallback("cam1", mouseCallback);

    //ros::Publisher trans_matrix_pub = nHandle.advertise<baxter_cams1::transMatrix>("/baxter_topic/right_hand/trans_matrix", 0);
    ros::Publisher marker_pub = nHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    while (ros::ok())
    {
        try
        {
            cam_image = cam_baxter->readImageFromCamera(baxter_camera::right_hand_camera, cam_image);

            if (cam_image)
            {
                src_img = cam_image->image.clone();
                //cv::cvtColor(src_img, gray_img, CV_BGR2GRAY);
                //cv::cvtColor(src_img, hsv_img, CV_BGR2HSV);

                for (int i = 0; i < g_box_coordinates.size(); i++)
                    cv::circle(src_img, g_box_coordinates.at(i), 2, cv::Scalar(255, 255, 0), 1, 8, 0);
            }

            if (coordinates_entered == 1)
            {
                detectCornerSet(right_intrinsic_mat, right_distortion_mat, g_box_coordinates, transformation_matrix, rviz_corners);
                sendBoxToRviz(rviz_corners, marker_pub);
            }

            cv::imshow("cam1", src_img);
            cv::waitKey(10);
        }
        catch (std::exception ex)
        {
            // Do nothing
        }

        loop_rate.sleep();
    }

    return 0;
}
