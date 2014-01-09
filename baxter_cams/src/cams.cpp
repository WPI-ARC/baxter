// NODE 2 implementation - Camera Tracking
// Listens on topic - /cv_image_right_hand_camera

#include "cams.h"

void threadFunc()
{
    ros::Rate r(50);    // Nyquist Sampling frequency f >= 2 BW = 2 * frame_rate = 2 * 24 = 48
    while (ros::ok())
    {
      ros::spinOnce();                   // Handle ROS events
      r.sleep();
    }
}

void getCorners(cv::Mat &gray_image, std::vector<cv::Point2f> &corners)
{
    double          quality_level, min_distance;

    quality_level = 0.1;
    min_distance = 30;

    cv::goodFeaturesToTrack(gray_image, corners, 200, quality_level, min_distance);
}

size_t getBiggestBlob(cv::Mat &binary_image, std::vector<cv::Point> &blob)
{
    std::vector< std::vector<cv::Point> > contours;
    std::size_t idx;
    int max_area = 0, area_id = 0, area;
    cv::Mat contourOutput;

    try
    {
        cv::findContours(binary_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        if (contours.size() != 0)
        {
            for (idx = 0; idx < contours.size(); idx++)
            {
                area = cv::contourArea(contours[idx]);
                if (area > max_area)
                {
                    max_area = area;
                    area_id = idx;
                }
            }

            cv::threshold(binary_image, binary_image, 255, 255, cv::THRESH_BINARY);
            cv::drawContours(binary_image, contours, area_id, cv::Scalar(255, 255, 255), CV_FILLED);

            blob.swap(contours[area_id]);
        }
    }
    catch (std::exception ex)
    {
        // Do nothing
    }

    return contours.size();
}

double absoluteVal(double x)
{
    if (x < 0)
        return -x;
    return x;
}

bool checkNeighbourhood(cv::Mat &binary_image, cv::Point2f &corners, std::vector<cv::Point> &blob)
{
    int i, j;
    bool corner_true;

    corner_true = false;


    if (cv::pointPolygonTest(blob, corners, false) == 1)
        return true;

    for (i = 0; i < binary_image.rows; i++)
        for (j = 0; j < binary_image.cols; j++)
        {
            if ((binary_image.at<unsigned char>(i, j) != 255) && (binary_image.at<unsigned char>(i, j) != 0))
                ROS_INFO("%d", binary_image.at<unsigned char>(i, j));
        }

    return false;
}

void findNearestNeighbours(cv::Point2f n_Point, std::vector<cv::Point2f> &corners)
{
    int i, j;
    double min_distance = 1000000.0f, temp_dist;
    cv::Point2f minPoint;
    std::vector<cv::Point2f> swap_corner_vec;

    for (i = 0; i < corners.size(); i++)
    {
        temp_dist = (((corners[i].x - n_Point.x)*(corners[i].x - n_Point.x)) + ((corners[i].y - n_Point.y)*(corners[i].y - n_Point.y)));
        if (temp_dist < min_distance)
        {
            min_distance = temp_dist;
            minPoint = corners[i];
        }
    }

    for (i = 0; i < corners.size(); i++)
    {
        if (corners[i] == minPoint)
        {
            swap_corner_vec.push_back(n_Point);
        }
        swap_corner_vec.push_back(corners[i]);
    }

    corners.swap(swap_corner_vec);
}

void drawPolygon(cv::Mat &image, std::vector<cv::Point2f> &corners)
{
    int                 i, j, k, l, size = corners.size();
    Eigen::MatrixXd     lenMat(size, size);
    std::vector<cv::Point2f>   hull, not_in_hull;
    bool present;

    if (corners.size() != 0)
    {
        not_in_hull.clear();
        cv::convexHull(corners, hull);

        hull.push_back(hull.at(0));

        if ((hull.size() - 1) != corners.size())
        {
            for (i = 0; i < corners.size(); i++)
            {
                for (j = 0; j < hull.size(); j++)
                {
                    if (hull[j] == corners[i])
                        present = true;
                }
                if (present == false)
                    not_in_hull.push_back(corners.at(i));
                present = false;
            }
        }

        for (i = 0; i < not_in_hull.size(); i++)
        {
            findNearestNeighbours(not_in_hull[i], hull);
        }

        for (i = 0; i < hull.size() - 1; i++)
        {
            cv::line(image, hull[i], hull[i + 1], CV_RGB(255, 0, 0), 1);
        }
    }
}

void shutdown(int sig)
{
    g_shutdown = true;
}

void mouseCallback(int event, int x, int y, int flags, void* param)
{
    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
                std::cout << "Mouse clicked at x = " << x << " y = " << y << std::endl;
                g_point_identifiers.push_back(cv::Point2f((float) x, (float) y));
                if (g_point_identifiers.size() > 6)
                {
                    g_point_identifiers.clear();
                    g_identified_corners = false;
                    g_set_identifiers = false;
                }
                if (g_point_identifiers.size() == 6)
                {
                    std::vector<cv::Point2f> temp_vec;
                    g_identified_corners = true;
                    for (int i = 0; i < g_point_identifiers.size(); i++)
                    {
                        temp_vec.push_back(findNearestPoint(g_point_identifiers.at(i), g_right_corners));
                    }
                    temp_vec.swap(g_right_corners);
                    g_set_identifiers = true;
                }

                break;
    }
}

cv::Point2f findNearestPoint(cv::Point2f &right_corner, std::vector<cv::Point2f> &corners_temp)
{
    int i;

    for (i = 0; i < corners_temp.size(); i++)
    {
        if ((absoluteVal(right_corner.x - (corners_temp.at(i)).x) < 5) && (absoluteVal(right_corner.y - (corners_temp.at(i)).y) < 5))
        {
            right_corner.x = (corners_temp.at(i)).x;
            right_corner.y = (corners_temp.at(i)).y;
            return right_corner;
        }
    }
}

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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cams");

    ros::NodeHandle             nHandle;
    cv_bridge::CvImagePtr       cam_image;
    cv::Mat                     src_img, gray_img, binary_img, smoothed_binary_img, hsv_img, imgA, imgB;
    std::vector<cv::Point2f>    corners;
    std::vector<cv::Point2f>    right_corners;
    std::vector<cv::Point2f>    next_corners;
    std::vector<uchar>			status;
    std::vector<float>			error;
    std::vector<cv::Point>      blob;
    bool                        start_tracking;

    cv::Mat                     right_intrinsic_mat(3, 3, cv::DataType<double>::type);
    cv::Mat                     right_distortion_mat(1, 5, cv::DataType<double>::type);
    cv::Mat                     rvec(3, 1, cv::DataType<double>::type);
    cv::Mat                     tvec(3, 1, cv::DataType<double>::type);
    cv::Mat                     rmatrix(3, 3, cv::DataType<double>::type);
    cv::Mat                     transformation_matrix(4, 4, cv::DataType<double>::type);

    for (int i = 0; i < 3; i++)
    {
        right_intrinsic_mat.at<double>(i, 0) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3)];
        right_intrinsic_mat.at<double>(i, 1) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 1];
        right_intrinsic_mat.at<double>(i, 2) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 2];
    }

    for (int i = 0; i < 5; i++)
        right_distortion_mat.at<double>(0, i) = 0;//baxter_camera::distortion_coeffs::right_hand_camera[i];

    for (int k = 0; k < 3; k++)
        ROS_INFO("%f       %f       %f", right_intrinsic_mat.at<double>(k, 0), right_intrinsic_mat.at<double>(k, 1), right_intrinsic_mat.at<double>(k, 2));

    for (int k = 0; k < 5; k++)
        ROS_INFO("%f", right_distortion_mat.at<double>(0, k));

    setBoxCoords();
    char c;
    std::cin >> c;

    signal(SIGINT, shutdown);
    g_shutdown = false;

    cam_baxter = new baxter::BaxterCam();
    state_control_baxter = new baxter::BaxterStateControl;

    cam_baxter->subscribeCameraImage(baxter_camera::right_hand_camera, nHandle);
    state_control_baxter->subscribeAssemblyState(nHandle);

    boost::thread spinning_thread(threadFunc);

    ros::Rate loop_rate(25);

    cv::namedWindow("cam1", cv::WINDOW_NORMAL);
    cv::namedWindow("cam2");

    /// Create Trackbar to choose type of Threshold
    cv::createTrackbar("Value: ","cam2", &threshold_value, 255);
    cv::setMouseCallback("cam1", mouseCallback);

    ROS_INFO("Enabled - %d", (state_control_baxter->enableRobot(nHandle)).enabled);

    start_tracking = false;
    g_identified_corners = false;
    g_set_identifiers = false;

    ros::Publisher trans_matrix_pub = nHandle.advertise<baxter_cams::transMatrix>("/baxter_topic/right_hand/trans_matrix", 0);

    while(g_shutdown == false)
    {
        try
        {
            int i;

            cam_image = cam_baxter->readImageFromCamera(baxter_camera::right_hand_camera, cam_image);

            if (cam_image)
            {
                src_img = cam_image->image.clone();
                cv::cvtColor(src_img, gray_img, CV_BGR2GRAY);
                cv::cvtColor(src_img, hsv_img, CV_BGR2HSV);
                smoothed_binary_img = gray_img.clone();
                imgB = gray_img.clone();

                cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), smoothed_binary_img);    // Segment the Red blobs from the image

                int MAX_KERNEL_LENGTH = 31;
                for ( i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
                    cv::GaussianBlur(smoothed_binary_img, smoothed_binary_img, cv::Size( i, i ), 0, 0 );            // Smoothen the image using Gaussian blur
                cv::threshold(smoothed_binary_img, smoothed_binary_img, threshold_value, 255, cv::THRESH_BINARY);   // Convert from grayscale to binary

                if (start_tracking == false)
                {
                    getCorners(gray_img, corners);

                    if (getBiggestBlob(smoothed_binary_img, blob) != 0)                                                 // Get the biggest blob (the box) out of the various blobs
                    {
                        binary_img = smoothed_binary_img.clone();                                                       // Store the smoothed binary image in binary_img

                        cv::Moments moment = cv::moments(smoothed_binary_img, true);                                    // Calculate moments of the image
                        cv::Point2f centroid = cv::Point2f((moment.m10/moment.m00), (moment.m01/moment.m00));           // Calculate the centroid of the box

                        cv::circle(src_img, centroid, 5, CV_RGB(255, 0, 0), 2);                                         // Draw the centroid (just for fun)

                        right_corners.clear();                                                                          // Clear the structure storing the correct corners

                        for (i = 0; i < corners.size(); i++)
                            if (((cv::Point2f) corners.at(i)).y > 215)
                                if (checkNeighbourhood(binary_img, corners.at(i), blob) == true)
                                {
                                    right_corners.push_back(corners.at(i));
                                }

                        cv::imshow("cam2", binary_img);                                                                 // Show the binary_img
                    }

                    if (right_corners.size() == 6)
                    {
                        start_tracking = true;
                        //while (!g_identified_corners);
                    }
                }
                else
                {
                    if (right_corners.size() != 6)
                    {
                        start_tracking = false;
                        continue;
                    }

                    status.clear();
                    error.clear();
                    next_corners.clear();
                    cv::calcOpticalFlowPyrLK(imgA,                                                                          // Source image
                                             imgB,                                                                          // Next image
                                             right_corners,                                                                 // Corners to track
                                             next_corners,                                                                  // Tracked corner coordinates
                                             status,                                                                        // Status vector
                                             error,                                                                         // Error vector
                                             cv::Size(21, 21),                                                              // Window frame size
                                             3,                                                                             // Max Pyramid Level
                                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1),    // Termination criteria
                                             0);                                                                            // Flags

                    next_corners.swap(right_corners);

                    if (getBiggestBlob(smoothed_binary_img, blob) != 0)
                    {
                        cv::Moments moment = cv::moments(smoothed_binary_img, true);                                    // Calculate moments of the image
                        cv::Point2f centroid = cv::Point2f((moment.m10/moment.m00), (moment.m01/moment.m00));           // Calculate the centroid of the box
                        std::vector<cv::Point2f> corners_temp;

                        getCorners(gray_img, corners);
                        for (i = 0; i < corners.size(); i++)
                            if (checkNeighbourhood(binary_img, corners.at(i), blob) == true)
                            {
                                corners_temp.push_back(corners.at(i));
                            }

                        for (i = 0; i < right_corners.size(); i++)
                            findNearestPoint(right_corners.at(i), corners_temp);
                    }
                }

                if (g_set_identifiers == false)
                {
                    g_right_corners = right_corners;
                }
                else
                {
                    ROS_INFO("right_corners = g_right_corners;                executed");
                    right_corners = g_right_corners;
                    g_set_identifiers = false;
                    for (i = 0; i < right_corners.size(); i++)
                        ROS_INFO("x - %f, y - %f", right_corners[i].x, right_corners[i].y);
                }

                if (right_corners.size() != 0)
                {
                    int j;

                    for (j = 0; j < right_corners.size(); j++)
                    {
                        if (((right_corners.at(j)).x < 0) || ((right_corners.at(j)).x > src_img.cols) || ((right_corners.at(j)).y < 0) || ((right_corners.at(j)).y > src_img.rows))
                            start_tracking = false;
                        else
                        {
                            cv::circle(src_img, right_corners.at(j), 5, CV_RGB(0, 0, 255), 2);
                            std::stringstream ss;
                            ss << static_cast<char>(65 + j);

                            if (g_identified_corners == true)
                            {
                                cv::putText(src_img, ss.str().c_str(), right_corners.at(j), cv::FONT_HERSHEY_PLAIN,
                                            0.6, cv::Scalar(255, 255, 255), 1);
                                cv::solvePnP(g_box_coordinates, right_corners, right_intrinsic_mat, right_distortion_mat, rvec, tvec);
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

                                //ROS_INFO("X - %f, Y - %f, Z - %f", tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2));
                                //ROS_INFO("RX - %f, RY - %f, RZ - %f", rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));

                                ROS_INFO("Homogeneous Transformation Matrix - ");

                                for (int k = 0; k < 4; k++)
                                        ROS_INFO("%10.6f       %10.6f       %10.6f       %10.6f",
                                                 transformation_matrix.at<double>(k, 0), transformation_matrix.at<double>(k, 1),
                                                 transformation_matrix.at<double>(k, 2), transformation_matrix.at<double>(k, 3));

                                baxter_cams::transMatrix msg;

                                for (int k = 0; k < 4; k++)
                                    for (int l = 0; l < 4; l++)
                                        msg.tMat.push_back(transformation_matrix.at<double>(k, l));

                                trans_matrix_pub.publish(msg);
                            }
                        }
                    }

                    drawPolygon(src_img, right_corners);
                }

                imgA = imgB;
                cv::circle(src_img, cv::Point(src_img.cols / 2, src_img.rows / 2), 2, cv::Scalar(255, 255, 0), 2);
                cv::line(src_img, cv::Point(0, src_img.rows / 2), cv::Point(src_img.cols, src_img.rows / 2), cv::Scalar(255, 255, 0), 1);
                cv::line(src_img, cv::Point(src_img.cols / 2, 0), cv::Point(src_img.cols / 2, src_img.rows), cv::Scalar(255, 255, 0), 1);
                cv::imshow("cam1", src_img);                                                                        // Show the src_img with the corners
                cv::waitKey(10);
            }
        }
        catch (std::exception ex)
        {
            // Do nothing
        }
        loop_rate.sleep();
    }

    //state_control_baxter->disableRobot(nHandle);
    ros::shutdown();
}

