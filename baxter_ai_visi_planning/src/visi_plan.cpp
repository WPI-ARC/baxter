#include "visi_plan.h"

#define RAND_SAMPL  1
#define SIM_ANNEAL  2

#define ALG_USED    SIM_ANNEAL

double g_best_cost;
geometry_msgs::PoseStamped g_best_pose;

// Ctrl+C interrupt handler
void shutdown(int sig)
{
    g_shutdown = true;
}

void threadFunc()
{
    ros::Rate r(100);    // Nyquist Sampling frequency f >= 2 BW = 2 * frame_rate = 2 * 24 = 48
    while (ros::ok())
    {
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
    }
}

int moveToBetterVisibilityPosition(geometry_msgs::PoseStamped targetPos)
{
    robotBaxter->kinematic_state->setStateValues(baxter::BaxterMove::joint_state_msg);

    if (robotBaxter->checkIKexists(targetPos, RIGHT_ARM))
    {
        ROS_INFO("Moving now - \n\n");
        robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        robotBaxter->moveToTarget(targetPos, RIGHT_ARM);
        return 1;
    }
    else
        return 0;

}

// the cost function takes the number of ray trace intersects and the distance between the object and the camera position
// it uses the weighting parameter "lambda"
// coi - corners of interest detected
double cost_function(int number_of_intersecting_points, int number_of_overlapping_pts, cv::Point3f object_wrt_base, cv::Point3f cam_wrt_base, robot_state::JointStateGroup *joint_state_group, int coi)
{
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobian_transpose;
    Eigen::MatrixXd JJt;
    std::vector<double> lambda;
    double ray_thing;
    double manipulability;
    double distance_object_camera;
    double cost, lambda_c, alpha_c, beta_c, lambda_c2;

    lambda_c = 100;
    lambda_c2 = 200;
    alpha_c = 60;
    beta_c = 25;

    distance_object_camera = std::sqrt((std::pow(object_wrt_base.x - cam_wrt_base.x, 2)) + (std::pow(object_wrt_base.y - cam_wrt_base.y, 2))
                                       + (std::pow(object_wrt_base.z - cam_wrt_base.z, 2)));

    joint_state_group->getJacobian(joint_state_group->getJointModelGroup()->getLinkModelNames().back(),
                                   reference_point_position,
                                   jacobian);

    jacobian_transpose = jacobian.transpose();
    JJt = jacobian * jacobian_transpose;

    Eigen::EigenSolver<Eigen::MatrixXd> es(JJt);

    for (int i = 0; i < es.eigenvalues().size(); i++)
        lambda.push_back(es.eigenvalues()[i].real());

    manipulability = *std::min_element(lambda.begin(), lambda.end());

    ray_thing = ((double) (number_of_intersecting_points - number_of_overlapping_pts)) / ((double) number_of_intersecting_points);

    ROS_INFO("intersecting pts - %d", number_of_intersecting_points);
    ROS_INFO("overlapping pts - %d", number_of_overlapping_pts);

    distance_object_camera = std::exp(-distance_object_camera);

    cost = -(lambda_c*ray_thing + lambda_c2*(coi / 8)  + alpha_c*manipulability + beta_c*distance_object_camera);

    if (cost < g_best_cost)
    {
        ROS_INFO("g_best_cost - %10.5f, cost - %10.5f", g_best_cost, cost);
        g_best_cost = cost;
        ROS_INFO("g_best_cost - %10.5f, cost - %10.5f", g_best_cost, cost);
    }

    ROS_INFO("Cost - %f", cost);

    ROS_INFO("Cost function : -[ (ray_tracing_pts) + lambda*(min_eigs(JJt)) + lambda2*(dist_obj_cam) ]");
    ROS_INFO("Cost function : -[ lambda*(%f) + alpha*(%10.5f) - beta*(%10.5f) ]", ray_thing, manipulability, distance_object_camera);

    return cost;
}

cv::Point3f sendFOVToRviz(double &fovx, double &fovy, double &focal_length, geometry_msgs::PoseStamped &random_pose,
                   cv::Point3f box_position, ros::Publisher &marker_pub)
{
    visualization_msgs::Marker FOV;
    cv::Point3f ret_pose;

    FOV.header.frame_id = "/reference/base";
    FOV.header.stamp = ros::Time::now();
    FOV.ns = "FOV";
    FOV.id = 1;
    FOV.action = visualization_msgs::Marker::ADD;
    FOV.type = visualization_msgs::Marker::CUBE;

    FOV.scale.x = 0.25 * 2 * std::cos(fovx * 0.0174532925 / 2);
    FOV.scale.y = 0.25 * 2 * std::cos(fovy * 0.0174532925 / 2);
    FOV.scale.z = 0.025;
    FOV.color.a = 0.5;
    FOV.color.r = 0.66;
    FOV.color.b = 0.66;
    FOV.color.g = 0.66;
    FOV.pose.position.x = random_pose.pose.position.x;
    FOV.pose.position.y = random_pose.pose.position.y;
    FOV.pose.position.z = random_pose.pose.position.z;
    FOV.pose.orientation.x = random_pose.pose.orientation.x;
    FOV.pose.orientation.y = random_pose.pose.orientation.y;
    FOV.pose.orientation.z = random_pose.pose.orientation.z;
    FOV.pose.orientation.w = random_pose.pose.orientation.w;

    // Normal
    box_position.x = box_position.x - random_pose.pose.position.x;
    box_position.y = box_position.y - random_pose.pose.position.y;
    box_position.z = box_position.z - random_pose.pose.position.z;
    double magnitude = std::sqrt(std::pow(box_position.x, 2) + std::pow(box_position.y, 2) + std::pow(box_position.z, 2));
    box_position.x = box_position.x / magnitude;
    box_position.y = box_position.y / magnitude;
    box_position.z = box_position.z / magnitude;
    FOV.pose.position.x += box_position.x * 0.25;
    FOV.pose.position.y += box_position.y * 0.25;
    FOV.pose.position.z += box_position.z * 0.25;

    marker_pub.publish(FOV);

    ret_pose.x = FOV.pose.position.x;
    ret_pose.y = FOV.pose.position.y;
    ret_pose.z = FOV.pose.position.z;

    return ret_pose;
}

void compute_field_of_view(cv::Mat camera_matrix, double &fovx, double &fovy, double &focal_length)
{
    cv::Size        image_size;
    double          aperture_width, aperture_height, aspect_ratio;
    cv::Point2d     principal_point;

    image_size.width = 960;
    image_size.height = 600;
    aperture_width = 3.888;
    aperture_height = 2.430;

    cv::calibrationMatrixValues(camera_matrix, image_size, aperture_width, aperture_height, fovx, fovy, focal_length, principal_point, aspect_ratio);
}

// Generate the random pose in the task space of the robot's right hand gripper/camera
geometry_msgs::PoseStamped generateRandomPose(cv::Point3f target_pt_wrt_base_frame, tf::TransformListener *transform, int seed_state, geometry_msgs::PoseStamped prev_rand_pose)
{
    int temp_rand;
    geometry_msgs::PoseStamped ret_pose, tPos_base, v_cam_object;


    if ((seed_state == RAND_SAMPL) || (prev_rand_pose.header.frame_id == "null"))
    {
        ret_pose.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_lower_shoulder];
        temp_rand = (rand() % 2000);
        temp_rand = temp_rand - 1000;
        ret_pose.pose.position.x = (temp_rand * 1.0571) / 1000;
        temp_rand = (rand() % 2000);
        temp_rand = temp_rand - 1000;
        ret_pose.pose.position.y = (temp_rand * 1.0571) / 1000;
        temp_rand = (rand() % 2000);
        temp_rand = temp_rand - 1000;
        ret_pose.pose.position.z = (temp_rand * 1.0571) / 1000;

        ret_pose.pose.orientation.w = 1.0;
        ret_pose.pose.orientation.x = 0.0;
        ret_pose.pose.orientation.y = 0.0;
        ret_pose.pose.orientation.z = 0.0;

        try
        {
            tf::StampedTransform stamped_t;
            transform->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_lower_shoulder], ros::Time(0), stamped_t);
        }
        catch (std::exception ex)
        {
            transform->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_lower_shoulder], ros::Time::now(), ros::Duration(1.0));
        }

        transform->transformPose("base", ret_pose, tPos_base);
    }
    else if (seed_state == SIM_ANNEAL)
    {
        tPos_base.header.frame_id = "/reference/base";
        temp_rand = (rand() % 20);
        temp_rand = temp_rand - 10;
        tPos_base.pose.position.x = prev_rand_pose.pose.position.x + ((temp_rand * 0.05) / 10);
        temp_rand = (rand() % 20);
        temp_rand = temp_rand - 10;
        tPos_base.pose.position.y = prev_rand_pose.pose.position.y + ((temp_rand * 0.05) / 10);
        temp_rand = (rand() % 20);
        temp_rand = temp_rand - 10;
        tPos_base.pose.position.z = prev_rand_pose.pose.position.z + ((temp_rand * 0.05) / 10);
        tPos_base.pose.orientation.w = prev_rand_pose.pose.orientation.w;
        tPos_base.pose.orientation.x = prev_rand_pose.pose.orientation.x;
        tPos_base.pose.orientation.y = prev_rand_pose.pose.orientation.y;
        tPos_base.pose.orientation.z = prev_rand_pose.pose.orientation.z;
    }

    //tPos_base = ret_pose;

    v_cam_object.pose.position.x = -(target_pt_wrt_base_frame.x - tPos_base.pose.position.x);
    v_cam_object.pose.position.y = -(target_pt_wrt_base_frame.y - tPos_base.pose.position.y);
    v_cam_object.pose.position.z = (target_pt_wrt_base_frame.z - tPos_base.pose.position.z);
    v_cam_object.pose.orientation.w = 1.0;
    v_cam_object.pose.orientation.x = 0.0;
    v_cam_object.pose.orientation.y = 0.0;
    v_cam_object.pose.orientation.z = 0.0;

    double                  angle_of_rotation, magnitude_v_cam_object;
    cv::Point3f             axis_of_rotation_xyz;

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
    angle_of_rotation = std::acos(v_cam_object.pose.position.z);// - (M_PI / 2);
    ROS_INFO("Angle of rotation - %10.5f", angle_of_rotation);

    magnitude_v_cam_object = std::sqrt(std::pow(axis_of_rotation_xyz.x, 2) + std::pow(axis_of_rotation_xyz.y, 2));
    axis_of_rotation_xyz.x = axis_of_rotation_xyz.x / magnitude_v_cam_object;
    axis_of_rotation_xyz.y = axis_of_rotation_xyz.y / magnitude_v_cam_object;

    ret_pose.pose.position.x = tPos_base.pose.position.x;
    ret_pose.pose.position.y = tPos_base.pose.position.y;
    ret_pose.pose.position.z = tPos_base.pose.position.z;

    // Create quaternion
    ret_pose.pose.orientation.w = std::cos(angle_of_rotation / 2);
    ret_pose.pose.orientation.x = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.x;
    ret_pose.pose.orientation.y = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.y;
    ret_pose.pose.orientation.z = std::sin(angle_of_rotation / 2) * axis_of_rotation_xyz.z;

    ret_pose.header.frame_id = "/base";

    return ret_pose;
}

// the random_sampling algorithm samples the task space
void random_sampling()
{
    // Create the task space which is essentially the points in the sphere
    Vector a, b(2, 3), c(1, 2, 3);
    Point p1(1, 0, 0), p2(0, 1, 0), p3(1, 0, 1), p4, p5(0.0, 1.0, 1.0), I;
    Triangle T;
    Ray R;

    T.V0 = p1;
    T.V1 = p2;
    T.V2 = p3;

    R.P0 = p4;
    R.P1 = p5;

    int intersect = intersect3D_RayTriangle(R, T, &I);

    std::cout << "Intersect: " << intersect << " Point: " << I.x << " " << I.y << " " << I.z << "\n";

}

void addFOVTriangles(std::vector<Triangle> &fov_triangles, geometry_msgs::PoseStamped &cam_pt_wrt_base, cv::Point3f &fov_point, double &fovx, double &fovy)
{
    Triangle T;
    Point p1, p2, p3;
    cv::Mat rot_mat;
    double x_difference, y_difference;
    Point temp;

    rot_mat = getTransform(cam_pt_wrt_base);

    x_difference = 0.5 * (std::cos(fovx * 0.0174532925 / 2)) / 100;
    y_difference = 0.5 * (std::cos(fovy * 0.0174532925 / 2)) / 100;

    p1.x = ((-50) * x_difference);
    p1.y = ((-50) * y_difference);
    p1.z = 0;

    p2.x = ((-50) * x_difference);
    p2.y = ((50) * y_difference);
    p2.z = 0;

    p3.x = ((50) * x_difference);
    p3.y = ((50) * y_difference);
    p3.z = 0;

    temp = getTransformedPoint(rot_mat, p1);

    p1.x = fov_point.x + temp.x;
    p1.y = fov_point.y + temp.y;
    p1.z = fov_point.z + temp.z;

    temp = getTransformedPoint(rot_mat, p2);

    p2.x = fov_point.x + temp.x;
    p2.y = fov_point.y + temp.y;
    p2.z = fov_point.z + temp.z;

    temp = getTransformedPoint(rot_mat, p3);

    p3.x = fov_point.x + temp.x;
    p3.y = fov_point.y + temp.y;
    p3.z = fov_point.z + temp.z;

    T.V0 = p1;
    T.V1 = p2;
    T.V2 = p3;

    fov_triangles.push_back(T);

    p3.x = ((50) * x_difference);
    p3.y = ((-50) * y_difference);
    p3.z = 0;

    temp = getTransformedPoint(rot_mat, p3);

    p3.x = fov_point.x + temp.x;
    p3.y = fov_point.y + temp.y;
    p3.z = fov_point.z + temp.z;

    T.V2 = p3;

    fov_triangles.push_back(T);
}

int checkVertexVisibility(std::vector<Triangle> &triangles1, std::vector<Triangle> &fov_triangles, cv::Point3f &box_pt,
                          cv::Point3f &cam_pt, std::vector<Triangle> &triangles2, std::vector<Triangle> &triangles3, std::vector<Triangle> &triangles4)
{
    int intersect;
    Point I;
    Ray R;
    double dist, temp_dist;

    R.P0.x = cam_pt.x;
    R.P0.y = cam_pt.y;
    R.P0.z = cam_pt.z;

    R.P1.x = box_pt.x;
    R.P1.y = box_pt.y;
    R.P1.z = box_pt.z;

/*    dist = std::sqrt(std::pow(cam_pt.x - box_pt.x, 2) + std::pow(cam_pt.y - box_pt.y, 2) + std::pow(cam_pt.z - box_pt.z, 2));

    for (int i = 0; i < triangles1.size(); i++)
        if (intersect3D_RayTriangle(R, triangles1.at(i), &I) == 1)
        {
            temp_dist = std::sqrt(std::pow(cam_pt.x - I.x, 2) + std::pow(cam_pt.y - I.y, 2) + std::pow(cam_pt.z - I.z, 2));
            ROS_INFO("%d - %10.5f -- %10.5f --", i, temp_dist, dist);
            ROS_INFO("%10.5f, %10.5f, %10.5f --- %10.5f, %10.5f, %10.5f", box_pt.x, box_pt.y, box_pt.z, I.x, I.y, I.z);
            if (temp_dist < dist)
                return 0;
        }*/

    intersect = 0;

    for (int i = 0; i < triangles2.size(); i++)
        intersect += intersect3D_RayTriangle(R, triangles2.at(i), &I);

    if (intersect > 0)
        return 0;

    //for (int i = 0; i < triangles3.size(); i++)
    //    intersect += intersect3D_RayTriangle(R, triangles3.at(i), &I);

    //if (intersect > 0)
    //    return 0;

    for (int i = 0; i < triangles4.size(); i++)
        intersect += intersect3D_RayTriangle(R, triangles4.at(i), &I);

    if (intersect > 0)
        return 0;

    for (int i = 0; i < fov_triangles.size(); i++)
        intersect += intersect3D_RayTriangle(R, fov_triangles.at(i), &I);

    return intersect;
}

// Find out the points that intersect both the object and the obstacle
int rayTracingCollisions(std::vector<Ray> &r, std::vector<Triangle> &triangles1, std::vector<Triangle> &triangles2, std::vector<Triangle> &triangles3)
{
    // Triangles1 = triangles of the object mesh
    // Triangles2 = triangles of the obstacle mesh
    // Triangles3 = triangles of the baxter robot box mesh

    int intersect;
    Point I;
    int points_intersect;

    intersect = 0;

    for (int i = 0; i < r.size(); i++)
    {
        points_intersect = 0;
        for (int j = 0; j < triangles1.size(); j++)
            points_intersect += intersect3D_RayTriangle((Ray) r.at(i), (Triangle) triangles1.at(j), &I);
        if (points_intersect >= 2)
        {
            points_intersect = 0;
            for (int j = 0; j < triangles3.size(); j++)
                points_intersect += intersect3D_RayTriangle((Ray) r.at(i), (Triangle) triangles3.at(j), &I);
            if (points_intersect >= 2)
            {
                intersect++;
                continue;
            }
            points_intersect = 0;
            for (int j = 0; j < triangles2.size(); j++)
                points_intersect += intersect3D_RayTriangle((Ray) r.at(i), (Triangle) triangles2.at(j), &I);
            if (points_intersect >= 2)
                intersect++;
        }
    }

    return intersect;
}

// Find out the points that intersect only the object
int rayTracing(std::vector<Ray> &r, std::vector<Triangle> &triangles)
{
    int         intersect;
    Point       I;
    long int    points_intersect;

    intersect = 0;

    for (int i = 0; i < r.size(); i++)
    {
        points_intersect = 0;
        for (int j = 0; j < triangles.size(); j++)
            points_intersect += intersect3D_RayTriangle((Ray) r.at(i), (Triangle) triangles.at(j), &I);

        if (points_intersect >= 2)
            intersect++;
    }

    return intersect;
}

void printTriangle(Triangle T)
{
    ROS_INFO("Triangle:");
    ROS_INFO("POINT A  :  X - %10.5f, Y - %10.5f, Z - %10.5f  ;", T.V0.x, T.V0.y, T.V0.z);
    ROS_INFO("POINT B  :  X - %10.5f, Y - %10.5f, Z - %10.5f  ;", T.V1.x, T.V1.y, T.V2.z);
    ROS_INFO("POINT C  :  X - %10.5f, Y - %10.5f, Z - %10.5f  ;", T.V2.x, T.V2.y, T.V2.z);
}

cv::Mat getTransform(geometry_msgs::PoseStamped rand_pose)
{
    double qx, qy, qz, qw;
    cv::Mat rot_matrix(3, 3, cv::DataType<double>::type);

    qx = rand_pose.pose.orientation.x;
    qy = rand_pose.pose.orientation.y;
    qz = rand_pose.pose.orientation.z;
    qw = rand_pose.pose.orientation.w;

    rot_matrix.at<double>(0, 0) = (1 - 2*qy*qy - 2*qz*qz);
    rot_matrix.at<double>(0, 1) = (2*qx*qy - 2*qz*qw);
    rot_matrix.at<double>(0, 2) = (2*qx*qz + 2*qy*qw);
    rot_matrix.at<double>(1, 0) = (2*qx*qy + 2*qz*qw);
    rot_matrix.at<double>(1, 1) = (1 - 2*qx*qx - 2*qz*qz);
    rot_matrix.at<double>(1, 2) = (2*qy*qz - 2*qx*qw);
    rot_matrix.at<double>(2, 0) = (2*qx*qz - 2*qy*qw);
    rot_matrix.at<double>(2, 1) = (2*qy*qz + 2*qx*qw);
    rot_matrix.at<double>(2, 2) = (1 - 2*qx*qx - 2*qy*qy);

    return rot_matrix;
}

Point getTransformedPoint(cv::Mat rot_matrix, Point pt)
{
    Point ret_pt;

    ret_pt.x = rot_matrix.at<double>(0, 0) * pt.x + rot_matrix.at<double>(0, 1) * pt.y + rot_matrix.at<double>(0, 2) * pt.z;
    ret_pt.y = rot_matrix.at<double>(1, 0) * pt.x + rot_matrix.at<double>(1, 1) * pt.y + rot_matrix.at<double>(1, 2) * pt.z;
    ret_pt.z = rot_matrix.at<double>(2, 0) * pt.x + rot_matrix.at<double>(2, 1) * pt.y + rot_matrix.at<double>(2, 2) * pt.z;

    return ret_pt;
}

void constructObstacleTriangles(std::vector<Triangle> &triangles, shapes::Mesh *obstacle_mesh, geometry_msgs::Pose &obstacle_pose)
{
    for (int i = 0; i < obstacle_mesh->vertex_count; i++)
    {
        ROS_INFO("Vertex %d - %10.5f, %10.5f, %10.5f", i, obstacle_mesh->vertices[3*i], obstacle_mesh->vertices[3*i + 1], obstacle_mesh->vertices[3*i + 2]);
    }

    Triangle T;

    triangles.clear();

    for (int j = 0; j < obstacle_mesh->triangle_count; j++)
    {
        ROS_INFO("%d, %d, %d", obstacle_mesh->triangles[3*j], obstacle_mesh->triangles[3*j + 1], obstacle_mesh->triangles[3*j + 2]);

        T.V0.x = obstacle_pose.position.x + obstacle_mesh->vertices[3*obstacle_mesh->triangles[3*j]];
        T.V0.y = obstacle_pose.position.y + obstacle_mesh->vertices[3*obstacle_mesh->triangles[3*j] + 1];
        T.V0.z = obstacle_pose.position.z + obstacle_mesh->vertices[3*obstacle_mesh->triangles[3*j] + 2];

        T.V1.x = obstacle_pose.position.x + obstacle_mesh->vertices[3*obstacle_mesh->triangles[3*j + 1]];
        T.V1.y = obstacle_pose.position.y + obstacle_mesh->vertices[(3*obstacle_mesh->triangles[3*j + 1]) + 1];
        T.V1.z = obstacle_pose.position.z + obstacle_mesh->vertices[(3*obstacle_mesh->triangles[3*j + 1]) + 2];

        T.V2.x = obstacle_pose.position.x + obstacle_mesh->vertices[(3*obstacle_mesh->triangles[3*j + 2])];
        T.V2.y = obstacle_pose.position.y + obstacle_mesh->vertices[(3*obstacle_mesh->triangles[3*j + 2]) + 1];
        T.V2.z = obstacle_pose.position.z + obstacle_mesh->vertices[(3*obstacle_mesh->triangles[3*j + 2]) + 2];

        triangles.push_back(T);
    }

    ROS_INFO("Obstacle position: %10.5f, %10.5f, %10.5f", obstacle_pose.position.x, obstacle_pose.position.y, obstacle_pose.position.z);

    for (int i = 0; i < triangles.size(); i++)
    {
        ROS_INFO("Triangle %d V1 - %10.5f, %10.5f, %10.5f", i, triangles.at(i).V0.x, triangles.at(i).V0.y, triangles.at(i).V0.z);
        ROS_INFO("Triangle %d V2 - %10.5f, %10.5f, %10.5f", i, triangles.at(i).V1.x, triangles.at(i).V1.y, triangles.at(i).V1.z);
        ROS_INFO("Triangle %d V3 - %10.5f, %10.5f, %10.5f", i, triangles.at(i).V2.x, triangles.at(i).V2.y, triangles.at(i).V2.z);
    }
}

void constructCameraViewTriangles(std::vector<Ray> &r, std::vector<Triangle> &triangles, std::vector<cv::Point3f> box_pts_wrt_base,
                                  geometry_msgs::PoseStamped cam_pt_wrt_base, cv::Point3f fov_point, double fovx, double fovy, ros::Publisher &marker_pub)
{
    cv::Point3f temp_pt;
    Point A, B, C, D, temp;
    Ray ray;
    Triangle triangle;
    double x_difference, y_difference;
    cv::Mat rot_mat;

    A.x = 0.25 * (std::cos(fovx * 0.0174532925 / 2));
    A.y = 0.25 * (std::cos(fovy * 0.0174532925 / 2));
    A.z = 0.25;

    B.x = (0.25 * (-std::cos(fovx * 0.0174532925 / 2)));
    B.y = 0.25 * (std::cos(fovy * 0.0174532925 / 2));
    B.z = 0.25;

    C.x = 0.25 * (-std::cos(fovx * 0.0174532925 / 2));
    C.y = 0.25 * (-std::cos(fovy * 0.0174532925 / 2));
    C.z = 0.25;

    D.x = 0.25 * (std::cos(fovx * 0.0174532925 / 2));
    D.y = 0.25 * (-std::cos(fovx * 0.0174532925 / 2));
    D.z = 0.25;

    rot_mat = getTransform(cam_pt_wrt_base);

    x_difference = 0.5 * (std::cos(fovx * 0.0174532925 / 2)) / 100;
    y_difference = 0.5 * (std::cos(fovy * 0.0174532925 / 2)) / 100;

    visualization_msgs::Marker FOV;
    geometry_msgs::Point F_pt;

    FOV.header.frame_id = "/reference/base";
    FOV.header.stamp = ros::Time::now();
    FOV.ns = "FOV";
    FOV.id = 1;
    FOV.action = visualization_msgs::Marker::ADD;
    FOV.type = visualization_msgs::Marker::POINTS;

    FOV.scale.x = 0.01;
    FOV.scale.y = 0.01;
    FOV.scale.z = 0.01;
    FOV.color.a = 1.0;
    FOV.color.r = 0.66;
    FOV.color.b = 0.66;
    FOV.color.g = 0.66;
    FOV.pose.orientation.w = 1.0;

    r.clear();
    for (int i = 0; i < 100; i++)
        for (int j = 0; j < 100; j++)
        {
            ray.P0.x = cam_pt_wrt_base.pose.position.x;
            ray.P0.y = cam_pt_wrt_base.pose.position.y;
            ray.P0.z = cam_pt_wrt_base.pose.position.z;

            ray.P1.x = ((i - 50) * x_difference);
            ray.P1.y = ((j - 50) * y_difference);
            ray.P1.z = 0;

            temp = getTransformedPoint(rot_mat, ray.P1);

            ray.P1.x = fov_point.x + temp.x;
            ray.P1.y = fov_point.y + temp.y;
            ray.P1.z = fov_point.z + temp.z;

            F_pt.x = ray.P1.x;
            F_pt.y = ray.P1.y;
            F_pt.z = ray.P1.z;

            FOV.points.push_back(F_pt);

            r.push_back(ray);
        }

    marker_pub.publish(FOV);

    triangles.clear();
    for (int i = 0; i < 8; i++)
    {
        triangle.V0.x = box_pts_wrt_base.at(i).x;
        triangle.V0.y = box_pts_wrt_base.at(i).y;
        triangle.V0.z = box_pts_wrt_base.at(i).z;

        if (i == 7)
        {
            triangle.V1.x = box_pts_wrt_base.at(0).x;
            triangle.V1.y = box_pts_wrt_base.at(0).y;
            triangle.V1.z = box_pts_wrt_base.at(0).z;
        }
        else
        {
            triangle.V1.x = box_pts_wrt_base.at(i + 1).x;
            triangle.V1.y = box_pts_wrt_base.at(i + 1).y;
            triangle.V1.z = box_pts_wrt_base.at(i + 1).z;
        }

        if (i == 6)
        {
            triangle.V2.x = box_pts_wrt_base.at(0).x;
            triangle.V2.y = box_pts_wrt_base.at(0).y;
            triangle.V2.z = box_pts_wrt_base.at(0).z;
        }
        else if (i == 7)
        {
            triangle.V2.x = box_pts_wrt_base.at(1).x;
            triangle.V2.y = box_pts_wrt_base.at(1).y;
            triangle.V2.z = box_pts_wrt_base.at(1).z;
        }

        triangle.V2.x = box_pts_wrt_base.at(i + 2).x;
        triangle.V2.y = box_pts_wrt_base.at(i + 2).y;
        triangle.V2.z = box_pts_wrt_base.at(i + 2).z;

        triangles.push_back(triangle);
    }

    for (int i = 0; i < 4; i++)
    {
        int j;

        if (i == 2)
            j = 3;
        else if (i == 3)
            j = 4;
        else
            j = i;

        triangle.V0.x = box_pts_wrt_base.at(j).x;
        triangle.V0.y = box_pts_wrt_base.at(j).y;
        triangle.V0.z = box_pts_wrt_base.at(j).z;

        triangle.V1.x = box_pts_wrt_base.at(j + 2).x;
        triangle.V1.y = box_pts_wrt_base.at(j + 2).y;
        triangle.V1.z = box_pts_wrt_base.at(j + 2).z;

        triangle.V2.x = box_pts_wrt_base.at(j + 4).x;
        triangle.V2.y = box_pts_wrt_base.at(j + 4).y;
        triangle.V2.z = box_pts_wrt_base.at(j + 4).z;

        triangles.push_back(triangle);
    }
}

void reconstructProbableObjectSpace()
{
    // Will contain the code which reads in from the topic
}

void trans_mat_callback(visualization_msgs::Marker msg)
{
    geometry_msgs::Point temp;
    g_number_of_object_pts = msg.points.size();
    for (int i = 0; i < g_number_of_object_pts; i++)
    {
        temp = msg.points.at(i);
        g_object_pts_wrt_cam[i].x = temp.x;
        g_object_pts_wrt_cam[i].y = temp.y;
        g_object_pts_wrt_cam[i].z = temp.z;
    }
}

// Temperature schedule used is the exponential decay function
double calculateProbability(double energy_old, double energy_new)
{
    double          T0 = 100.0;
    double          A = 0.0691;
    static double   cur_temp = T0;
    static int      iterator = 0;

    cur_temp = T0*std::exp(-A*iterator);
    iterator++;

    ROS_INFO("Current Temperature - %10.5f, Iterator ---- %d", cur_temp, iterator);

    if (cur_temp < 0.001)
    {
        iterator = 0;
        return 100.0;
    }

    if (energy_new < energy_old)
        return 1.0;
    else if (energy_old == energy_new)
        return 0.0;
    else if (energy_new > energy_old)
        return (std::exp(-(energy_new - energy_old)/cur_temp));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visi_plan");
    ros::NodeHandle nHandle;

    cv::Mat         right_intrinsic_mat(3, 3, cv::DataType<double>::type);
    double          fovx, fovy, focal_length;
    double          energy_0 = 0.0, energy_new = 0.0;
    double          best_cost;

    for (int i = 0; i < 3; i++)
    {
        right_intrinsic_mat.at<double>(i, 0) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3)];
        right_intrinsic_mat.at<double>(i, 1) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 1];
        right_intrinsic_mat.at<double>(i, 2) = baxter_camera::intrinsic_matrix::right_hand_camera[(i * 3) + 2];
    }

    cv::Point3f centroid_wrt_base;
    geometry_msgs::PoseStamped random_pose, last_valid_pose;

    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    /* Get a shared pointer to the model */
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    /* Get the configuration for the joints in the right arm of the PR2*/
    robot_state::JointStateGroup* right_joint_state_group = kinematic_state->getJointStateGroup("right_arm");
    robot_state::JointStateGroup* left_joint_state_group  = kinematic_state->getJointStateGroup("left_arm");
    robot_state::JointStateGroup* both_joint_state_group  = kinematic_state->getJointStateGroup("both_arms");

    planning_scene::PlanningScene planning_scene(kinematic_model);  // Collision (Planning Scene which contains the objects in the world)
    collision_detection::CollisionRequest collision_request;        // Collision request
    collision_detection::CollisionResult collision_result;          // Collision result

    robotBaxter = new baxter::BaxterMove();
    robotBaxter->subscribeJointStates(nHandle);

    signal(SIGINT, shutdown);

    transform_listener = new tf::TransformListener();

    ros::Subscriber sub = nHandle.subscribe("/visualization_marker", 1, trans_mat_callback);

    boost::thread spinning_thread(threadFunc);
    ros::ServiceClient service_client = nHandle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
    while(!service_client.exists())
    {
        ROS_INFO("Waiting for service");
        sleep(1.0);
    }

    ros::Publisher robot_state_publisher = nHandle.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 0, true);
    ros::Publisher marker_pub = nHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher marker_pub2 = nHandle.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
    ros::Publisher collision_object_publisher = nHandle.advertise<moveit_msgs::CollisionObject>("collision_object", 100);           // Collision - Publisher to the collision_object topic
    ros::Publisher marker_pub3 = nHandle.advertise<visualization_msgs::Marker>("visualization_marker3", 10);

    moveit_msgs::DisplayRobotState msg;

    transform_listener->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time::now(), ros::Duration(1.0));

    std::vector<Ray>            R;
    std::vector<Triangle>       triangles1, triangles2, fov_triangles, table_triangles, baxter_triangles;
    std::vector<cv::Point3f>    box_pts_wrt_base;

    shapes::Box         obstacle_shape(0.27, 0.34, 0.34), table_obstacle_shape(0.5, 1.21, 0.025), baxter_obstacle_shape(0.3, 0.6, 1.7);
    shapes::Mesh        *obstacle_mesh, *table_obstacle_mesh, *baxter_obstacle_mesh;
    shapes::ShapeMsg    obstacle_mesh_msg, table_obstacle_mesh_msg, baxter_obstacle_mesh_msg;
    shape_msgs::Mesh    obs_mesh_msg, table_obs_mesh_msg, baxter_obs_mesh_msg;
    visualization_msgs::Marker marker3;

    obstacle_mesh_msg = obs_mesh_msg;
    obstacle_mesh = shapes::createMeshFromShape(&obstacle_shape);

    table_obstacle_mesh_msg = table_obs_mesh_msg;
    table_obstacle_mesh = shapes::createMeshFromShape(&table_obstacle_shape);

    baxter_obstacle_mesh_msg = baxter_obs_mesh_msg;
    baxter_obstacle_mesh = shapes::createMeshFromShape(&baxter_obstacle_shape);

    moveit_msgs::CollisionObject collision_object, table_collision_object, baxter_collision_object;

    collision_object.header.frame_id = "/reference/base";
    collision_object.id = "obstacle_mesh";
    geometry_msgs::Pose pose_obstacle;
    pose_obstacle.position.x = 0.65;
    pose_obstacle.position.y = -0.2;
    pose_obstacle.position.z = 0.05;
    pose_obstacle.orientation.x = 0.0;
    pose_obstacle.orientation.y = 0.0;
    pose_obstacle.orientation.z = 0.0;
    pose_obstacle.orientation.w = 1.0;

    shapes::constructMsgFromShape(obstacle_mesh, obstacle_mesh_msg);
    obs_mesh_msg = boost::get<shape_msgs::Mesh>(obstacle_mesh_msg);
    shape_tools::constructMarkerFromShape(obs_mesh_msg, marker3);

    marker3.color.b = 1.0;
    marker3.color.a = 0.8;
    marker3.header.frame_id = "/reference/base";
    marker3.pose = pose_obstacle;

    collision_object.meshes.push_back(obs_mesh_msg);
    collision_object.mesh_poses.push_back(pose_obstacle);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    constructObstacleTriangles(triangles2, obstacle_mesh, pose_obstacle);

    table_collision_object.header.frame_id = "/reference/base";
    table_collision_object.id = "table_obstacle_mesh";
    geometry_msgs::Pose table_pose_obstacle;
    table_pose_obstacle.position.x = 0.8;
    table_pose_obstacle.position.y = 0.0;
    table_pose_obstacle.position.z = -0.14;
    table_pose_obstacle.orientation.x = 0.0;
    table_pose_obstacle.orientation.y = 0.0;
    table_pose_obstacle.orientation.z = 0.0;
    table_pose_obstacle.orientation.w = 1.0;

    shapes::constructMsgFromShape(table_obstacle_mesh, table_obstacle_mesh_msg);
    table_obs_mesh_msg = boost::get<shape_msgs::Mesh>(table_obstacle_mesh_msg);
    //shape_tools::constructMarkerFromShape(table_obs_mesh_msg, )

    table_collision_object.meshes.push_back(table_obs_mesh_msg);
    table_collision_object.mesh_poses.push_back(table_pose_obstacle);
    table_collision_object.operation = moveit_msgs::CollisionObject::ADD;

    constructObstacleTriangles(table_triangles, table_obstacle_mesh, table_pose_obstacle);

    baxter_collision_object.header.frame_id = "/reference/base";
    baxter_collision_object.id = "baxter_obstacle_mesh";
    geometry_msgs::Pose baxter_pose_obstacle;
    baxter_pose_obstacle.position.x = 0.0;
    baxter_pose_obstacle.position.y = 0.0;
    baxter_pose_obstacle.position.z = 0.0;
    baxter_pose_obstacle.orientation.x = 0.0;
    baxter_pose_obstacle.orientation.y = 0.0;
    baxter_pose_obstacle.orientation.z = 0.0;
    baxter_pose_obstacle.orientation.w = 1.0;

    shapes::constructMsgFromShape(baxter_obstacle_mesh, baxter_obstacle_mesh_msg);
    baxter_obs_mesh_msg = boost::get<shape_msgs::Mesh>(baxter_obstacle_mesh_msg);

    table_collision_object.meshes.push_back(table_obs_mesh_msg);
    table_collision_object.mesh_poses.push_back(table_pose_obstacle);
    table_collision_object.operation = moveit_msgs::CollisionObject::ADD;

    constructObstacleTriangles(baxter_triangles, baxter_obstacle_mesh, baxter_pose_obstacle);

    srand(time(NULL));

    while (ros::ok())
    {
        g_best_cost = 0.0;
        energy_0 = 0.0;
        // Publish the object on the collision_object topic and MoveIt will take it into account during collision checking while generating the IK solution!
        collision_object_publisher.publish(collision_object);
        collision_object_publisher.publish(table_collision_object);
        marker_pub3.publish(marker3);

        if (g_number_of_object_pts > 1)
        {
            geometry_msgs::PoseStamped tPos_link, tPos_base;

            tPos_link.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
            tPos_link.pose.position.x = g_object_pts_wrt_cam.at(1).x;
            tPos_link.pose.position.y = g_object_pts_wrt_cam.at(1).y;
            tPos_link.pose.position.z = g_object_pts_wrt_cam.at(1).z;
            tPos_link.pose.orientation.w = 1.0;
            tPos_link.pose.orientation.x = 0.0;
            tPos_link.pose.orientation.y = 0.0;
            tPos_link.pose.orientation.z = 0.0;

            try
            {
                tf::StampedTransform stamped_t;
                transform_listener->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time(0), stamped_t);
            }
            catch (std::exception ex)
            {
                transform_listener->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time::now(), ros::Duration(1.0));
            }

            transform_listener->transformPose("/reference/base", tPos_link, tPos_base);

            centroid_wrt_base.x = tPos_base.pose.position.x;
            centroid_wrt_base.y = tPos_base.pose.position.y;
            centroid_wrt_base.z = tPos_base.pose.position.z;

            box_pts_wrt_base.clear();
            for (int k = 0; k < g_object_pts_wrt_cam.size(); k++)
            {
                cv::Point3f pt;

                tPos_link.header.frame_id = "/reference" + baxter_links::link_names[baxter_links::right_hand_camera];
                tPos_link.pose.position.x = g_object_pts_wrt_cam.at(k).x;
                tPos_link.pose.position.y = g_object_pts_wrt_cam.at(k).y;
                tPos_link.pose.position.z = g_object_pts_wrt_cam.at(k).z;
                tPos_link.pose.orientation.w = 1.0;
                tPos_link.pose.orientation.x = 0.0;
                tPos_link.pose.orientation.y = 0.0;
                tPos_link.pose.orientation.z = 0.0;

                try
                {
                    tf::StampedTransform stamped_t;
                    transform_listener->lookupTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time(0), stamped_t);
                }
                catch (std::exception ex)
                {
                    transform_listener->waitForTransform("/reference/base", "/reference" + baxter_links::link_names[baxter_links::right_hand_camera], ros::Time::now(), ros::Duration(1.0));
                }

                transform_listener->transformPose("/reference/base", tPos_link, tPos_base);

                pt.x = tPos_base.pose.position.x;
                pt.y = tPos_base.pose.position.y;
                pt.z = tPos_base.pose.position.z;

                box_pts_wrt_base.push_back(pt);
            }

            ROS_INFO("pts wrt base - %lu %lu", box_pts_wrt_base.size(), g_object_pts_wrt_cam.size());
            char ml;
            std::cin >> ml;


            ROS_INFO("Centroid - X - %10.5f, Y - %10.5f, Z - %10.5f", centroid_wrt_base.x, centroid_wrt_base.y, centroid_wrt_base.z);

            last_valid_pose.header.frame_id = "null";

            while (ros::ok())
            {                
                compute_field_of_view(right_intrinsic_mat, fovx, fovy, focal_length);
                ROS_INFO("fovx = %10.5f, fovy = %10.5f, focal_length = %10.5f", fovx, fovy, focal_length);

                // Setting the frame id to null tells the random pose generator that
                random_pose = generateRandomPose(centroid_wrt_base, transform_listener, ALG_USED, last_valid_pose);

                ROS_INFO("RANDOM POSE:");
                ROS_INFO("Position - %10.5f, %10.5f, %10.5f", random_pose.pose.position.x, random_pose.pose.position.y, random_pose.pose.position.z);
                ROS_INFO("Orientation - %10.5f, %10.5f, %10.5f, %10.5f", random_pose.pose.orientation.w, random_pose.pose.orientation.x, random_pose.pose.orientation.y, random_pose.pose.orientation.z);

                moveit_msgs::GetPositionIK::Request service_request;
                moveit_msgs::GetPositionIK::Response service_response;
                service_request.ik_request.group_name = "right_arm";
                service_request.ik_request.pose_stamped.header.frame_id = random_pose.header.frame_id;
                service_request.ik_request.pose_stamped.pose.position.x = random_pose.pose.position.x;
                service_request.ik_request.pose_stamped.pose.position.y = random_pose.pose.position.y;
                service_request.ik_request.pose_stamped.pose.position.z = random_pose.pose.position.z;
                service_request.ik_request.pose_stamped.pose.orientation.x = random_pose.pose.orientation.x;
                service_request.ik_request.pose_stamped.pose.orientation.y = random_pose.pose.orientation.y;
                service_request.ik_request.pose_stamped.pose.orientation.z = random_pose.pose.orientation.z;
                service_request.ik_request.pose_stamped.pose.orientation.w = random_pose.pose.orientation.w;

                service_request.ik_request.avoid_collisions = true;

                bool valid = service_client.call(service_request, service_response);

                ROS_INFO_STREAM("Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ") << service_response.error_code.val);

                if (service_response.error_code.val == service_response.error_code.SUCCESS)
                {
                    int intersecting_pts, intersecting_pts2;

                    ROS_INFO("Valid IK!");
                    //right_joint_state_group->setVariableValues(service_response.solution.joint_state.position);
                    for (int k = 0; k < service_response.solution.joint_state.name.size(); k++)
                        ROS_INFO("Joint: %s - %10.5f", ((std::string) service_response.solution.joint_state.name.at(k)).c_str(), service_response.solution.joint_state.position.at(k));
                    both_joint_state_group->setVariableValues(service_response.solution.joint_state);
                    /* get a robot state message describing the pose in kinematic_state */

                    //collision_result.clear();
                    //planning_scene.checkCollision(collision_request, collision_result, *kinematic_state);

                    //std::cout << "Collision : " << collision_result.collision;

                    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
                    /* send the message to the RobotState display */
                    robot_state_publisher.publish(msg);

                    cv::Point3f cam_pt_wrt_base, fov_pt_wrt_base;
                    cam_pt_wrt_base.x = random_pose.pose.position.x;
                    cam_pt_wrt_base.y = random_pose.pose.position.y;
                    cam_pt_wrt_base.z = random_pose.pose.position.z;

                    fov_pt_wrt_base = sendFOVToRviz(fovx, fovy, focal_length, random_pose, centroid_wrt_base, marker_pub);

                    constructCameraViewTriangles(R, triangles1, box_pts_wrt_base, random_pose, fov_pt_wrt_base, fovx, fovy, marker_pub2);
                    fov_triangles.clear();
                    addFOVTriangles(fov_triangles, random_pose, fov_pt_wrt_base, fovx, fovy);
                    int cou = 0;
                    for (int k = 0; k < 8; k++)
                    {
                        if (checkVertexVisibility(triangles1, fov_triangles, box_pts_wrt_base.at(k), cam_pt_wrt_base, triangles2, table_triangles, baxter_triangles) != 0)
                        {
                            ROS_INFO("Check ------------ %d", k);
                            cou++;
                        }
                    }
                    intersecting_pts = rayTracing(R, triangles1);
                    //intersecting_pts2 = rayTracing(R, triangles2);
                    intersecting_pts2 = rayTracingCollisions(R, triangles1, triangles2, baxter_triangles);
                    ROS_INFO("Ray tracing - %d", intersecting_pts);
                    ROS_INFO("Ray tracing - %d", intersecting_pts2);
                    ROS_INFO("Cou - %d", cou);
                    energy_new = cost_function(intersecting_pts, intersecting_pts2, centroid_wrt_base, cam_pt_wrt_base, right_joint_state_group, cou);

                    double probability = calculateProbability(energy_0, energy_new);

                    ROS_INFO("Probability -------------- %f", probability);

                    // Sim Annealing iteration done!
                    if (probability == 100.0)
                    {
                        service_request.ik_request.group_name = "right_arm";
                        service_request.ik_request.pose_stamped.header.frame_id = g_best_pose.header.frame_id;
                        service_request.ik_request.pose_stamped.pose.position.x = g_best_pose.pose.position.x;
                        service_request.ik_request.pose_stamped.pose.position.y = g_best_pose.pose.position.y;
                        service_request.ik_request.pose_stamped.pose.position.z = g_best_pose.pose.position.z;
                        service_request.ik_request.pose_stamped.pose.orientation.x = g_best_pose.pose.orientation.x;
                        service_request.ik_request.pose_stamped.pose.orientation.y = g_best_pose.pose.orientation.y;
                        service_request.ik_request.pose_stamped.pose.orientation.z = g_best_pose.pose.orientation.z;
                        service_request.ik_request.pose_stamped.pose.orientation.w = g_best_pose.pose.orientation.w;
                        service_request.ik_request.avoid_collisions = true;

                        service_client.call(service_request, service_response);

                        if (service_response.error_code.val == service_response.error_code.SUCCESS)
                        {
                            both_joint_state_group->setVariableValues(service_response.solution.joint_state);
                            robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
                            robot_state_publisher.publish(msg);
                        }

                        ROS_INFO("Min energy state - %10.5f", g_best_cost);

                        if (g_best_cost > -150.0)
                            break;

                        char h;
                        std::cin >> h;

                        if (h == 'y')
                            if (moveToBetterVisibilityPosition(random_pose) == 0)
                                ROS_INFO("No movement");
                            else
                                ROS_INFO("Moving");

                        break;
                    }

                    if (probability == 1.0)
                        energy_0 = energy_new;
                    else if (probability > 0.5)
                    {
                        ROS_INFO("PROBABILITY < 0.5 == %10.5f", probability);
                        energy_0 = energy_new;
                    }

                    /*if (cou == 8)
                    {
                        ROS_INFO("Best cost till now - %10.5f", g_best_cost);
                        char y;
                        std::cin >> y;
                        if (y == 'y')
                            if (moveToBetterVisibilityPosition(random_pose) == 0)
                                ROS_INFO("No movement");
                            else
                                ROS_INFO("Moving");
                    }*/

                    if (energy_0 == energy_new)
                    {
                        last_valid_pose.header.frame_id = random_pose.header.frame_id;
                        last_valid_pose.pose.position.x = random_pose.pose.position.x;
                        last_valid_pose.pose.position.y = random_pose.pose.position.y;
                        last_valid_pose.pose.position.z = random_pose.pose.position.z;
                        last_valid_pose.pose.orientation.w = random_pose.pose.orientation.w;
                        last_valid_pose.pose.orientation.x = random_pose.pose.orientation.x;
                        last_valid_pose.pose.orientation.y = random_pose.pose.orientation.y;
                        last_valid_pose.pose.orientation.z = random_pose.pose.orientation.z;

                        ROS_INFO("%10.5f, %10.5f, %10.5f - last valid pose", last_valid_pose.pose.position.x, last_valid_pose.pose.position.y, last_valid_pose.pose.position.z);
                    }

                    if (energy_new == g_best_cost)
                    {
                        g_best_pose.header.frame_id = random_pose.header.frame_id;
                        g_best_pose.pose.position.x = random_pose.pose.position.x;
                        g_best_pose.pose.position.y = random_pose.pose.position.y;
                        g_best_pose.pose.position.z = random_pose.pose.position.z;
                        g_best_pose.pose.orientation.w = random_pose.pose.orientation.w;
                        g_best_pose.pose.orientation.x = random_pose.pose.orientation.x;
                        g_best_pose.pose.orientation.y = random_pose.pose.orientation.y;
                        g_best_pose.pose.orientation.z = random_pose.pose.orientation.z;

                        ROS_INFO("Least energy till now - %f", g_best_cost);

                        //char h;
                        //std::cin >> h;
                    }
                }
                else
                    ROS_INFO("INVALID IK!");
            }
        }
    }

    return 0;
}

  //<node name="ik_test_sim" pkg="baxter_cams1" type="ik_test_sim" respawn="false" output="screen" />
