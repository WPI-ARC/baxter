#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "baxter_cams/transMatrix.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "vector"
//#include "Eigen/Dense"
#include "eigen3/Eigen/Dense"

#include "baxter_move.h"
#include "boost/thread.hpp"

#include "signal.h"
#include "exception"

baxter::BaxterCam*              cam_baxter;
baxter::BaxterStateControl*     state_control_baxter;
bool                            g_shutdown;
std::vector<cv::Point2f>        g_point_identifiers;
std::vector<cv::Point2f>        g_right_corners;
bool                            g_identified_corners;
bool                            g_set_identifiers;

std::vector<cv::Point3f>        g_box_coordinates;

int threshold_value = 0;
int threshold_type = 3;

void threadFunc();
void getCorners(cv::Mat &gray_image, std::vector<cv::Point2f> &corners);
size_t getBiggestBlob(cv::Mat &binary_image, std::vector<cv::Point> &blob);
double absoluteVal(double x);
bool checkNeighbourhood(cv::Mat &binary_image, cv::Point2f &corners, std::vector<cv::Point> &blob);
void findNearestNeighbours(cv::Point2f n_Point, std::vector<cv::Point2f> &corners);
void drawPolygon(cv::Mat &image, std::vector<cv::Point2f> &corners);
void mouseCallback(int event, int x, int y, int flags, void* param);
void shutdown(int sig);
cv::Point2f findNearestPoint(cv::Point2f &right_corner, std::vector<cv::Point2f> &corners_temp);
void setBoxCoords(void);
