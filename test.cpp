#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "apriltags/gridDetector.hpp"

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75)
{
    cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}

void findAprilgridCorners(const cv::Mat &srcGray, std::vector<cv::Point2d> &imagePoints_p2d, std::vector<cv::Point3d> &objectPoints_p3d)
{
    double length = 0.088;
    double ratio = 0.3;
    calibration_toolkit::AprilgridDetector detector(length, ratio);
    Eigen::MatrixXd imagePoints_m2d;
    std::vector<bool> imagePoints_flags;
    detector.computeObservation(srcGray, imagePoints_m2d, imagePoints_flags);
    detector.convertResults_OpenCV(imagePoints_m2d, imagePoints_flags, objectPoints_p3d, imagePoints_p2d);
}

std::string cvWindowName = "Aprilgrid Detector Example";

int main(int argc, char *argv[])
{
    cv::Mat imageRaw = cv::imread("test.jpg");
    cv::Mat imageGray;
    cv::Mat imageDisplay;
    if (imageRaw.channels() != 1)
        cv::cvtColor(imageRaw, imageGray, cv::COLOR_BGR2GRAY);
    else
        imageRaw.copyTo(imageGray);

    std::vector<cv::Point2d> p2ds;
    std::vector<cv::Point3d> p3ds;
    findAprilgridCorners(imageGray, p2ds, p3ds);

    for (size_t i = 0; i < p2ds.size(); i++)
        cv::circle(imageRaw, p2ds[i], 3, CV_RGB(255, 0, 0), 1);

    display_resize(imageRaw, imageDisplay);
    cv::imshow(cvWindowName, imageDisplay);
    cv::waitKey(0);
    return 0;
}
