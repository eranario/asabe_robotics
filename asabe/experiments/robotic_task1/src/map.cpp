#include "opencv2/opencv.hpp"


int main() {
    cv::Mat map(500, 500, CV_8UC3, cv::Scalar(255,255,255));
    cv::circle(map, {100, 100}, 5, cv::Scalar(0, 255, 0), -1); // cart
    cv::imshow("Map", map);
    cv::waitKey(0);
    return 0;
}
