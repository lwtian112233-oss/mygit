#include <iostream>
#include <opencv2/opencv.hpp>
#include "lane_detector.h"

int main(int argc, char** argv) {
    std::string img_path = "test.jpg";
    if (argc > 1) img_path = argv[1];
    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::cerr << "Failed to open image: " << img_path << std::endl;
        return 2;
    }
    LaneDetector::DetectorConfig cfg;
    cfg.lane_width_m = 3.7; // typical lane width
    LaneDetector detector(cfg);
    double out_m = std::numeric_limits<double>::quiet_NaN();
    double off_px = detector.detectAndDraw(img, &out_m);
    int lane_px = detector.getLastLanePixelWidth();
    double pxpm = detector.getLastPixelsPerMeterUsed();
    std::cout << "offset_px=" << off_px << " offset_m=" << out_m << " lane_px=" << lane_px << " px_per_m=" << pxpm << std::endl;
    if (pxpm > 0.0) return 0;
    // fallback success if lane_px detected
    if (lane_px > 0) return 0;
    return 3;
}
