#include <iostream>
#include <opencv2/opencv.hpp>
#include "lane_detector.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <image|video path or 0 for webcam>\n";
        return 1;
    }

    std::string path = argv[1];
    // If user passed "0" we open webcam
    if (path == "0") {
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) { std::cerr << "Cannot open webcam\n"; return 1; }
            LaneDetector::DetectorConfig cfg;
            LaneDetector detector(cfg);
        cv::Mat frame;
        while (cap.read(frame)) {
                double offset_m = 0.0;
                double offset = detector.detectAndDraw(frame, &offset_m);
            cv::imshow("lane_detector", frame);
            if (cv::waitKey(1) == 27) break; // ESC
        }
        return 0;
    }

    // Try opening as video first
    cv::VideoCapture cap(path);
    if (cap.isOpened()) {
            LaneDetector::DetectorConfig cfg;
            LaneDetector detector(cfg);
        cv::Mat frame;
        while (cap.read(frame)) {
                double offset_m = 0.0;
                double offset = detector.detectAndDraw(frame, &offset_m);
            cv::imshow("lane_detector", frame);
            if (cv::waitKey(1) == 27) break;
        }
        return 0;
    }

    // Fallback to single image
    cv::Mat img = cv::imread(path);
    if (img.empty()) { std::cerr << "Cannot open file: " << path << "\n"; return 1; }
        LaneDetector::DetectorConfig cfg;
        LaneDetector detector(cfg);
     double offset_m = 0.0;
     double offset = detector.detectAndDraw(img, &offset_m);
    cv::imshow("lane_detector", img);
    cv::waitKey(0);
    return 0;
}
