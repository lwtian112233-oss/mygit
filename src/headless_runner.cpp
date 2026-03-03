#include <iostream>
#include <opencv2/opencv.hpp>
#include "lane_detector.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input image> <output image> [--print-offset]\n";
        return 1;
    }
    std::string in = argv[1];
    std::string out = argv[2];
    bool print_offset = false;
    std::string config_path;
    std::string offset_file;
    double cli_lane_width = 0.0;
    double cli_pixels_per_meter = 0.0;
    // temporary CLI overrides (unset indicated by NaN or -1)
    double tmp_src_top_left_x = std::numeric_limits<double>::quiet_NaN();
    double tmp_src_top_right_x = std::numeric_limits<double>::quiet_NaN();
    double tmp_src_top_y = std::numeric_limits<double>::quiet_NaN();
    double tmp_src_bottom_left_x = std::numeric_limits<double>::quiet_NaN();
    double tmp_src_bottom_right_x = std::numeric_limits<double>::quiet_NaN();
    double tmp_dst_left_x = std::numeric_limits<double>::quiet_NaN();
    double tmp_dst_right_x = std::numeric_limits<double>::quiet_NaN();
    int tmp_canny_low = -1;
    int tmp_canny_high = -1;
    int tmp_hough_threshold = -1;
    int tmp_hough_min_line_len = -1;
    int tmp_hough_max_line_gap = -1;
    double tmp_smoothing_alpha = std::numeric_limits<double>::quiet_NaN();
    for (int i=3;i<argc;i++) {
        std::string a = argv[i];
        if (a == "--print-offset") print_offset = true;
        else if (a == "--config" && i+1<argc) { config_path = argv[++i]; }
        else if (a == "--offset-file" && i+1<argc) { offset_file = argv[++i]; }
        else if (a == "--lane-width" && i+1<argc) { cli_lane_width = atof(argv[++i]); }
        else if (a == "--pixels-per-meter" && i+1<argc) { cli_pixels_per_meter = atof(argv[++i]); }
        else if (a == "--src-top-left-x" && i+1<argc) { tmp_src_top_left_x = atof(argv[++i]); }
        else if (a == "--src-top-right-x" && i+1<argc) { tmp_src_top_right_x = atof(argv[++i]); }
        else if (a == "--src-top-y" && i+1<argc) { tmp_src_top_y = atof(argv[++i]); }
        else if (a == "--src-bottom-left-x" && i+1<argc) { tmp_src_bottom_left_x = atof(argv[++i]); }
        else if (a == "--src-bottom-right-x" && i+1<argc) { tmp_src_bottom_right_x = atof(argv[++i]); }
        else if (a == "--dst-left-x" && i+1<argc) { tmp_dst_left_x = atof(argv[++i]); }
        else if (a == "--dst-right-x" && i+1<argc) { tmp_dst_right_x = atof(argv[++i]); }
        else if (a == "--canny-low" && i+1<argc) { tmp_canny_low = atoi(argv[++i]); }
        else if (a == "--canny-high" && i+1<argc) { tmp_canny_high = atoi(argv[++i]); }
        else if (a == "--hough-threshold" && i+1<argc) { tmp_hough_threshold = atoi(argv[++i]); }
        else if (a == "--hough-min-line-len" && i+1<argc) { tmp_hough_min_line_len = atoi(argv[++i]); }
        else if (a == "--hough-max-line-gap" && i+1<argc) { tmp_hough_max_line_gap = atoi(argv[++i]); }
        else if (a == "--smoothing-alpha" && i+1<argc) { tmp_smoothing_alpha = atof(argv[++i]); }
    }
    cv::Mat img = cv::imread(in);
    if (img.empty()) { std::cerr << "Cannot open file: " << in << "\n"; return 1; }
    LaneDetector::DetectorConfig cfg;
    if (!config_path.empty()) {
        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["src_top_left_x"] >> cfg.src_top_left_x;
            fs["src_top_right_x"] >> cfg.src_top_right_x;
            fs["src_top_y"] >> cfg.src_top_y;
            fs["src_bottom_left_x"] >> cfg.src_bottom_left_x;
            fs["src_bottom_right_x"] >> cfg.src_bottom_right_x;
            fs["dst_left_x"] >> cfg.dst_left_x;
            fs["dst_right_x"] >> cfg.dst_right_x;
            fs["smoothing_alpha"] >> cfg.smoothing_alpha;
            fs["canny_low"] >> cfg.canny_low;
            fs["canny_high"] >> cfg.canny_high;
            fs["hough_threshold"] >> cfg.hough_threshold;
            fs["hough_min_line_len"] >> cfg.hough_min_line_len;
            fs["hough_max_line_gap"] >> cfg.hough_max_line_gap;
            fs.release();
        }
    }
    // apply CLI overrides
    if (!std::isnan(tmp_src_top_left_x)) cfg.src_top_left_x = tmp_src_top_left_x;
    if (!std::isnan(tmp_src_top_right_x)) cfg.src_top_right_x = tmp_src_top_right_x;
    if (!std::isnan(tmp_src_top_y)) cfg.src_top_y = tmp_src_top_y;
    if (!std::isnan(tmp_src_bottom_left_x)) cfg.src_bottom_left_x = tmp_src_bottom_left_x;
    if (!std::isnan(tmp_src_bottom_right_x)) cfg.src_bottom_right_x = tmp_src_bottom_right_x;
    if (!std::isnan(tmp_dst_left_x)) cfg.dst_left_x = tmp_dst_left_x;
    if (!std::isnan(tmp_dst_right_x)) cfg.dst_right_x = tmp_dst_right_x;
    if (tmp_canny_low >= 0) cfg.canny_low = tmp_canny_low;
    if (tmp_canny_high >= 0) cfg.canny_high = tmp_canny_high;
    if (tmp_hough_threshold >= 0) cfg.hough_threshold = tmp_hough_threshold;
    if (tmp_hough_min_line_len >= 0) cfg.hough_min_line_len = tmp_hough_min_line_len;
    if (tmp_hough_max_line_gap >= 0) cfg.hough_max_line_gap = tmp_hough_max_line_gap;
    if (!std::isnan(tmp_smoothing_alpha)) cfg.smoothing_alpha = tmp_smoothing_alpha;
    if (cli_lane_width > 0.0) cfg.lane_width_m = cli_lane_width;
    if (cli_pixels_per_meter > 0.0) cfg.pixels_per_meter = cli_pixels_per_meter;
    // If config file provided, try to read camera calibration and enable undistort
    if (!config_path.empty()) {
        cv::FileStorage fs2(config_path, cv::FileStorage::READ);
        if (fs2.isOpened()) {
            cv::Mat cam, dist;
            fs2["camera_matrix"] >> cam;
            fs2["dist_coeffs"] >> dist;
            if (!cam.empty()) { cfg.camera_matrix = cam; cfg.dist_coeffs = dist; cfg.undistort = true; }
            fs2.release();
        }
    }
    // If requested, undistort the single input image before detection
    if (cfg.undistort && !cfg.camera_matrix.empty()) {
        cv::Mat und;
        cv::undistort(img, und, cfg.camera_matrix, cfg.dist_coeffs);
        img = und;
    }

    LaneDetector detector(cfg);
    double offset_m = std::numeric_limits<double>::quiet_NaN();
    double offset = detector.detectAndDraw(img, &offset_m);
    if (!cv::imwrite(out, img)) { std::cerr << "Failed to write: " << out << "\n"; return 1; }
    std::cout << "Wrote " << out << "\n";
    if (!offset_file.empty()) {
        std::ofstream of(offset_file);
        of << "{";
        of << "\"offset_px\":";
        if (std::isnan(offset)) of << "null";
        else of << offset;
        // try to compute meters if not returned by detector
        double computed_m = std::numeric_limits<double>::quiet_NaN();
        if (!std::isnan(offset_m)) {
            computed_m = offset_m; // detector already provided meters
        } else if (!std::isnan(offset)) {
            if (cfg.pixels_per_meter > 0.0) {
                computed_m = offset / cfg.pixels_per_meter;
            } else if (cfg.lane_width_m > 0.0) {
                // estimate pixel lane width by scanning bottom region for filled lane color
                int h = img.rows;
                int y0 = std::max(0, static_cast<int>(h * 0.6));
                int y1 = h - 1;
                std::vector<int> widths;
                for (int y = y0; y <= y1; ++y) {
                    int left = -1, right = -1;
                    for (int x = 0; x < img.cols; ++x) {
                        cv::Vec3b p = img.at<cv::Vec3b>(y, x);
                        int g = p[1], r = p[2], b = p[0];
                        if (g > 100 && g > r + 20 && g > b + 20) {
                            if (left == -1) left = x;
                            right = x;
                        }
                    }
                    if (left != -1 && right != -1 && right > left) widths.push_back(right - left);
                }
                if (!widths.empty()) {
                    std::sort(widths.begin(), widths.end());
                    int median = widths[widths.size()/2];
                    if (median > 0) {
                        double px_per_m = static_cast<double>(median) / cfg.lane_width_m;
                        computed_m = offset / px_per_m;
                    }
                }
            }
        }
        of << ",\"offset_m\":";
        if (std::isnan(computed_m)) of << "null";
        else of << computed_m;
        // include extra metadata
        of << ",\"lane_pixel_width\":" << detector.getLastLanePixelWidth();
        of << ",\"pixels_per_meter_used\":";
        if (detector.getLastPixelsPerMeterUsed() > 0.0) of << detector.getLastPixelsPerMeterUsed();
        else if (cfg.pixels_per_meter > 0.0) of << cfg.pixels_per_meter;
        else of << "null";
        of << ",\"confidence\":" << detector.getLastConfidence();
        of << "}\n";
    }
    if (print_offset) {
        if (std::isnan(offset)) std::cout << "offset=NaN\n";
        else std::cout << "offset=" << offset << "\n";
    }
    return 0;
}
