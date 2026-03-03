#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetector {
public:
    struct DetectorConfig {
        // source trapezoid ratios (relative to image width/height)
        float src_top_left_x = 0.45f;
        float src_top_right_x = 0.55f;
        float src_top_y = 0.62f;
        float src_bottom_left_x = 0.05f;
        float src_bottom_right_x = 0.95f;
        // destination rectangle x ratios
        float dst_left_x = 0.2f;
        float dst_right_x = 0.8f;

        // image processing params
        double smoothing_alpha = 0.2;
        int canny_low = 50;
        int canny_high = 150;
        int hough_threshold = 50;
        int hough_min_line_len = 30;
        int hough_max_line_gap = 10;
        // real-world scaling
        double lane_width_m = 0.0; // expected lane width in meters (optional)
        double pixels_per_meter = 0.0; // optional override, px/m
            // optional camera calibration (if provided, frames can be undistorted)
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            bool undistort = false;
            // performance / debug
            bool verbose = false;
            int process_scale = 1; // 1 = full size, 2 = half size, etc.
    };

    LaneDetector();
    explicit LaneDetector(const DetectorConfig &cfg);
    // Detect lane lines and draw result on the input frame (in-place).
    // `detectAndDraw` processes a single frame: it applies pre-processing,
    // perspective transform (bird's-eye), detects lane points, fits quadratic
    // curves for left/right lanes, applies simple temporal smoothing, draws
    // a filled lane polygon on `frame` and returns lateral offset in pixels
    // (positive means vehicle is to the right of lane center). If `out_m` is
    // non-null and meter conversion is available, sets *out_m to the offset in meters.
    // If lanes cannot be determined returns NaN.
    double detectAndDraw(cv::Mat &frame, double *out_m = nullptr);

    // Public getters for metadata from last detection
    int getLastLanePixelWidth() const;
    double getLastPixelsPerMeterUsed() const;
    double getLastConfidence() const;

private:
    cv::Mat regionOfInterest(const cv::Mat &img);
    void drawLines(cv::Mat &img, const std::vector<cv::Vec4i> &lines);
    std::vector<cv::Vec4i> filterAndAverageLines(const std::vector<cv::Vec4i> &lines, int imgWidth, int imgHeight);

    // New helpers for improved pipeline
    cv::Mat perspectiveTransform(const cv::Mat &img, bool forward);
    // Fit quadratic x = a*y^2 + b*y + c to a set of points (y as independent variable)
    bool polyfitQuadratic(const std::vector<cv::Point> &pts, cv::Vec3d &coeffs);

    // Temporal smoothing: keep previous coefficients for left/right lanes
    cv::Vec3d prev_left_{0,0,0};
    cv::Vec3d prev_right_{0,0,0};
    bool has_prev_left_ = false;
    bool has_prev_right_ = false;
    double smoothing_alpha_ = 0.2; // exponential smoothing factor
    DetectorConfig cfg_;
    cv::Mat getPerspectiveMat(int w, int h, bool forward);
    // Cache for perspective matrices and ROI mask to avoid recomputing each frame
    void ensureCache(int w, int h);
    cv::Mat cached_M_forward_, cached_M_inverse_, cached_roi_mask_;
    int cached_w_ = 0, cached_h_ = 0;
        // Simple Kalman filter to smooth lateral offset across frames
        cv::KalmanFilter kf_{2,1,0};
        bool kalman_initialized_ = false;
        double last_smoothed_offset_ = std::numeric_limits<double>::quiet_NaN();
    // Metadata from last detection (updated by detectAndDraw)
    int last_lane_pixel_width = 0;            // measured lane pixel width at bottom
    double last_pixels_per_meter_used = 0.0;  // px per meter used for conversion (if any)
    double last_confidence = 0.0;             // simple confidence score [0..1]
};
