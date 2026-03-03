#include "lane_detector.h"
#include <limits>
#include <iostream>
#include <algorithm>

static double slope(const cv::Vec4i &l) {
    double dx = l[2] - l[0];
    double dy = l[3] - l[1];
    if (fabs(dx) < 1e-6) return 0.0;
    return dy / dx;
}

cv::Mat LaneDetector::regionOfInterest(const cv::Mat &img) {
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    // polygon roughly covering bottom half of the image
    std::vector<cv::Point> pts;
    int w = img.cols;
    int h = img.rows;
    pts.emplace_back(w * 0.1, h);
    pts.emplace_back(w * 0.45, h * 0.6);
    pts.emplace_back(w * 0.55, h * 0.6);
    pts.emplace_back(w * 0.9, h);
    std::vector<std::vector<cv::Point>> fillPts = {pts};
    cv::fillPoly(mask, fillPts, cv::Scalar(255, 255, 255));
    cv::Mat out;
    cv::bitwise_and(img, mask, out);
    return out;
}

std::vector<cv::Vec4i> LaneDetector::filterAndAverageLines(const std::vector<cv::Vec4i> &lines, int imgWidth, int imgHeight) {
    // Separate into left and right by slope sign and x position
    std::vector<std::pair<double, double>> left; // slope, intercept
    std::vector<std::pair<double, double>> right;

    for (const auto &l : lines) {
        double m = slope(l);
        if (fabs(m) < 0.3) continue; // filter nearly horizontal
        double b = l[1] - m * l[0];
        double x_mid = (l[0] + l[2]) / 2.0;
        if (m < 0 && x_mid < imgWidth / 2) left.emplace_back(m, b);
        else if (m > 0 && x_mid > imgWidth / 2) right.emplace_back(m, b);
    }

    std::vector<cv::Vec4i> out;
    auto make_line = [&](const std::vector<std::pair<double, double>> &arr) -> void {
        if (arr.empty()) return;
        double m_avg = 0, b_avg = 0;
        for (auto &p : arr) { m_avg += p.first; b_avg += p.second; }
        m_avg /= arr.size(); b_avg /= arr.size();
        int y1 = static_cast<int>(imgHeight * 0.6); // approximate top y of lane
        int y2 = imgHeight;       // bottom y
        // convert y to x: x = (y - b) / m
        int x1 = static_cast<int>((y1 - b_avg) / m_avg);
        int x2 = static_cast<int>((y2 - b_avg) / m_avg);
        out.emplace_back(cv::Vec4i(x1, y1, x2, y2));
    };

    make_line(left);
    make_line(right);
    return out;
}

void LaneDetector::drawLines(cv::Mat &img, const std::vector<cv::Vec4i> &lines) {
    for (const auto &l : lines) {
        cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 4);
    }
}

// Simple perspective transform matrices cached per image size.
// Build perspective transform matrix using the detector's configuration ratios
cv::Mat LaneDetector::getPerspectiveMat(int w, int h, bool forward) {
    std::vector<cv::Point2f> src{
        {w * cfg_.src_top_left_x, h * cfg_.src_top_y},
        {w * cfg_.src_top_right_x, h * cfg_.src_top_y},
        {w * cfg_.src_bottom_right_x, (float)h},
        {w * cfg_.src_bottom_left_x, (float)h}
    };
    std::vector<cv::Point2f> dst{
        {w * cfg_.dst_left_x, 0.0f},
        {w * cfg_.dst_right_x, 0.0f},
        {w * cfg_.dst_right_x, (float)h},
        {w * cfg_.dst_left_x, (float)h}
    };
    if (forward) return cv::getPerspectiveTransform(src, dst);
    return cv::getPerspectiveTransform(dst, src);
}

cv::Mat LaneDetector::perspectiveTransform(const cv::Mat &img, bool forward) {
    if (cached_w_ != img.cols || cached_h_ != img.rows) ensureCache(img.cols, img.rows);
    cv::Mat out;
    if (forward) cv::warpPerspective(img, out, cached_M_forward_, img.size(), cv::INTER_LINEAR);
    else cv::warpPerspective(img, out, cached_M_inverse_, img.size(), cv::INTER_LINEAR);
    return out;
}

void LaneDetector::ensureCache(int w, int h) {
    cached_w_ = w; cached_h_ = h;
    cached_M_forward_ = getPerspectiveMat(w, h, true);
    cached_M_inverse_ = getPerspectiveMat(w, h, false);
    // build ROI mask once per size (mask in bird-eye space)
    cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
    std::vector<cv::Point> pts;
    pts.emplace_back(static_cast<int>(w * 0.1), h);
    pts.emplace_back(static_cast<int>(w * 0.45), static_cast<int>(h * 0.6));
    pts.emplace_back(static_cast<int>(w * 0.55), static_cast<int>(h * 0.6));
    pts.emplace_back(static_cast<int>(w * 0.9), h);
    std::vector<std::vector<cv::Point>> fillPts = {pts};
    cv::fillPoly(mask, fillPts, cv::Scalar(255));
    cached_roi_mask_ = mask;
}

// Constructors
LaneDetector::LaneDetector(): cfg_(), smoothing_alpha_(cfg_.smoothing_alpha) {}
LaneDetector::LaneDetector(const DetectorConfig &cfg): cfg_(cfg), smoothing_alpha_(cfg_.smoothing_alpha) {}

// initialize Kalman filter matrices (state: [x, v], measurement: [x])
static void initKalman(cv::KalmanFilter &kf) {
    kf.transitionMatrix = (cv::Mat_<float>(2,2) << 1.0f, 1.0f, 0.0f, 1.0f);
    kf.measurementMatrix = (cv::Mat_<float>(1,2) << 1.0f, 0.0f);
    kf.processNoiseCov = cv::Mat::eye(2,2,CV_32F) * 1e-3f;
    kf.measurementNoiseCov = cv::Mat::eye(1,1,CV_32F) * 1e-1f;
    kf.errorCovPost = cv::Mat::eye(2,2,CV_32F);
}

bool LaneDetector::polyfitQuadratic(const std::vector<cv::Point> &pts, cv::Vec3d &coeffs) {
    if (pts.size() < 3) return false;
    // Solve for coeffs [a b c] in x = a*y^2 + b*y + c
    int n = (int)pts.size();
    cv::Mat A(n, 3, CV_64F);
    cv::Mat B(n, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
        double y = pts[i].y;
        A.at<double>(i, 0) = y*y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        B.at<double>(i, 0) = pts[i].x;
    }
    cv::Mat X;
    bool ok = cv::solve(A, B, X, cv::DECOMP_SVD);
    if (!ok) return false;
    coeffs[0] = X.at<double>(0,0);
    coeffs[1] = X.at<double>(1,0);
    coeffs[2] = X.at<double>(2,0);
    return true;
}

// Helper to sample points along Hough lines and assign to left/right
static void collectLinePoints(const std::vector<cv::Vec4i> &lines, std::vector<cv::Point> &leftPts, std::vector<cv::Point> &rightPts, int imgCenterX) {
    for (const auto &l : lines) {
        double m = slope(l);
        if (fabs(m) < 0.3) continue;
        // sample points along the segment (reduced steps for perf)
        int steps = 6;
        for (int i=0;i<=steps;i++){
            double t = i/(double)steps;
            int x = static_cast<int>(l[0] + t*(l[2]-l[0]));
            int y = static_cast<int>(l[1] + t*(l[3]-l[1]));
            if (x < imgCenterX) leftPts.emplace_back(x,y);
            else rightPts.emplace_back(x,y);
        }
    }
}

// Estimate pixels-per-meter by finding strong vertical gradients near expected
// left/right lane edge positions at the bottom region of the image. Returns
// px_per_m (>0) on success or 0 on failure.
static double estimatePxPerMFromGradient(const cv::Mat &frame, int left_x, int right_x, double lane_width_m) {
    if (lane_width_m <= 0.0) return 0.0;
    cv::Mat gray;
    if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else gray = frame;
    int h = gray.rows, w = gray.cols;
    int y0 = std::max(0, static_cast<int>(h * 0.6));
    int y1 = h - 1;
    int x0 = std::max(0, left_x - 120);
    int x1 = std::min(w - 1, right_x + 120);
    if (x1 <= x0) return 0.0;

    cv::Mat gx;
    cv::Sobel(gray, gx, CV_32F, 1, 0, 3);

    std::vector<float> profile(x1 - x0 + 1, 0.0f);
    for (int y = y0; y <= y1; ++y) {
        const float *row = gx.ptr<float>(y);
        for (int x = x0; x <= x1; ++x) profile[x - x0] += std::abs(row[x]);
    }
    // smooth profile with simple box filter
    int k = 7;
    std::vector<float> smooth(profile.size(), 0.0f);
    for (size_t i = 0; i < profile.size(); ++i) {
        int a = std::max<int>(0, i - k/2);
        int b = std::min<int>(profile.size()-1, i + k/2);
        float s = 0; int cnt = 0;
        for (int j=a;j<=b;++j){ s+=profile[j]; cnt++; }
        smooth[i] = s / std::max(1, cnt);
    }
    int mid = (left_x + right_x) / 2 - x0;
    if (mid <= 0 || mid >= static_cast<int>(smooth.size())) return 0.0;
    // find peak in left and right halves
    auto left_it = std::max_element(smooth.begin(), smooth.begin() + std::max(1, mid));
    auto right_it = std::max_element(smooth.begin() + std::min((int)smooth.size()-1, mid+1), smooth.end());
    int left_peak = static_cast<int>(std::distance(smooth.begin(), left_it)) + x0;
    int right_peak = static_cast<int>(std::distance(smooth.begin(), right_it)) + x0;
    if (right_peak <= left_peak) return 0.0;
    int pixel_width = right_peak - left_peak;
    if (pixel_width <= 0) return 0.0;
    double px_per_m = static_cast<double>(pixel_width) / lane_width_m;
    return px_per_m;
}

double LaneDetector::detectAndDraw(cv::Mat &frame, double *out_m) {
    // 1. Preprocess
    cv::Mat gray, blur, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, edges, static_cast<int>(cfg_.canny_low), static_cast<int>(cfg_.canny_high));

    // 2. Perspective transform to bird's-eye (improves lane fitting)
    cv::Mat bev = perspectiveTransform(edges, true);

    // 3. Mask ROI in bird's-eye
    cv::Mat masked = regionOfInterest(bev);

    // 4. Line detection (Hough)
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(masked, lines, 1, CV_PI / 180, cfg_.hough_threshold, cfg_.hough_min_line_len, cfg_.hough_max_line_gap);

    // 5. Collect points and fit quadratic curves
    std::vector<cv::Point> leftPts, rightPts;
    collectLinePoints(lines, leftPts, rightPts, frame.cols/2);

    cv::Vec3d leftCoef, rightCoef;
    bool leftOk = polyfitQuadratic(leftPts, leftCoef);
    bool rightOk = polyfitQuadratic(rightPts, rightCoef);

    // 6. Temporal smoothing (single-image demo uses previous values if available)
    if (leftOk) {
        if (has_prev_left_) prev_left_ = smoothing_alpha_ * leftCoef + (1.0 - smoothing_alpha_) * prev_left_;
        else { prev_left_ = leftCoef; has_prev_left_ = true; }
    }
    if (rightOk) {
        if (has_prev_right_) prev_right_ = smoothing_alpha_ * rightCoef + (1.0 - smoothing_alpha_) * prev_right_;
        else { prev_right_ = rightCoef; has_prev_right_ = true; }
    }

    // 7. Draw fitted lanes back on original image (inverse perspective)
    cv::Mat overlay = frame.clone();
    int h = frame.rows;
    // Prepare inverse perspective matrix to map bird's-eye points back to original image
    cv::Mat M_inv = getPerspectiveMat(frame.cols, frame.rows, false);
    auto drawPoly = [&](const cv::Vec3d &coef) {
        std::vector<cv::Point2f> bev_pts;
        for (int y = static_cast<int>(h*0.5); y < h; y += 5) {
            double x = coef[0]*y*y + coef[1]*y + coef[2];
            bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
        if (bev_pts.size() < 2) return;
        std::vector<cv::Point2f> orig_pts_f;
        cv::perspectiveTransform(bev_pts, orig_pts_f, M_inv);
        std::vector<cv::Point> orig_pts;
        orig_pts.reserve(orig_pts_f.size());
        for (const auto &p : orig_pts_f) orig_pts.emplace_back(static_cast<int>(p.x + 0.5f), static_cast<int>(p.y + 0.5f));
        if (orig_pts.size() >= 2) cv::polylines(overlay, orig_pts, false, cv::Scalar(0,255,0), 4);
    };

    std::vector<cv::Point> left_orig_pts, right_orig_pts;
    if (leftOk) {
        // sample bev points and map back
        std::vector<cv::Point2f> bev_pts;
        for (int y = static_cast<int>(h*0.5); y < h; y += 5) {
            double x = prev_left_[0]*y*y + prev_left_[1]*y + prev_left_[2];
            bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
        std::vector<cv::Point2f> orig_pts_f;
        cv::perspectiveTransform(bev_pts, orig_pts_f, M_inv);
        for (const auto &p : orig_pts_f) left_orig_pts.emplace_back(static_cast<int>(p.x+0.5f), static_cast<int>(p.y+0.5f));
    }
    if (rightOk) {
        std::vector<cv::Point2f> bev_pts;
        for (int y = static_cast<int>(h*0.5); y < h; y += 5) {
            double x = prev_right_[0]*y*y + prev_right_[1]*y + prev_right_[2];
            bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
        std::vector<cv::Point2f> orig_pts_f;
        cv::perspectiveTransform(bev_pts, orig_pts_f, M_inv);
        for (const auto &p : orig_pts_f) right_orig_pts.emplace_back(static_cast<int>(p.x+0.5f), static_cast<int>(p.y+0.5f));
    }

    // If we have both sides, draw filled polygon between them
    double lateral_offset = std::numeric_limits<double>::quiet_NaN();
    double lateral_offset_m = std::numeric_limits<double>::quiet_NaN();
    if (!left_orig_pts.empty() && !right_orig_pts.empty()) {
        // build polygon: left points (top->bottom) + right points (bottom->top)
        std::vector<cv::Point> poly = left_orig_pts;
        for (auto it = right_orig_pts.rbegin(); it != right_orig_pts.rend(); ++it) poly.emplace_back(*it);
        std::vector<std::vector<cv::Point>> fillPolyPts = {poly};
        cv::fillPoly(overlay, fillPolyPts, cv::Scalar(0, 200, 0));

        // compute lane center at bottom by averaging bottom-most point x
        int left_x = left_orig_pts.back().x;
        int right_x = right_orig_pts.back().x;
        int lane_center = (left_x + right_x) / 2;
        int image_center = frame.cols / 2;
        lateral_offset = static_cast<double>(lane_center - image_center);
        // compute meters if possible
        double pixel_to_meter = 0.0;
        // Prefer to compute pixel lane width by mapping bottom points derived from bev polynomial
        int pixel_lane_width = std::abs(right_x - left_x);
        // compute bev bottom x positions from quadratic coefficients
        double bev_y = static_cast<double>(h);
        double bev_left_x = std::numeric_limits<double>::quiet_NaN();
        double bev_right_x = std::numeric_limits<double>::quiet_NaN();
        auto coef_valid = [](const cv::Vec3d &c)->bool {
            for (int i=0;i<3;++i) {
                if (!std::isfinite(c[i])) return false;
            }
            // avoid degenerate all-zero
            if (std::abs(c[0]) < 1e-12 && std::abs(c[1]) < 1e-8 && std::abs(c[2]) < 1e-3) return false;
            return true;
        };
        if (has_prev_left_ && coef_valid(prev_left_)) bev_left_x = prev_left_[0]*bev_y*bev_y + prev_left_[1]*bev_y + prev_left_[2];
        if (has_prev_right_ && coef_valid(prev_right_)) bev_right_x = prev_right_[0]*bev_y*bev_y + prev_right_[1]*bev_y + prev_right_[2];
        // Fallback: if bev coefficients invalid, try linear averaged Hough lines in bev space
        if (!std::isfinite(bev_left_x) || !std::isfinite(bev_right_x)) {
            auto avg_lines = filterAndAverageLines(lines, frame.cols, frame.rows);
            if (!avg_lines.empty()) {
                // avg_lines: first=left, second=right (if available)
                if (avg_lines.size() >= 1) bev_left_x = static_cast<double>(avg_lines[0][2]);
                if (avg_lines.size() >= 2) bev_right_x = static_cast<double>(avg_lines[1][2]);
            }
        }
        std::vector<cv::Point2f> bev_bottom_pts = { cv::Point2f(static_cast<float>(bev_left_x),(float)bev_y), cv::Point2f(static_cast<float>(bev_right_x),(float)bev_y) };
        std::vector<cv::Point2f> mapped_bottom_pts;
        try {
            cv::perspectiveTransform(bev_bottom_pts, mapped_bottom_pts, M_inv);
            if (mapped_bottom_pts.size() == 2) {
                int mapped_left_x = static_cast<int>(mapped_bottom_pts[0].x + 0.5f);
                int mapped_right_x = static_cast<int>(mapped_bottom_pts[1].x + 0.5f);
                int mapped_width = std::abs(mapped_right_x - mapped_left_x);
                if (mapped_width > 0) pixel_lane_width = mapped_width;
            }
        } catch(...) {
            // fall back to pixel_lane_width from orig points
        }

        if (cfg_.verbose) {
            std::cerr << "DEBUG: leftPts=" << leftPts.size() << " rightPts=" << rightPts.size()
                      << " left_x=" << left_x << " right_x=" << right_x << " pixel_lane_width=" << pixel_lane_width
                      << " bev_left_x=" << bev_left_x << " bev_right_x=" << bev_right_x << " has_prev_left=" << has_prev_left_ << " has_prev_right=" << has_prev_right_ << "\n";
        }
        if (cfg_.pixels_per_meter > 0.0) {
            pixel_to_meter = cfg_.pixels_per_meter;
        } else if (cfg_.lane_width_m > 0.0 && pixel_lane_width > 0) {
            pixel_to_meter = static_cast<double>(pixel_lane_width) / cfg_.lane_width_m; // px per meter
        } else if (cfg_.lane_width_m > 0.0) {
            // try gradient-based estimate around bottom of image
            double grad_pxpm = estimatePxPerMFromGradient(frame, left_x, right_x, cfg_.lane_width_m);
            if (grad_pxpm > 0.0) pixel_to_meter = grad_pxpm;
        }
        if (cfg_.verbose) {
            std::cerr << "DEBUG: pixel_to_meter=" << pixel_to_meter << " cfg.pixels_per_meter=" << cfg_.pixels_per_meter
                      << " cfg.lane_width_m=" << cfg_.lane_width_m << "\n";
        }
        if (pixel_to_meter > 0.0) {
            lateral_offset_m = lateral_offset / pixel_to_meter;
            char buf2[128];
            snprintf(buf2, sizeof(buf2), " (%.2fm)", lateral_offset_m);
            cv::putText(overlay, buf2, cv::Point(20,60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
        }
        // record metadata
        this->last_lane_pixel_width = pixel_lane_width;
        if (pixel_to_meter > 0.0) this->last_pixels_per_meter_used = pixel_to_meter;
        double pts_total = static_cast<double>(leftPts.size() + rightPts.size());
        this->last_confidence = std::min(1.0, pts_total / 200.0);
        // draw center line
        cv::line(overlay, cv::Point(lane_center, h), cv::Point(lane_center, h/2), cv::Scalar(255,0,0), 2);
        // annotate offset
        char buf[128];
        snprintf(buf, sizeof(buf), "offset=%.1f px", lateral_offset);
        cv::putText(overlay, buf, cv::Point(20,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2);
    }

    // 8. Blend overlay into frame
    double alpha = 0.7;
    cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);

    // Apply Kalman smoothing to lateral offset (pixels) if available
    if (std::isfinite(lateral_offset)) {
        if (!kalman_initialized_) {
            initKalman(kf_);
            kf_.statePost.at<float>(0) = static_cast<float>(lateral_offset);
            kf_.statePost.at<float>(1) = 0.0f;
            kalman_initialized_ = true;
            last_smoothed_offset_ = lateral_offset;
        } else {
            cv::Mat prediction = kf_.predict();
            cv::Mat measurement(1,1,CV_32F);
            measurement.at<float>(0) = static_cast<float>(lateral_offset);
            cv::Mat estimated = kf_.correct(measurement);
            last_smoothed_offset_ = static_cast<double>(estimated.at<float>(0));
        }
    }

    if (out_m) *out_m = lateral_offset_m;
    // return smoothed offset in pixels when available
    if (std::isfinite(last_smoothed_offset_)) return last_smoothed_offset_;
    return lateral_offset;
}

int LaneDetector::getLastLanePixelWidth() const { return last_lane_pixel_width; }
double LaneDetector::getLastPixelsPerMeterUsed() const { return last_pixels_per_meter_used; }
double LaneDetector::getLastConfidence() const { return last_confidence; }

