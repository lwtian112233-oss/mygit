#include "lane_detector.h"
#include <limits>
#include <iostream>

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

std::vector<cv::Vec4i> LaneDetector::filterAndAverageLines(const std::vector<cv::Vec4i> &lines, int imgWidth) {
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
        int y1 = imgWidth * 0.6; // approximate top y of lane
        int y2 = imgWidth;       // bottom y
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
    cv::Mat M = getPerspectiveMat(img.cols, img.rows, forward);
    cv::Mat out;
    cv::warpPerspective(img, out, M, img.size(), cv::INTER_LINEAR);
    return out;
}

// Constructors
LaneDetector::LaneDetector(): cfg_(), smoothing_alpha_(cfg_.smoothing_alpha) {}
LaneDetector::LaneDetector(const DetectorConfig &cfg): cfg_(cfg), smoothing_alpha_(cfg_.smoothing_alpha) {}

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
        // sample points along the segment
        int steps = 10;
        for (int i=0;i<=steps;i++){
            double t = i/(double)steps;
            int x = static_cast<int>(l[0] + t*(l[2]-l[0]));
            int y = static_cast<int>(l[1] + t*(l[3]-l[1]));
            if (x < imgCenterX) leftPts.emplace_back(x,y);
            else rightPts.emplace_back(x,y);
        }
    }
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
        prev_left_ = smoothing_alpha_ * leftCoef + (1.0 - smoothing_alpha_) * prev_left_;
    }
    if (rightOk) {
        prev_right_ = smoothing_alpha_ * rightCoef + (1.0 - smoothing_alpha_) * prev_right_;
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
        double bev_left_x = prev_left_[0]*bev_y*bev_y + prev_left_[1]*bev_y + prev_left_[2];
        double bev_right_x = prev_right_[0]*bev_y*bev_y + prev_right_[1]*bev_y + prev_right_[2];
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

        if (cfg_.pixels_per_meter > 0.0) {
            pixel_to_meter = cfg_.pixels_per_meter;
        } else if (cfg_.lane_width_m > 0.0 && pixel_lane_width > 0) {
            pixel_to_meter = static_cast<double>(pixel_lane_width) / cfg_.lane_width_m; // px per meter
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

    if (out_m) *out_m = lateral_offset_m;
    return lateral_offset;
}

int LaneDetector::getLastLanePixelWidth() const { return last_lane_pixel_width; }
double LaneDetector::getLastPixelsPerMeterUsed() const { return last_pixels_per_meter_used; }
double LaneDetector::getLastConfidence() const { return last_confidence; }

