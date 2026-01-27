#ifndef LANE_DETECT_HPP
#define LANE_DETECT_HPP

#include <functional>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>

inline cv::Mat applyBirdsEyeView(const cv::Mat &image) {
    int height = image.rows;
    int width = image.cols;

    cv::Point2f src[4] = {
        cv::Point2f(width * 0.3f, height * 0.4f),
        cv::Point2f(width * 0.7f, height * 0.4f),
        cv::Point2f(width, height),
        cv::Point2f(0, height)
    };

    cv::Point2f dst[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(width, 0),
        cv::Point2f(width, height),
        cv::Point2f(0, height)
    };

    cv::Mat M = cv::getPerspectiveTransform(src, dst);
    cv::Mat result;
    cv::warpPerspective(image, result, M, cv::Size(width, height));
    return result;
}

inline std::pair<cv::Mat, std::pair<int, int>> detectLane(
    const cv::Mat &image,
    int truck_id,
    const std::vector<int> &last_left_fit,
    const std::vector<int> &last_right_fit
) {
    int height = image.rows;
    int width = image.cols;

    cv::Mat hls;
    cv::cvtColor(image, hls, cv::COLOR_BGR2HLS);

    cv::Mat yellow_mask, white_mask;
    cv::inRange(hls, cv::Scalar(10, 50, 100), cv::Scalar(40, 255, 255), yellow_mask);
    cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 70), white_mask);

    cv::Mat combined_mask;
    cv::bitwise_or(yellow_mask, white_mask, combined_mask);

    cv::Mat masked_hls;
    cv::bitwise_and(image, image, masked_hls, combined_mask);

    cv::Mat gray, blur, edges;
    cv::cvtColor(masked_hls, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, edges, 50, 150);

    cv::Mat histogram;
    cv::reduce(edges.rowRange(height * 2 / 3, height), histogram, 0, cv::REDUCE_SUM, CV_32S);

    std::vector<int> hist_values;
    histogram.colRange(0, histogram.cols).copyTo(hist_values);

    int midpoint = width / 2;
    int leftx = std::distance(hist_values.begin(), std::max_element(hist_values.begin(), hist_values.begin() + midpoint));
    int rightx = std::distance(hist_values.begin() + midpoint, std::max_element(hist_values.begin() + midpoint, hist_values.end())) + midpoint;

    if (leftx == 0 && std::accumulate(hist_values.begin(), hist_values.begin() + midpoint, 0) < 5000)
        leftx = last_left_fit.empty() ? width / 5 : last_left_fit.back();

    if (rightx == midpoint && std::accumulate(hist_values.begin() + midpoint, hist_values.end(), 0) < 5000)
        rightx = last_right_fit.empty() ? width * 4 / 5 : last_right_fit.back();

    int nwindows = 15;
    int margin = 30;
    int minpix = 5;
    int window_height = height / nwindows;

    std::vector<cv::Point> nonzero;
    cv::findNonZero(edges, nonzero);

    std::vector<int> left_lane_inds, right_lane_inds;

    for (int window = nwindows - 1; window >= 0; window--) {
        int win_y_low = window * window_height;
        int win_y_high = (window + 1) * window_height;

        int left_win_xlow = leftx - margin;
        int left_win_xhigh = leftx + margin;
        int right_win_xlow = rightx - margin;
        int right_win_xhigh = rightx + margin;

        for (size_t i = 0; i < nonzero.size(); ++i) {
            int x = nonzero[i].x;
            int y = nonzero[i].y;
            if (y >= win_y_low && y < win_y_high) {
                if (x >= left_win_xlow && x < left_win_xhigh) left_lane_inds.push_back(i);
                if (x >= right_win_xlow && x < right_win_xhigh) right_lane_inds.push_back(i);
            }
        }

        if (left_lane_inds.size() > static_cast<size_t>(minpix)) {
            float mean_x = 0.0f;
            for (int idx : left_lane_inds) mean_x += nonzero[idx].x;
            mean_x /= left_lane_inds.size();
            leftx = static_cast<int>(0.8f * mean_x + 0.2f * leftx);
        }

        if (right_lane_inds.size() > static_cast<size_t>(minpix)) {
            float mean_x = 0.0f;
            for (int idx : right_lane_inds) mean_x += nonzero[idx].x;
            mean_x /= right_lane_inds.size();
            rightx = static_cast<int>(0.8f * mean_x + 0.2f * rightx);
        }
    }

    cv::Mat window_img = cv::Mat::zeros(image.size(), image.type());
    cv::rectangle(window_img, cv::Point(leftx - margin, 0), cv::Point(leftx + margin, height), cv::Scalar(255, 0, 0), 2);
    cv::rectangle(window_img, cv::Point(rightx - margin, 0), cv::Point(rightx + margin, height), cv::Scalar(0, 0, 255), 2);

    cv::Mat lane_overlay;
    cv::addWeighted(image, 1.0, window_img, 0.3, 0, lane_overlay);

    return std::make_pair(lane_overlay, std::make_pair(leftx, rightx));
}

inline float calculateSteering(float lane_left_x, float lane_right_x, int img_width, std::function<float(float)> pid_compute){
    float img_center = img_width / 2.0f;
    float lane_center = (lane_left_x + lane_right_x) / 2.0f;
    float error = (lane_center - img_center) / img_center;
    float pid_output = pid_compute(error);
    return std::clamp(-pid_output * 35.0f, -30.0f, 30.0f);
}

#endif // LANE_DETECT_HPP
