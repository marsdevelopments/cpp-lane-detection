#pragma once

#include <array>

#include <opencv2/opencv.hpp>

namespace cst
{
    constexpr int kVideoWidth = 1280;
    constexpr int kVideoHeight = 720;

    // constexpr int kTrapezoidOffsetX = 0;
    constexpr int kTrapezoidOffsetX = -10;
    // constexpr int kTrapezoidOffsetY = static_cast<int>(kVideoHeight * 0.4);

    // Parameters big trapezoid
    // constexpr double kTrapezoidBottomWidth = 0.52; // Width of bottom edge of trapezoid, expressed as percentage of image width
    // constexpr double kTrapezoidTopWidth = 0.155;    // Above comment also applies here, but then for the top edge of trapezoid
    // constexpr double kTrapezoidHeight = 0.215;      // Height of the trapezoid expressed as percentage of image height
    constexpr double kTrapezoidBottomWidth = 0.575; // Width of bottom edge of trapezoid, expressed as percentage of image width
    constexpr double kTrapezoidTopWidth = 0.225;    // Above comment also applies here, but then for the top edge of trapezoid
    constexpr double kTrapezoidHeight = 0.225;      // Height of the trapezoid expressed as percentage of image height

    // Parameters small trapezoid
    // constexpr double kSmallBottomWidth = 0.8; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    // constexpr double kSmallTopWidth = 0.6;     // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    // constexpr double kSmallHeight = 1.0;       // Height of the small trapezoid expressed as percentage of height of big trapezoid
    // constexpr double kSmallBottomWidth = 0.5; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    // constexpr double kSmallTopWidth = 0.25;   // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    // constexpr double kSmallHeight = 1.0;      // Height of the small trapezoid expressed as percentage of height of big trapezoid
    constexpr double kSmallBottomWidth = 0.0; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    constexpr double kSmallTopWidth = 0.0;    // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    constexpr double kSmallHeight = 0.0;      // Height of the small trapezoid expressed as percentage of height of big trapezoid

    constexpr int y_top = kVideoHeight - static_cast<int>(kVideoHeight * kTrapezoidHeight);
    constexpr int y_bottom = kVideoHeight;

    const std::array<cv::Point, 8> trapezoid_roi_points = {
        // outer trapezoid
        // bottom left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidBottomWidth)) / 2.0) + kTrapezoidOffsetX,
            y_bottom),
        // top left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidTopWidth)) / 2.0) + kTrapezoidOffsetX,
            y_top),
        // top right
        cv::Point(
            static_cast<int>(kVideoWidth - (kVideoWidth * (1.0 - kTrapezoidTopWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight - static_cast<int>(kVideoHeight * kTrapezoidHeight)),
        // bottom right
        cv::Point(
            static_cast<int>(kVideoWidth - (kVideoWidth * (1.0 - kTrapezoidBottomWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight),

        // inner trapezoid
        // bottom left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidBottomWidth * kSmallBottomWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight),
        // top left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidTopWidth * kSmallTopWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight - static_cast<int>(kVideoHeight * kTrapezoidHeight * kSmallHeight)),
        // top right
        cv::Point(
            static_cast<int>(kVideoWidth - (kVideoWidth * (1.0 - kTrapezoidTopWidth * kSmallTopWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight - static_cast<int>(kVideoHeight * kTrapezoidHeight * kSmallHeight)),
        // bottom right
        cv::Point(
            static_cast<int>(kVideoWidth - (kVideoWidth * (1.0 - kTrapezoidBottomWidth * kSmallBottomWidth)) / 2.0) + kTrapezoidOffsetX,
            kVideoHeight)};

    constexpr int kMinLaneWidth = 50;
    constexpr double kSlopeThreshold = 0.5;

    constexpr int kCannyLowThreshold = 60;
    constexpr int kCannyRatio = 3;
    constexpr int kCannyHighThreshold = kCannyLowThreshold * kCannyRatio;

    constexpr int kLaneLineWidth = 25;

    constexpr int x_center = cst::kVideoWidth / 2;
    constexpr size_t lines_array_size = 300;
    constexpr size_t retrack_array_size = 75;
    constexpr int retrack_x_delta = 25;
}