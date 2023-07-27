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

    // Parameters outer trapezoid
    constexpr double kTrapezoidBottomWidth = 0.575; // Width of bottom edge of trapezoid, expressed as percentage of image width
    constexpr double kTrapezoidTopWidth = 0.225;    // Above comment also applies here, but then for the top edge of trapezoid
    constexpr double kTrapezoidHeight = 0.225;      // Height of the trapezoid expressed as percentage of image height

    // Parameters inner trapezoid
    constexpr double kSmallBottomWidth = 0.0; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    constexpr double kSmallTopWidth = 0.0;    // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    constexpr double kSmallHeight = 0.0;      // Height of the small trapezoid expressed as percentage of height of big trapezoid

    // Y coordinate of highest point of the trapezoid
    constexpr int kTopY = kVideoHeight - static_cast<int>(kVideoHeight * kTrapezoidHeight);
    // Y coordinate of lowest point of the trapezoid
    constexpr int kBottomY = kVideoHeight;

    // array of points making ROI trapezoid
    const std::array<cv::Point, 8> trapezoid_roi_points = {
        // outer trapezoid
        // bottom left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidBottomWidth)) / 2.0) + kTrapezoidOffsetX,
            kBottomY),
        // top left
        cv::Point(
            static_cast<int>((kVideoWidth * (1.0 - kTrapezoidTopWidth)) / 2.0) + kTrapezoidOffsetX,
            kTopY),
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

    constexpr int kCannyLowThreshold = 60;
    constexpr int kCannyRatio = 3;
    constexpr int kCannyHighThreshold = kCannyLowThreshold * kCannyRatio;

    // X coordinate marking the center of the image
    constexpr int kCenterX = cst::kVideoWidth / 2;
    // size of the array containing random built lines, which is used to find lanes in ROI 
    constexpr size_t kLinesArraySize = 400;
    // size of the similar array but is used to re-track already found lanes
    constexpr size_t kRetrackArraySize = 75;
    // ammount of pixels which will be added to found x coordinate of lines
    // which will define the area where to re-track lanes
    constexpr int kRetrackDeltaX = 25;
}