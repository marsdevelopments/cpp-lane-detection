#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "constants.hpp"

class RandomLaneTracking
{
public:
    cv::Vec4i track_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame, const cv::Vec4i &line_points_x);

private:
    cv::Mat edited_frame_;
    cv::Mat original_frame_;

    static const size_t average_size = 5;

    bool long_missing_flag = false;
    size_t missing_frames_counter = 0;

    std::array<std::pair<int, int>, average_size> left_average = {{{0, 0},
                                                                   {0, 0},
                                                                   {0, 0},
                                                                   {0, 0},
                                                                   {0, 0}}};

    std::array<std::pair<int, int>, average_size> right_average = {{{0, 0},
                                                                    {0, 0},
                                                                    {0, 0},
                                                                    {0, 0},
                                                                    {0, 0}}};

private:
    void clear_averages();

    void add_average(std::array<std::pair<int, int>, average_size> &source, const int &x1, const int &x2);

    void add_left_average(const int &x1, const int &x2);

    void add_right_average(const int &x1, const int &x2);

    std::pair<int, int> get_average(const std::array<std::pair<int, int>, average_size> &source);

    std::pair<int, int> get_left_average();

    std::pair<int, int> get_right_average();

    void draw_from_average(bool drawLeftLane, bool drawRightLane, int y1, int y2);

    void draw_points(std::pair<int, int> left, std::pair<int, int> right, int y1, int y2);
};