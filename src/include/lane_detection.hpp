#pragma once

#include <array>

#include <opencv2/opencv.hpp>

class LaneDetection
{
public:
    LaneDetection();

    cv::Mat find_lines(const cv::Mat &frame, const cv::Mat& edited_frame);

private:
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

    std::vector<cv::Vec4i> hough_lines_;

    cv::Mat frame_;

private:
    void clear_averages();

    void add_average(std::array<std::pair<int, int>, average_size> &source, int x1, int x2);

    std::pair<int, int> get_average(const std::array<std::pair<int, int>, average_size> &source);

    void draw_points(cv::Vec4i pointsX, int y1, int y2);

    void draw_from_average(bool drawLeftLane, bool drawRightLane, int y1, int y2);

    void drawLanes();

    void houghLines(const cv::Mat& edited_frame, bool drawHough);

    /**
     * @brief Multiplies two vectors and then calculates the sum of the multiplied values.
     * vector A and B must be the same size and their values must be of the same type.
     *
     * @param A vector<T>.
     * @param B vector<T>.
     * @return X sum of the multiplied values.
     */
    template <typename T, typename X>
    X multiplyAndSum(std::vector<T> A, std::vector<T> B);

    /**
     * @brief Calculates the coefficients (slope and intercept) of the best fitting line
     * given independent and dependent values. Vector A and B must be the same size
     * and their values must be of the same type.
     *
     * @param A vector<T> (independent values).
     * @param B vector<T> (dependent values).
     * @return vector<X> where first element is the slope (b1), second element is intercept (b0).
     */
    template <typename T, typename X>
    std::vector<X> estimate_coefficients(std::vector<T> A, std::vector<T> B);
};