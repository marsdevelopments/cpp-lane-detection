#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "LanePoints.hpp"
#include "constants.hpp"

class LaneTracking
{
public:
  /**
   * @brief Start lane tracking from scrath, and search the whole ROI
   *
   * @param edited_frame edited frame by PreProcessing in which search will be
   * done
   * @param debug_frame frame on which debug lines will be drawn
   *
   * @return found lane points
   */
  LanePoints restart_tracking(const cv::Mat& edited_frame,
                              const cv::Mat& debug_frame);

  /**
   * @brief Search in a limited area, defined by previously found lane points
   * and @ref cst::retrack_x_delta
   *
   * @param edited_frame edited frame by PreProcessing in which search will be
   * done
   * @param debug_frame frame on which debug lines will be drawn
   *
   * @return found lane points
   */
  LanePoints retrack(const cv::Mat& edited_frame, const cv::Mat& debug_frame);

private:
  static const size_t average_size = 1;

  std::pair<int, int> left_line;
  std::pair<int, int> right_line;

  const int bottom_left_x_ = cst::trapezoid_roi_points.at(0).x;
  const int top_left_x_ = cst::trapezoid_roi_points.at(1).x;
  const int top_right_x_ = cst::trapezoid_roi_points.at(2).x;
  const int bottom_right_x_ = cst::trapezoid_roi_points.at(3).x;

  cv::Mat edited_frame_;

  bool long_missing_flag = false;
  size_t missing_frames_counter = 0;

  std::array<std::pair<int, int>, average_size> left_average = {
    { { 0, 0 } /* , { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } */ }
  };

  std::array<std::pair<int, int>, average_size> right_average = {
    { { 0, 0 } /* , { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } */ }
  };

  LanePoints last_lane_points;

private:
  /**
   * @brief Replace all numbers averages arrays with zero
   */
  void clear_averages();

  /**
   * @brief Add a number to an average array
   *
   * @param source array into which to add values
   * @param x1 top-side x value
   * @param x2 bottom-side x value
   */
  void add_average(std::array<std::pair<int, int>, average_size>& source,
                   const int& x1,
                   const int& x2);
  /**
   * @brief Add a number to an left-side average array
   *
   * @param x1 top-side x value
   * @param x2 bottom-side x value
   */
  void add_left_average(const int& x1, const int& x2);
  /**
   * @brief Add a number to an right-side average array
   *
   * @param x1 top-side x value
   * @param x2 bottom-side x value
   */
  void add_right_average(const int& x1, const int& x2);

  /**
   * @brief Get average values from an average array
   *
   * @param source array from which to compute the average value
   */
  std::pair<int, int> get_average(
    const std::array<std::pair<int, int>, average_size>& source);
  /**
   * @brief Get average values from left-side average array
   */
  std::pair<int, int> get_left_average();
  /**
   * @brief Get average values from left-side average array
   */
  std::pair<int, int> get_right_average();

  /**
   * @brief Compact average array results into LanePoints and manage the moving average filter
   *
   * @param left_line_found was left line found
   * @param right_line_found was right line found
   * 
   * @return found lane points
   */
  LanePoints get_from_average(bool left_line_found, bool right_line_found);
};