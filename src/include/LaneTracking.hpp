#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "LanePoints.hpp"
#include "constants.hpp"

// lane finder class
class LaneTracking
{
public:
  /**
   * @brief Start lane tracking from scrath, and search the whole ROI
   *
   * @param edited_frame edited frame by PreProcessing which will be used to
   * search lanes
   * @param debug_frame frame on which debug lines will be drawn
   *
   * @return found lane points
   */
  LanePoints restart_tracking(const cv::Mat& edited_frame,
                              const cv::Mat& debug_frame);

  /**
   * @brief Search in a limited area, defined by previously found lane points
   * and @ref cst::kRetrackDeltaX
   *
   * @param edited_frame edited frame by PreProcessing in which search will be
   * done
   * @param debug_frame frame on which debug lines will be drawn
   *
   * @return found lane points
   */
  LanePoints retrack(const cv::Mat& edited_frame, const cv::Mat& debug_frame);

private:

#pragma region Average Filter

  static const size_t average_size_ = 10;

  // last average_size ammount of left line points
  std::array<std::pair<int, int>, average_size_> left_average_ = {
    { { 0, 0 } /* , { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } */ }
  };

  // last average_size ammount of right line points
  std::array<std::pair<int, int>, average_size_> right_average_ = {
    { { 0, 0 } /* , { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } */ }
  };

  // ammount of frames neither of the lines where found
  size_t missing_frames_counter_ = 0;
  // if true, will restart average filter
  bool long_missing_flag_ = false;

#pragma endregion

  // left line points
  std::pair<int, int> left_line_;
  // right line points
  std::pair<int, int> right_line_;

  // most left a left line can be
  const int bottom_left_x_ = cst::trapezoid_roi_points.at(0).x;
  const int top_left_x_ = cst::trapezoid_roi_points.at(1).x;
  // most right a right line can be
  const int top_right_x_ = cst::trapezoid_roi_points.at(2).x;
  const int bottom_right_x_ = cst::trapezoid_roi_points.at(3).x;

  cv::Mat edited_frame_;
  
  LanePoints last_lane_points;

private:
  /**
   * @brief Evaluate validity of left_line_ & right_line_, and add them to the filter
   */
  void process_lines();

  /**
   * @brief Replace all values in averages arrays with zero
   */
  void clear_averages();

  /**
   * @brief Add a number to an average array
   *
   * @param source array into which to add values
   * @param x1 top-side x value
   * @param x2 bottom-side x value
   */
  void add_average(std::array<std::pair<int, int>, average_size_>& source,
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
    const std::array<std::pair<int, int>, average_size_>& source);
  /**
   * @brief Get average values from left-side average array
   */
  std::pair<int, int> get_left_average();
  /**
   * @brief Get average values from left-side average array
   */
  std::pair<int, int> get_right_average();

  /**
   * @brief Compact average array results into LanePoints and manage the moving
   * average filter
   *
   * @param left_line_found was left line found
   * @param right_line_found was right line found
   *
   * @return found lane points
   */
  LanePoints get_from_average(bool left_line_found, bool right_line_found);
};