#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "LanePoints.hpp"

// manager of whole operation
class TrackerManager
{
public:
  TrackerManager(const std::string video_path);

  void start_tracking();

private:
  const std::string video_path_;

private:
  /**
   * @brief draw a lane from lane_points
   *
   * @param frame frame on which to draw on
   * @param lane_points lane points to draw
   *
   * @return frame with drawn lane
   */
  cv::Mat draw_points(cv::Mat frame, const LanePoints& lane_points);

  /**
   * @brief Checks if lane points positions are not suspicious,
   * or if they should not be confirmed by a reset
   *
   * @param lane_points lane points to check
   *
   * @return if tracking reset should be triggered
   */
  bool are_good_to_retrack(const LanePoints& lane_points);
};