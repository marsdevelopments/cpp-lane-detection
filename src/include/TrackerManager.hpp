#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "LanePoints.hpp"

class TrackerManager
{
public:
  TrackerManager(const std::string video_path);

  void start_tracking();

private:
  cv::Mat draw_points(cv::Mat frame, const LanePoints& lane_points);

  bool are_good_to_retrack(const LanePoints& lane_points);

  const std::string video_path_;
};