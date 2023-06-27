#include "TrackerManager.hpp"

#include "LaneTracking.hpp"
#include "PreProcessing.hpp"
// #include "RandomLaneDetection.hpp"

TrackerManager::TrackerManager(const std::string video_path)
  : video_path_(video_path)
{
}

void
TrackerManager::start_tracking()
{
  cv::VideoCapture cap(video_path_);

  if (!cap.isOpened()) {
    std::cout << "Failed to open videofile!" << std::endl;
    return;
  }

  PreProcessing pre_processor;
  //   RandomLaneDetection lane_detection;
  LaneTracking lane_tracking;

  const cv::Point text_position(10, 30);

  int frame_counter = 0;

  bool good_to_retrack = false;

  cv::Mat frame;

  while (true) {
    cap >> frame;

    // Stop if frame is empty (end of video)
    if (frame.empty())
      break;

    ++frame_counter;

    cv::Mat edited = pre_processor.process_adaptive_threshold(frame.clone());
    // cv::imshow("Processed", edited);

    LanePoints result = good_to_retrack
                          ? lane_tracking.retrack(edited, frame)
                          : lane_tracking.restart_tracking(edited, frame);

    good_to_retrack = are_good_to_retrack(result);

    frame = draw_points(frame, result);
    cv::putText(frame,
                "Frame: " + std::to_string(frame_counter),
                text_position,
                cv::FONT_HERSHEY_PLAIN,
                1.5,
                cv::Scalar(255, 255, 255));
    cv::imshow("Lanes", frame);

    // Press  ESC on keyboard to exit
    if (cv::waitKey(1) == 27)
      break;
  }

  std::cout << std::endl;

  cap.release();
  // output.release();
}

cv::Mat
TrackerManager::draw_points(cv::Mat frame, const LanePoints& lane_points)
{
  const int left_top = lane_points.left_top;
  const int left_bottom = lane_points.left_bottom;
  const int right_top = lane_points.right_top;
  const int right_bottom = lane_points.right_bottom;

  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

  // Draw lines and fill poly made up of the four points described above if both
  // bools are true
  cv::Mat dst; // Holds blended image

  if (lane_points.draw_left)
    cv::line(frame,
             cv::Point(left_top, cst::y_top),
             cv::Point(left_bottom, cst::y_bottom),
             cv::Scalar(255, 0, 0),
             7);

  if (lane_points.draw_right)
    cv::line(frame,
             cv::Point(right_top, cst::y_top),
             cv::Point(right_bottom, cst::y_bottom),
             cv::Scalar(255, 0, 0),
             7);

  if (lane_points.draw_left && lane_points.draw_right) {
    cv::Point pts[4] = {
      cv::Point(left_top, cst::y_top),       // Starting point left lane
      cv::Point(left_bottom, cst::y_bottom), // Ending point left lane
      cv::Point(right_top, cst::y_bottom),   // Ending point right lane
      cv::Point(right_bottom, cst::y_top)    // Starting point right lane
    };

    // cv::fillConvexPoly(mask, pts, 4, cv::Scalar(235, 229, 52)); // Color is
    // light blue

    // Blend the mask and source image together
    addWeighted(frame, 1, mask, 0.3, 0.0, dst);
  }

  return frame;
}

bool
TrackerManager::are_good_to_retrack(const LanePoints& lane_points)
{
  if (!lane_points.draw_left && !lane_points.draw_right)
    return false;

  const int left_margin = cst::x_center - cst::retrack_x_delta;
  if (lane_points.left_top > left_margin ||
      lane_points.left_bottom > left_margin)
    return false;

  const int right_margin = cst::x_center + cst::retrack_x_delta;
  if (lane_points.right_top < right_margin ||
      lane_points.right_bottom < right_margin)
    return false;

  return true;
}