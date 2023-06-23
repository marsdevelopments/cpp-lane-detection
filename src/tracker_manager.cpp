#include "tracker_manager.hpp"

#include "pre_processing.hpp"
#include "random_lane_detection.hpp"
#include "random_lane_tracking.hpp"

TrackerManager::TrackerManager(const std::string video_path)
    : video_path_(video_path)
{
}

void TrackerManager::start_tracking()
{
    cv::VideoCapture cap(video_path_);

    if (!cap.isOpened())
    {
        std::cout << "Failed to open videofile!" << std::endl;
        return;
    }

    PreProcessing pre_processor;
    RandomLaneDetection lane_detection;
    RandomLaneTracking lane_tracking;

    const cv::Point text_position(10, 30);

    int frame_counter = 0;

    cv::Mat frame;

    while (true)
    {
        cap >> frame;

        // Stop if frame is empty (end of video)
        if (frame.empty())
            break;

        ++frame_counter;

        // if (frame_counter < 2500)
        //     continue;

        // cv::imshow("Original", frame);
        // cv::putText(frame, "Frame: " + std::to_string(frame_counter), text_position, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));

        cv::Mat edited = pre_processor.process_adaptive_threshold(frame.clone());
        cv::imshow("Processed", edited);

        // Detect straight lines and draw the lanes if possible
        cv::Vec4i detected_lines = lane_detection.find_lines(frame, edited);

        lane_tracking.track_lines()

        // cv::putText(lanes, "Frame: " + std::to_string(frame_counter), text_position, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255));
        // cv::imshow("Lanes", lanes);
        // cv::imshow("ROI", maskedIMG);
        // cv::imshow("filtered", edited);

        // output << lanes;
        // std::cout << "\r" << frame_counter;

        // Press  ESC on keyboard to exit
        if (cv::waitKey(1) == 27)
            break;
    }

    std::cout << std::endl;

    cap.release();
    // output.release();
}