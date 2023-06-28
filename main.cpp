/// Use this file with videos ///

// #include "lane-detection.h"
// #include "pre_processing.hpp"
// #include "lane_detection.hpp"

// #include "constants.hpp"

#include "TrackerManager.hpp"

int main()
{
    TrackerManager tracker_manager{"C:\\Users\\Admin\\dev\\projects\\cpp-lane-detection\\videos\\dubai_line_test_full.mp4"};
    // TrackerManager tracker_manager{"C:\\Users\\Admin\\dev\\projects\\cpp-lane-detection\\videos\\dubai_night.mp4"};

    tracker_manager.start_tracking();

    return 0;

    // Load source video
    // if (argc != 2) {
    //     std::cout << "Usage: ./exe path-to-video" << std::endl;
    //     return -1;
    // }

    // Initialize video capture for reading a videofile
    // VideoCapture cap(argv[1]);
    // cv::VideoCapture cap("C:\\Users\\Admin\\dev\\projects\\cpp-lane-detection\\videos\\dubai_line_test_full.mp4");
    // cv::VideoCapture cap("C:\\Users\\Admin\\dev\\projects\\cpp-lane-detection\\videos\\dubai_night.mp4");
    // cv::VideoWriter output("C:\\Users\\Admin\\dev\\projects\\cpp-lane-detection\\videos\\dubai_night_output.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(cst::kVideoWidth, cst::kVideoHeight));

    // Check if video can be opened
    // if (!cap.isOpened())
    // {
    //     std::cout << "Failed to open videofile!" << std::endl;
    //     return -1;
    // }

    // PreProcessing pre_processor;
    // LaneDetection lane_detection;

    // const cv::Point text_position(10, 30);

    // int frame_counter = 0;

    // cv::Mat frame;

    // Read and analyze video
    // while (true)
    // {
    //     cap >> frame;

    //     // Stop if frame is empty (end of video)
    //     if (frame.empty())
    //         break;

    //     ++frame_counter;

    //     // if (frame_counter < 2500)
    //     //     continue;

    //     // cv::imshow("Original", frame);
    //     // cv::putText(frame, "Frame: " + std::to_string(frame_counter), text_position, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));

    //     // cv::Mat edited = pre_processor.process_threshold(frame.clone());
    //     // cv::Mat edited = pre_processor.process_canny(frame.clone());
    //     // cv::Mat edited = pre_processor.process_custom(frame.clone());
    //     cv::Mat edited = pre_processor.process_adaptive_threshold(frame.clone());
    //     cv::imshow("Processed", edited);

    //     // cv::Mat maskedIMG = RegionOfInterest(edited);

    //     // Detect straight lines and draw the lanes if possible
    //     // std::vector<cv::Vec4i> linesP = hough_lines(edited, frame.clone(), false);
    //     // cv::Mat lanes = drawLanes(frame, linesP);
    //     // cv::Mat lanes = lane_detection.find_lines_hough(frame, edited);
    //     cv::Mat lanes = lane_detection.find_lines_custom(frame, edited);

    //     // cv::putText(lanes, "Frame: " + std::to_string(frame_counter), text_position, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));
    //     cv::putText(lanes, "Frame: " + std::to_string(frame_counter), text_position, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255));
    //     cv::imshow("Lanes", lanes);
    //     // cv::imshow("ROI", maskedIMG);
    //     // cv::imshow("filtered", edited);

    //     // output << lanes;
    //     // std::cout << "\r" << frame_counter;

    //     // Press  ESC on keyboard to exit
    //     if (cv::waitKey(1) == 27)
    //         break;
    // }

    // std::cout << std::endl;

    // cap.release();
    // // output.release();

    // return 0;
}