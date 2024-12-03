//
// Created by Tim Mascal on 12/3/24.
//
// The purpose of this file is to interface with the ArduCam with ROS


#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open the default camera (usually the laptop's webcam)
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }


    // Set camera resolution
    int width = 1920;  // Desired width
    int height = 1080;  // Desired height
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;
        // Capture a frame
        cap >> frame;

        // Check if frame is empty
        if (frame.empty()) {
            std::cerr << "Error: Could not grab a frame." << std::endl;
            break;
        }

        // Display the frame
        cv::imshow("Camera Feed", frame);

        // Wait for 'q' key press for 30 ms. If 'q' key is pressed, break loop
        if (cv::waitKey(30) == 'q') {
            break;
        }

    }

    // Release the camera or video cap
    cap.release();
    cv::destroyAllWindows();

    return 0;
}