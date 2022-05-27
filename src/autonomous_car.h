#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "raspicam/raspicam_cv.h"
#include <vector>
#include <algorithm>
#include <string>

/*
 * Car class which contains all the functions and members to
 * run the image processing pipeline to drive the car on a track
*/

class AutonomousCar{
	public:
		cv::Mat frame, frame_perspective, frame_post_color;
	public:
		raspicam::RaspiCam_Cv Camera;
		cv::Mat frame1, matrix, gray_frame_perspective, frame_threshold, frame_edge, frame_post, frame_bw;
		cv::Mat horizontal, horizontalStructure;
		int horizontal_size {0};
		cv::Point2f Source[4] = {cv::Point2f(75,200), cv::Point2f(295,200), cv::Point2f(50,240), cv::Point2f(320,240)};
		cv::Point2f Destination[4] = {cv::Point2f(50,0), cv::Point2f(310,0), cv::Point2f(50,240), cv::Point2f(310,240)};
		int LaneCenter {0}, FrameCenter {0}, DeltaDeviation {0};	

	public:
		bool InitialiseCamera();
		void Setup (int argc, char **argv);
		void Capture();
		void Perspective();
		void Threshold();
		void HoughTransform();
		int LaneEnd();
		int getDeltaDeviation();
};
