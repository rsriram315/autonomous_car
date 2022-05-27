#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "raspicam_cv.h"


// Function prototypes
void Setup (int argc, char **argv, raspicam::RaspiCam_Cv &Camera);


int main(int argc, char** argv){
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    int i{0};
    Setup(argc, argv, Camera);
    
    // Connect to camera and exit if failure
    std::cout << "Connecting to pi camera" << std::endl;
    if (!Camera.open()){
        std::cout << "Failed to connec to camera" << std::endl;
        return -1;
    }
    std::cout << "Camera ID" << Camera.getId() << std::endl;
    
    // Capture consequtive images with raspicam
    for(i; i<300; i++){
	Camera.grab();
	Camera.retrieve(image);
	cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	cv::imshow("Capture", image);
	cv::imwrite("Sample_"+std::to_string(i)+".jpg", image);
	cv::waitKey();
	}
	return 0;
}
 
void Setup (int argc, char **argv, raspicam::RaspiCam_Cv &Camera){
    Camera.set(cv::CAP_PROP_FRAME_WIDTH, ("-w", argc, argv, 390));
    Camera.set(cv::CAP_PROP_FRAME_HEIGHT, ("-h", argc, argv, 240));
    Camera.set(cv::CAP_PROP_BRIGHTNESS, ("-br", argc, argv, 50));
    Camera.set(cv::CAP_PROP_CONTRAST, ("-co", argc, argv, 50));
    Camera.set(cv::CAP_PROP_SATURATION, ("-sa", argc, argv, 50));
    Camera.set(cv::CAP_PROP_GAIN, ("-g", argc, argv, 50));
    Camera.set(cv::CAP_PROP_FPS, ("-fps", argc, argv, 60));
}
