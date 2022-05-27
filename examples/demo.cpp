#include "../src/autonomous_car.h"
#include <pigpio.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include "raspicam/raspicam_cv.h"

int main(int argc, char **argv){
	// Check if gpio in initialised correctly
	if (gpioInitialise() < 0) return 1;
	
	// set gpio pins for arduino output
    gpioSetMode(5, PI_OUTPUT);
    gpioSetMode(6, PI_OUTPUT);
    gpioSetMode(13, PI_OUTPUT);
    gpioSetMode(19, PI_OUTPUT);
    
    // setup of autononous car agent
	AutonomousCar car;
    
    // Setup camera with required parameters
    car.Setup(argc, argv);
    
    // Open camera to capture images
    if (!car.InitialiseCamera()){
        std::cout << "Failed to connect to camera" << std::endl;
        return -1;
    }
    
	// calculate time to calculate Frame rate
    auto start = std::chrono::system_clock::now();
    for (int i=0;i<1000;i++){
        std::cout << "Camera open: " << car.Camera.isOpened() << std::endl;
        // Grab a image from the camera and process the frame
        car.Capture();
        car.Perspective();
        car.Threshold();
        car.HoughTransform();
        
        if(car.LaneEnd()) {
            gpioWrite(5, 0);
            gpioWrite(6, 0); // decimal value = 8
            gpioWrite(13, 0);
            gpioWrite(19, 1);
            std::cout << "Lane End : Stop Car" << std::endl;
            return 1;
            }
        else {
            if (car.getDeltaDeviation() ==0){
                gpioWrite(5, 1); // decimal value = 1
                gpioWrite(6, 0);
                gpioWrite(13, 0);
                gpioWrite(19, 0);
                std::cout << "Move Forward" << std::endl;
            }
            else if (car.getDeltaDeviation() > 0 && car.getDeltaDeviation() < 10){
                gpioWrite(5, 0);
                gpioWrite(6, 1); // decimal value = 2
                gpioWrite(13, 0);
                gpioWrite(19, 0);
                std::cout << "Move Light Right" << std::endl;
            }
            else if (car.getDeltaDeviation() > 10 && car.getDeltaDeviation() < 20){
                gpioWrite(5, 1); // decimal value = 3
                gpioWrite(6, 1);
                gpioWrite(13, 0);
                gpioWrite(19, 0);
                std::cout << "Move Medium Right" << std::endl;
            }
            else if (car.getDeltaDeviation() > 20){
                gpioWrite(5, 0); // decimal value = 4
                gpioWrite(6, 0);
                gpioWrite(13, 1);
                gpioWrite(19, 0);
                std::cout << "Move Extreme Right" << std::endl;
            }
            else if (car.getDeltaDeviation() < 0 && car.getDeltaDeviation() > -10){
                gpioWrite(5, 1);
                gpioWrite(6, 0); // decimal value = 5
                gpioWrite(13, 1);
                gpioWrite(19, 0);
                std::cout << "Move Light Left" << std::endl;
            }
            else if (car.getDeltaDeviation() < -10 && car.getDeltaDeviation() > -20){
                gpioWrite(5, 0); // decimal value = 6
                gpioWrite(6, 1);
                gpioWrite(13, 1);
                gpioWrite(19, 0);
                std::cout << "Move Medium Left" << std::endl;
            }
            else if (car.getDeltaDeviation() < -20){
                gpioWrite(5, 1); // decimal value = 7
                gpioWrite(6, 1);
                gpioWrite(13, 1);
                gpioWrite(19, 0);
                std::cout << "Move Extreme Left" << std::endl;
            }        
        }
        
        // show the original feed from the camera
        cv::putText(car.frame, "Deviation from LaneCenter: "+std::to_string(car.getDeltaDeviation()), cv::Point2i(1,50), 0, 0.5, cv::Scalar(0,0,255), 1, cv::LINE_8);
        cv::namedWindow("original", cv::WINDOW_KEEPRATIO);
        cv::moveWindow("original", 0, 100);
        cv::resizeWindow("original", 640, 480);
        cv::imshow("original", car.frame);
        
        // show perspective wrap from region of interest
        cv::namedWindow("perspective", cv::WINDOW_KEEPRATIO);
        cv::moveWindow("perspective", 640, 100);
        cv::resizeWindow("perspective", 640, 480);
        cv::imshow("perspective", car.frame_perspective);
        
        // post processeed frame with lane markings
        cv::namedWindow("final", cv::WINDOW_KEEPRATIO);
        cv::moveWindow("final", 1280, 100);
        cv::resizeWindow("final", 640, 480);
        cv::imshow("final", car.frame_post_color);
        // cv::imwrite("Sample_"+std::to_string(i)+".jpg", car.frame);
    }
    
    // Calculate frame rate for the car
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = end - start;
    std::cout << "Time for capturing 1000 frames" << elapsed_time.count() << std::endl;
    std::cout << "Frame Rate:\t" << static_cast<double>(1000/elapsed_time.count()) << std::endl;
    
    // Closing the pigpio daemon
    gpioTerminate();
    car.Camera.release();
    return 0;
}
	
