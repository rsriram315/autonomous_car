#include "autonomous_car.h"

// Open raspberrypi camera for capturing images
bool AutonomousCar::InitialiseCamera(){
    std::cout << "Connecting to camera" << std::endl;
    std::cout << "Camera ID: " << Camera.getId() << std::endl;
    Camera.open();
    return Camera.isOpened();
}

// Setup raspberry pi camera with required parameters
void AutonomousCar::Setup (int argc, char **argv){
    Camera.set(cv::CAP_PROP_FRAME_WIDTH, ("-w", argc, argv, 390));
    Camera.set(cv::CAP_PROP_FRAME_HEIGHT, ("-h", argc, argv, 240));
    Camera.set(cv::CAP_PROP_BRIGHTNESS, ("-br", argc, argv, 50));
    Camera.set(cv::CAP_PROP_CONTRAST, ("-co", argc, argv, 50));
    Camera.set(cv::CAP_PROP_SATURATION, ("-sa", argc, argv, 50));
    Camera.set(cv::CAP_PROP_GAIN, ("-g", argc, argv, 50));
    Camera.set(cv::CAP_PROP_FPS, ("-fps", argc, argv, 60));
}

// Capture a frame from the camera
void AutonomousCar::Capture(){
    Camera.grab();
    Camera.retrieve(frame);
    //cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
}

// Creates a perspective wrap of the region of interest from the camera capture
void AutonomousCar::Perspective(){
    // input image area of interest
    cv::line(frame, Source[0], Source[1], cv::Scalar(0,0,255), 2);
    cv::line(frame, Source[1], Source[3], cv::Scalar(0,0,255), 2);
    cv::line(frame, Source[3], Source[2], cv::Scalar(0,0,255), 2);
    cv::line(frame, Source[2], Source[0], cv::Scalar(0,0,255), 2);
    
    // prespective area of interest for mapping
    //cv::line(frame, Destination[0], Destination[1], cv::Scalar(0,255,0), 2);
    //cv::line(frame, Destination[1], Destination[3], cv::Scalar(0,255,0), 2);
    //cv::line(frame, Destination[3], Destination[2], cv::Scalar(0,255,0), 2);
    //cv::line(frame, Destination[2], Destination[0], cv::Scalar(0,255,0), 2);
    
    // perspective transformation
    matrix = cv::getPerspectiveTransform(Source, Destination);
    cv::warpPerspective(frame, frame_perspective, matrix, cv::Size(360,240));
}

// Performs gaussian blur, thresholding and canny edge detection on the inout image
void AutonomousCar::Threshold(){
    cv::cvtColor(frame_perspective, gray_frame_perspective, cv::COLOR_BGR2GRAY);
    cv::blur(gray_frame_perspective, gray_frame_perspective, cv::Size(3,3));
    cv::inRange(gray_frame_perspective, 100, 250, frame_threshold);
    cv::Canny(gray_frame_perspective, frame_edge, 150, 400, 3);
    cv::add(frame_threshold, frame_edge, frame_post);
    cv::cvtColor(frame_post, frame_post_color, cv::COLOR_GRAY2RGB);
}

// Perform hough transform to extract lane lines from the image
// and the deviation of the lane centre from the frame centre
void AutonomousCar::HoughTransform(){
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame_post, lines, 1, CV_PI/180, 50, 50, 10);
    // std::cout << "No of hough lines: " << lines.size()<< std::endl;
    std::sort(lines.begin(), lines.end(),
                            [](const cv::Vec4i& a, const cv::Vec4i& b){ return a[0] < b[0];});
    if (lines.size() > 2){
        lines.erase(lines.begin()+1, lines.end()-1);
    }
    // std::cout << "No of hough lines: " << lines.size()<< std::endl;
    for (size_t i =0; i<lines.size(); i++){
        cv::Vec4i line = lines[i];
        cv::line(frame_post_color, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,255,0), 2);
    }
    
    if (lines.size() == 2){
    LaneCenter = (lines[1][0] - lines[0][0]) / 2 + lines[0][0];
    std::cout << "LaneCenter: " << LaneCenter << std::endl;
    FrameCenter = 180;
    cv::line(frame_post_color, cv::Point2f(LaneCenter,0), cv::Point2f(LaneCenter,240), cv::Scalar(0,255,0), 2);
    cv::line(frame_post_color, cv::Point2f(FrameCenter,0), cv::Point2f(FrameCenter,240), cv::Scalar(255,0,0), 2);
    DeltaDeviation = LaneCenter - FrameCenter;
    } 
}

// Checks if lane end is reached and return 1 or 0 accordingly
int AutonomousCar::LaneEnd(){
    cv::adaptiveThreshold(frame_post, frame_bw, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 25 ,-2);
    horizontal = frame_bw.clone();
    horizontal_size = horizontal.cols / 30;
    cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontal_size, 1));
    cv::erode(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));
    cv::dilate(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));    
    std::vector<cv::Vec4i> lanes;
    cv::HoughLinesP(horizontal, lanes, 1, CV_PI/2, 50, 100, 1);
    for (size_t i =0; i<lanes.size(); i++){
        cv::Vec4i line = lanes[i];
        cv::line(frame_post_color, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(147,20,255), 2);
    }
    if (!lanes.empty()) return 1;
    else return 0;    
}

// get computed Deltadeviation value
int AutonomousCar::getDeltaDeviation(){return DeltaDeviation;}
