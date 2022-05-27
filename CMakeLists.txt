cmake_minimum_required(VERSION 3.18.4)
project(autonomous_car VERSION  1.0.0)
set(raspicam_DIR "/usr/local/lib/cmake")
find_package(OpenCV REQUIRED)
find_package(raspicam REQUIRED)

add_executable(
    demo
    examples/demo.cpp
    src/autonomous_car.cpp
)

target_link_libraries (demo ${raspicam_CV_LIBS} pigpio)