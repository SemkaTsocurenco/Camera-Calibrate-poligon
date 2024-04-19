//
// Created by egor on 27.07.22.
//

#ifndef CAMERACALIBRATION_CAMERACALIBRATOR_HPP
#define CAMERACALIBRATION_CAMERACALIBRATOR_HPP

#include <opencv2/opencv.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <utility>

class CameraCalibrator
{
public:
    CameraCalibrator(const char* serverIP_, int serverPort_, std::string  configPath_, std::string  matrixPath_);

    void init();

    void calibrate();



private:
    const char* serverIP;
    const int serverPort;
    const std::string configPath;
    const std::string matrixPath;
    bool is_inited;
    int longitudinal_count{};
    int lateral_count{};
    float square_size{};
    float distance_to_board{};
    float lateral_offset{};

    void _saveCalibration(const cv::Mat& transition_matrix);

};

#endif //CAMERACALIBRATION_CAMERACALIBRATOR_HPP
