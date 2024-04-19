//
// Created by egor on 23.08.22.
//

#ifndef CAMERACHESSBOARDCALIBRATION_CAMERACALIBRATOR_H
#define CAMERACHESSBOARDCALIBRATION_CAMERACALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <utility>
#include <queue>
#include <algorithm>
#include <numeric>
#include <condition_variable>

class CameraCalibrator
{
public:
    CameraCalibrator(const char* serverIP_, int serverPort_, std::string calibrationPath_, std::string  inputSource, int device);

    ~CameraCalibrator();

    void save_images(const std::string& imagesPath, bool inputVideo, int duration_);

    void create_markers(const std::vector<int>& ids);

    void calibrate(const std::string& imagesPath, int rows, int cols);

    void distance_calibrate(float markerSize);

    void create_board();

    void calibrate_with_charuco_board(const std::string& imagesPath);

    void show();

    cv::Mat _nextFrame();

    bool init();

private:
    const char* serverIP;
    const int serverPort;
    const std::string calibrationPath;
    const std::string transformPath;
    bool is_inited;
    int _socket;
    int _device;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    std::vector<int> aruco_ids;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat R;
    cv::Mat T;
    std::queue<cv::Mat> frameQueue;
    std::mutex m;
    std::condition_variable frameReady;
    bool terminated;
    std::string _inputSource;

    void _saveCalibration();

    void _saveTransform(const cv::Mat& perspectiveTransformMatrix);

    void _readCalibration();

    void _init_stream();

    bool _init_socket();

    cv::Mat _readTransform();
};

#endif //CAMERACHESSBOARDCALIBRATION_CAMERACALIBRATOR_H
