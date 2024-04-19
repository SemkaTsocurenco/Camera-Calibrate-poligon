//
// Created by egor on 27.07.22.
//


#include "CameraCalibrator.hpp"

CameraCalibrator::CameraCalibrator(const char *serverIP_, int serverPort_, std::string  configPath_, std::string  matrixPath_)
    :serverIP(serverIP_)
    ,serverPort(serverPort_)
    ,configPath(std::move(configPath_))
    ,matrixPath(std::move(matrixPath_))
    ,is_inited(false)
{}

void CameraCalibrator::calibrate()
{
    if (!is_inited)
        return;

    int sokt;
    struct  sockaddr_in serverAddr{};
    socklen_t           addrLen = sizeof(struct sockaddr_in);

    if ((sokt = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "socket() failed" << std::endl;
    }

    serverAddr.sin_family = PF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(serverIP);
    serverAddr.sin_port = htons(serverPort);

    if (connect(sokt, (sockaddr*)&serverAddr, addrLen) < 0) {
        std::cerr << "connect() failed!" << std::endl;
    }

    cv::Mat img;
    cv::Mat gray;
    img = cv::Mat::zeros(600, 960, CV_8UC3);
    gray = cv::Mat::zeros(600, 960, CV_8UC1);
    int imgSize = static_cast<int>(img.total() * img.elemSize());
    uchar* iptr = img.data;
    size_t bytes;
    int key;

    //make img continuos
    if (!img.isContinuous()) {
        img = img.clone();
    }

    while (key != 'q')
    {

        if ((bytes = recv(sokt, iptr, imgSize, MSG_WAITALL)) == -1) {
            std::cerr << "recv failed, received bytes = " << bytes << std::endl;
        }
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        auto result = cv::findChessboardCorners(gray, cv::Size(longitudinal_count, lateral_count) ,corners);
        if (result)
        {
            std::vector<int> indices = {0, longitudinal_count - 1, longitudinal_count * (lateral_count - 1),
                                        longitudinal_count * lateral_count - 1};
            std::vector<int> bottom_indices;
            std::vector<int> top_indices;
            if (corners[indices[0]].y > corners[indices[1]].y) {
                bottom_indices = {indices[0], indices[2]};
                top_indices = {indices[1], indices[3]};
            } else {
                bottom_indices = {indices[1], indices[3]};
                top_indices = {indices[0], indices[2]};
            }

            int top_left, top_right, bottom_left, bottom_right;
            if (corners[top_indices[0]].x < corners[top_indices[1]].x) {
                top_left = top_indices[0];
                top_right = top_indices[1];
            } else {
                top_left = top_indices[1];
                top_right = top_indices[0];
            }

            if (corners[bottom_indices[0]].x < corners[bottom_indices[1]].x) {
                bottom_left = bottom_indices[0];
                bottom_right = bottom_indices[1];
            } else {
                bottom_left = bottom_indices[1];
                bottom_right = bottom_indices[0];
            }

            auto length = square_size * static_cast<float>(longitudinal_count - 1);
            auto width = square_size * static_cast<float>(lateral_count - 1);

            auto pt_A = corners[top_left];
            auto pt_B = corners[bottom_left];
            auto pt_C = corners[bottom_right];
            auto pt_D = corners[top_right];

            auto input_pts = std::vector<cv::Point2f>({pt_A, pt_B, pt_C, pt_D});
            auto output_pts = std::vector<cv::Point2f>(
                    {cv::Point2f(0, 0), cv::Point2f(0, length), cv::Point2f(width, length),
                     cv::Point2f(width, 0)});

            std::cout << input_pts << std::endl;
            std::cout << output_pts << std::endl;
            auto calibration_matrix = cv::getPerspectiveTransform(input_pts, output_pts);
            std::cout << calibration_matrix << std::endl;

            _saveCalibration(calibration_matrix);
            cv::drawChessboardCorners(img,cv::Size(longitudinal_count, lateral_count), corners, result);
            cv::imshow("new", img);
            cv::waitKey(0);
        }
        cv::imshow("new", img);

        key = cv::waitKey(10) >= 0;
        if (key)
            break;
    }

    close(sokt);
}

void CameraCalibrator::init()
{
    cv::FileStorage calibrationConfig(configPath, cv::FileStorage::READ);
    if (calibrationConfig.isOpened())
    {

        calibrationConfig["distanceToBoard"] >> distance_to_board;
        calibrationConfig["lateralOffset"] >> lateral_offset;
        calibrationConfig["lateralCount"] >> lateral_count;
        calibrationConfig["longitudinalCount"] >> longitudinal_count;
        calibrationConfig["squareSize"] >> square_size;

        is_inited = true;

        std::cout << "Camera calibration loaded" << std::endl;

        calibrationConfig.release();

    }
}

void CameraCalibrator::_saveCalibration(const cv::Mat &transition_matrix)
{
    cv::FileStorage calibrationConfig(matrixPath, cv::FileStorage::WRITE);
    float longitudinal_offset = distance_to_board + square_size * static_cast<float>(longitudinal_count - 1);
    if (calibrationConfig.isOpened())
    {
        calibrationConfig << "transitionMatrix" << transition_matrix;
        calibrationConfig << "lateralOffset" << lateral_offset;
        calibrationConfig << "longitudinalOffset" << longitudinal_offset;
        calibrationConfig.release();
        std::cout << "Camera calibration saved" << std::endl;
    }
    else
    {
        std::cerr << "Camera calibration saving failed. File: " << matrixPath << " IO Error\n";
    }
}


