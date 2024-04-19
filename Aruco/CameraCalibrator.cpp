//
// Created by egor on 23.08.22.
//

#include "CameraCalibrator.h"

std::vector<int> argsort(const std::vector<int> &array)
{
    std::vector<int> indices(array.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&array](int left, int right) -> bool {
                  // sort indices according to corresponding array element
                  return array[left] < array[right];
              });

    return indices;
}

CameraCalibrator::CameraCalibrator(const char *serverIP_, int serverPort_, std::string  calibrationPath_, std::string  inputSource, int device)
        :serverIP(serverIP_)
        ,serverPort(serverPort_)
        ,calibrationPath(std::move(calibrationPath_))
        ,is_inited(false)
        ,_socket(0)
        ,terminated(false)
        ,_inputSource(std::move(inputSource))
        ,_device(device)
        ,transformPath("PerspectiveTransformMatrix.yaml")
{
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    board = cv::aruco::CharucoBoard::create(6, 8, 0.04f, 0.02f, dictionary);
    parameters = cv::aruco::DetectorParameters::create();
}

void CameraCalibrator::save_images(const std::string& imagesPath, bool inputVideo = false, int duration_ = 1)
{
    if (!is_inited)
    {
        std::cout << "Can't receive images without connection" << std::endl;
        return;
    }

    int key;
    int imageNumber = 1;
    std::vector<cv::Mat> images;
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    auto duration = std::chrono::milliseconds(1000 * duration_);
    auto start = std::chrono::steady_clock::now();

    cv::VideoWriter videoWriter;
    if (inputVideo)
    {
        videoWriter.open(imagesPath, codec, 25, cv::Size(960, 600), true);
        if (!videoWriter.isOpened())
        {
            std::cout  << "Could not open the output video for write: " << imagesPath << std::endl;
            return;
        }
    }
    while (key != 'q')
    {
        auto img = _nextFrame();
        if (!inputVideo)
        {
            auto end = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start) > duration)
            {
                start = std::chrono::steady_clock::now();
                std::cout << "save image num" << imageNumber << std::endl;
                std::cout << "prepare for next image" << std::endl;
                imageNumber++;
                images.push_back(img.clone());
            }
        }
        else
        {
            videoWriter << img;
        }
        key = cv::waitKey(10) >= 0;
        cv::imshow("new", img);
        if (key)
            break;
    }
    if (!inputVideo)
        imwrite(imagesPath, images);
}

void CameraCalibrator::calibrate(const std::string& imagesPath, int rows, int cols)
{
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f> > imgpoints;
    std::vector<cv::Point3f> objp;

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        objp.emplace_back(j,i,0);
    }
    cv::Mat gray;
    std::vector<cv::Mat> images;
    cv::imreadmulti(imagesPath, images);
    bool success;
    std::vector<cv::Point2f> corner_pts;
    for (auto& frame: images)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray, cv::Size(rows, cols), corner_pts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (success)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            // Displaying the detected corner points on the checkerboard
            cv::drawChessboardCorners(frame, cv::Size(rows, cols), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        cv::destroyAllWindows();

    }
    /*
    * * Performing camera calibration by
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the
    * detected corners (imgpoints)
    */

    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), _cameraMatrix, _distCoeffs, R, T);
    _saveCalibration();

}

void CameraCalibrator::_saveCalibration()
{
    cv::FileStorage calibrationConfig(calibrationPath, cv::FileStorage::WRITE);
    if (calibrationConfig.isOpened())
    {
        calibrationConfig << "cameraMatrix" << _cameraMatrix;
        calibrationConfig << "distCoeffs" << _distCoeffs;
        calibrationConfig << "Rvec" << R;
        calibrationConfig << "Tvec" << T;
        calibrationConfig.release();
        std::cout << "Camera calibration saved" << std::endl;
    }
    else
    {
        std::cerr << "Camera calibration saving failed. File: " <<  calibrationPath << " IO Error\n";
    }
}

void CameraCalibrator::create_markers(const std::vector<int>& ids)
{
    cv::Mat markerImage;
    aruco_ids = ids;
    for (auto id: ids)
    {
        cv::aruco::drawMarker(dictionary, id, 2000, markerImage, 1);
        cv::imwrite("marker" + std::to_string(id) + ".png", markerImage);

    }
}

void CameraCalibrator::distance_calibrate(float markerSize)
{
    if (!is_inited)
    {
        std::cout << "Can't receive images without connection" << std::endl;
        return;
    }
    _readCalibration();

    int key;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    while (key != 'q')
    {
        auto img = _nextFrame();
        cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        cv::Mat outputImage = img.clone();
        if (!markerIds.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
            cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, _cameraMatrix, _distCoeffs, rvecs, tvecs);
            std::vector<cv::Point2f> input_points;
            std::vector<cv::Point2f> output_points;
            auto indices = argsort(markerIds);
            cv::Vec3d rot = rvecs[indices[0]];
            rot[0] = 0;
            cv::Mat rmatrix;
            std::swap(rot[1], rot[2]);
            cv::Rodrigues(rot, rmatrix);
            // draw axis for each marker
            std::cout << "frame" << std::endl;
            for(int indice : indices)
            {
                auto tvec = cv::Mat(rmatrix * cv::Mat(tvecs[indice]));
                cv::Point2f point(static_cast<float>(tvec.at<double>(0)), static_cast<float>(tvec.at<double>(2)));
                output_points.push_back(point);
                std::cout << "id: " << markerIds[indice] << std::endl;
                std::cout << point << std::endl;
                point.x = (markerCorners[indice][0].x + markerCorners[indice][1].x + markerCorners[indice][2].x + markerCorners[indice][3].x) / 4;
                point.y = (markerCorners[indice][0].y + markerCorners[indice][1].y + markerCorners[indice][2].y + markerCorners[indice][3].y) / 4;
                input_points.push_back(point);
//                std::cout << markerCorners[0][0] << " " << markerCorners[0][1] << " " << markerCorners[0][2] << " " << markerCorners[0][3] << std::endl;
                cv::drawFrameAxes(outputImage, _cameraMatrix, _distCoeffs, rvecs[indice], tvecs[indice], 0.1);
            }
            if (markerIds.size() == 4)
            {
                auto transformMatrix = cv::getPerspectiveTransform(input_points, output_points);
                cv::line(outputImage, input_points[0], input_points[1], cv::Scalar(255,255,0), 3);
                cv::line(outputImage, input_points[1], input_points[2], cv::Scalar(255,255,0), 3);
                cv::line(outputImage, input_points[2], input_points[3], cv::Scalar(255,255,0), 3);
                cv::line(outputImage, input_points[3], input_points[0], cv::Scalar(255,255,0), 3);
                cv::Vec3d point(200,100,1);
                auto new_point = cv::Mat(transformMatrix * cv::Mat(point));
                auto d = new_point.at<double>(2);
                auto result = cv::Mat(new_point / d);
                std::cout << "x :" << result.at<double>(0) << " y: " << result.at<double>(1) << std::endl;
                cv::circle(outputImage, cv::Point(200,100), 0, cv::Scalar(0, 255, 255), 5);
                _saveTransform(transformMatrix);
                cv::imshow("new", outputImage);
                cv::waitKey(0);
                break;
            }
        }
        cv::imshow("new", outputImage);
        key = cv::waitKey(10) >= 0;
        if (key)
            break;
    }
}

bool CameraCalibrator::_init_socket()
{
    if (is_inited)
        return true;
    struct  sockaddr_in serverAddr{};
    socklen_t           addrLen = sizeof(struct sockaddr_in);

    if ((_socket = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "socket() failed" << std::endl;
        return false;
    }

    serverAddr.sin_family = PF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(serverIP);
    serverAddr.sin_port = htons(serverPort);

    if (connect(_socket, (sockaddr*)&serverAddr, addrLen) < 0)
    {
        std::cerr << "connect() failed!" << std::endl;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::thread thread_( [this]
    {
        cv::Mat img;
        img = cv::Mat::zeros(600, 960, CV_8UC3);
        int imgSize = static_cast<int>(img.total() * img.elemSize());
        uchar* iptr = img.data;
        size_t bytes;

        if (!img.isContinuous())
        {
            img = img.clone();
        }

        while (!terminated)
        {
            if ((bytes = recv(_socket, iptr, imgSize, MSG_WAITALL)) == -1)
            {
                std::cerr << "recv failed, received bytes = " << bytes << std::endl;
                continue;
            }
            std::lock_guard<std::mutex> lock(m);
            frameQueue.push(img.clone());
            if (frameQueue.size() > 5)
                frameQueue.pop();
            frameReady.notify_one();
        }
    });
    thread_.detach();
    std::cout << "Socket inited" << std::endl;
    return true;
}

void CameraCalibrator::_readCalibration()
{
    cv::FileStorage calibrationConfig(calibrationPath, cv::FileStorage::READ);
    if (calibrationConfig.isOpened())
    {

        calibrationConfig["cameraMatrix"] >> _cameraMatrix;
        calibrationConfig["distCoeffs"] >> _distCoeffs;
        calibrationConfig["Rvec"] >> R;
        calibrationConfig["Tvec"] >> T;
        std::cout << "Camera calibration loaded" << std::endl;

        calibrationConfig.release();
    }
}

void CameraCalibrator::create_board()
{
    cv::Mat boardImage;
    board->draw(cv::Size(600, 800), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}

void CameraCalibrator::calibrate_with_charuco_board(const std::string& imagesPath)
{
    cv::Size imgSize(600, 960);
    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;

    cv::Mat gray;
    std::vector<cv::Mat> images;
    cv::imreadmulti(imagesPath, images);

    for (auto& frame: images)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(gray, board->dictionary, markerCorners, markerIds, parameters);

        if (!markerIds.empty())
        {
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray, board, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            if (!charucoIds.empty())
            {
                cv::aruco::drawDetectedCornersCharuco(frame, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                if (charucoCorners.size() > 5)
                {
                    allCharucoIds.push_back(charucoIds);
                    allCharucoCorners.push_back(charucoCorners);
                }
            }
        }

        cv::imshow("Image",frame);
        cv::waitKey(0);
        cv::destroyAllWindows();

    }
    std::vector<cv::Mat> rvecs, tvecs;
    std::cout << "Charuco size: " << allCharucoIds.size() << std::endl;
    double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, _cameraMatrix, _distCoeffs, rvecs, tvecs);
    _saveCalibration();
    std::cout << "Rep error: " << repError << std::endl;
}

cv::Mat CameraCalibrator::_nextFrame()
{
    std::unique_lock<std::mutex> lock(m);
    frameReady.wait(lock,
                    [this]()
    {
        return !frameQueue.empty();
    });
    cv::Mat frame = frameQueue.front().clone();
    frameQueue.pop();
    return frame;
}

CameraCalibrator::~CameraCalibrator()
{
    terminated = true;
    std::cout << "terminate" << std::endl;
    if (_inputSource == "socket")
        close(_socket);
}

void CameraCalibrator::_init_stream()
{
    if (is_inited)
        return;

    // std::thread thread_([this]
    // {
    //     cv::VideoCapture inputVideo;
    //     inputVideo.open(_device);
    //     while (inputVideo.grab() and !terminated)
    //     {
    //         cv::Mat image;
    //         inputVideo.retrieve(image);
    //         std::lock_guard<std::mutex> lock(m);
    //         frameQueue.push(image.clone());
    //         if (frameQueue.size() > 5)
    //             frameQueue.pop();
    //         frameReady.notify_one();
    //     }
    // });
    // thread_.detach();
    
    
    
    
    
    std::thread thread_([this]
    {
        cv::VideoCapture inputVideo("dump_2024.04.04_124321.mp4");
        cv::Mat frame;
        
        while (!terminated)
        {
            inputVideo >> frame;
            if (frame.empty()) {
                std::cout << "empty frame" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(32));
            std::lock_guard<std::mutex> lock(m);
            frameQueue.push(frame.clone());
            if (frameQueue.size() > 5)
                frameQueue.pop();
            frameReady.notify_one();
            
        }
    });
    thread_.detach();
}

bool CameraCalibrator::init()
{
    if (_inputSource == "socket")
    {
        is_inited = _init_socket();
    }
    if (_inputSource == "stream")
    {
        _init_stream();
        is_inited = true;
    }
    return is_inited;
}

void CameraCalibrator::show()
{
    if (!is_inited)
    {
        std::cout << "Can't receive images without connection" << std::endl;
        return;
    }
    int key;
    while (key != 'q')
    {
        auto img = _nextFrame();
        cv::imshow("new", img);
        key = cv::waitKey(10) >= 0;
        if (key)
            break;
    }
}

void CameraCalibrator::_saveTransform(const cv::Mat& perspectiveTransformMatrix)
{
    cv::FileStorage calibrationConfig(transformPath, cv::FileStorage::WRITE);
    if (calibrationConfig.isOpened())
    {
        calibrationConfig << "perspectiveTransformMatrix" << perspectiveTransformMatrix;
        calibrationConfig.release();
        std::cout << "Perspective transform saved" << std::endl;
    }
    else
    {
        std::cerr << "Perspective transform saving failed. File: " <<  transformPath << " IO Error\n";
    }
}

cv::Mat CameraCalibrator::_readTransform()
{
    cv::FileStorage calibrationConfig(transformPath, cv::FileStorage::READ);
    if (calibrationConfig.isOpened())
    {
        cv::Mat perspectiveTransform;
        calibrationConfig["perspectiveTransformMatrix"] >> perspectiveTransform;
        std::cout << "Perspective transform matrix loaded" << std::endl;

        calibrationConfig.release();
        return perspectiveTransform;
    }
    return {};
}



