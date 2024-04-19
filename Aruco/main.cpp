#include "CameraCalibrator.h"

const char* keys =
        {
                "{help h usage ?  |                      | print this message                   }"
                "{input           |stream                | input for frames (stream or socker)  }"
                "{calibrationPath |CameraCalibration.yaml| path for CameraCalibration.yaml file }"
                "{device          |0                     | video device for input stream        }"
                "{action          |7                     | \n0 - save video, \n"
                "1 - save dataset of images to tiff file, \n"
                "2 - create charuco board image, \n"
                "3 - create aruco markers image, \n"
                "4 - calibrate camera using charuco board, \n"
                "5 - calibrate camera using chess board, \n"
                "6 - check charuco board axes, \n"
                "7 - show image to the screen                                                   }"
        };

int main(int argc, const char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    std::string inputSource = "stream";
    std::string calibrationPath = "/home/tsokurenkosv/camera_calibration/Aruco/print/CameraCalibration.yaml";
    int device = 0;
    const char* serverIP = "192.168.1.203";
    int serverPort = 4097;

    if (parser.has("input"))
        inputSource = parser.get<std::string>("input");
    if (parser.has("calibrationPath"))
        calibrationPath = parser.get<std::string>("calibrationPath");
    // if (parser.has("device"))
        // device = parser.get<int>("device");
    // int action = parser.get<int>("action");
    int action = 6;

    CameraCalibrator camera_calibrator(serverIP, serverPort, calibrationPath, inputSource, device);
    switch (action)
//    {
//        case 0:
//        {
//            camera_calibrator.init();
//            camera_calibrator.save_images("chessboardImages.avi", true, 0);
//            break;
//        }
//        case 1:
//        {
//            camera_calibrator.init();
//            camera_calibrator.save_images("chessboardImages.tiff", false, 3);
//            break;
//        }
//        case 2:
//        {
//            camera_calibrator.create_board();
//            break;
//        }
//        case 3:
//        {
//            camera_calibrator.create_markers({0,1,2,3});
//            break;
//        }
//        case 4:
//        {
//            camera_calibrator.calibrate_with_charuco_board("chessboardImages.tiff");
//            break;
//        }
//        case 5:
//        {
//   camera_calibrator.calibrate("/home/tsokurenkosv/camera_calibration/Aruco/1.tiff", 4, 6);
//            break;
//        }
        case 6:
        {
            camera_calibrator.init();
            camera_calibrator.distance_calibrate(0.4);
            break;
        }
//        case 7:
//        {
//            camera_calibrator.init();
//            camera_calibrator.show();
//            break;
//        }
//        default:
//            parser.printMessage();
//            break;
//    }
    return 0;
}
