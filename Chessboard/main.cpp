#include <iostream>
#include "CameraCalibrator.hpp"

int main(int argc, char** argv) {

    //--------------------------------------------------------
    //networking stuff: socket , connect
    //--------------------------------------------------------

    const char* serverIP = "192.168.1.203";
    int serverPort = 4097;

    auto camera_calibrator = CameraCalibrator(serverIP, serverPort, "/home/egor/CalibrationConfig.yaml", "Calibration.yaml");
    camera_calibrator.init();
    camera_calibrator.calibrate();

    return 0;
}
