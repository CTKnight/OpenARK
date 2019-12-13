#include "D435iCamera.h"
#include <iostream>
#include <thread>
#include "Util.h"
#include "CommonSlam.h"

using namespace ark;

int main(int argc, char **argv)
{

    if (argc >4 ) {
        std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]" << std::endl
        <<"Args given: " << argc << std::endl;
        return -1;
    }

    google::InitGoogleLogging(argv[0]);

    okvis::Duration deltaT(0.0);
    if (argc == 4) {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    // read configuration file
    std::string configFilename;
    if (argc > 1) configFilename = argv[1];
    else configFilename = util::resolveRootPath("config/d435i_intr.yaml");

    std::string vocabFilename;
    if (argc > 2) vocabFilename = argv[2];
    else vocabFilename = util::resolveRootPath("config/brisk_vocab.bn");

    OkvisSLAMSystem slam(vocabFilename, configFilename);
    cv::FileStorage configFile(configFilename, cv::FileStorage::READ);

    printf("Camera initialization started...\n");
    fflush(stdout);
    CameraParameter cameraParameter;
    if (configFile["emitterPower"].isReal()) {
        configFile["emitterPower"] >> cameraParameter.emitterPower;
    }
    D435iCamera camera(cameraParameter);

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);

    commonSlam(slam, camera, configFile, false);
    return 0;
}
