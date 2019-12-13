#include <iostream>
#include <thread>
#include <atomic>
#include <ctime>
#include <csignal>
#include <exception>
#include "concurrency.h"
#include "MockD435iCamera.h"
#include "CommonSlam.h"

using namespace ark;
using boost::filesystem::path;

void signal_handler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGILL, signal_handler);
    std::signal(SIGABRT, signal_handler);
    std::signal(SIGSEGV, signal_handler);
    std::signal(SIGTERM, signal_handler);
    if (argc > 5)
    {
        std::cerr << "Usage: ./" << argv[0] << " [configuration-yaml-file] [vocabulary-file] [skip-first-seconds] [data_path]" << std::endl
                  << "Args given: " << argc << std::endl;
        return -1;
    }

    google::InitGoogleLogging(argv[0]);

    // read configuration file
    std::string configFilename;
    if (argc > 1)
        configFilename = argv[1];
    else
        configFilename = util::resolveRootPath("config/d435i_intr.yaml");

    std::string vocabFilename;
    if (argc > 2)
        vocabFilename = argv[2];
    else
        vocabFilename = util::resolveRootPath("config/brisk_vocab.bn");

    okvis::Duration deltaT(0.0);
    if (argc > 3)
    {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    path dataPath{"./data_path_25-10-2019 16-47-28"};
    if (argc > 4)
    {
        dataPath = path(argv[4]);
    }

    OkvisSLAMSystem slam(vocabFilename, configFilename);
    cv::FileStorage configFile(configFilename, cv::FileStorage::READ);

    //setup display
    if (!MyGUI::Manager::init())
    {
        fprintf(stdout, "Failed to initialize GLFW\n");
        return -1;
    }

    printf("Camera initialization started...\n");
    fflush(stdout);
    MockD435iCamera camera(dataPath);

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);
    commonSlam(slam, camera, configFile, true);
    return 0;
}