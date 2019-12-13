#pragma once

#include <iostream>
#include <thread>
#include <cmath>
#include "CameraSetup.h"
#include "OkvisSLAMSystem.h"
#include "glfwManager.h"
#include "Util.h"
#include <boost/filesystem.hpp>

using namespace ark;
using boost::filesystem::path;

void commonSlam(
    OkvisSLAMSystem &slam,
    ImuCamera &camera,
    const cv::FileStorage &configFile,
    bool isReplay
);