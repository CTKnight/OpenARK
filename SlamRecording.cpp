

#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include <nanoflann.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenARK Libraries
#include "Version.h"
#include "D435iCamera.h"
#include "Util.h"

#include "Core.h"
#include "Visualizer.h"

using namespace ark;

std::string getTimeTag() {
    std::ostringstream oss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}

int main(int argc, char ** argv) {
    printf("Welcome to OpenARK v %s Data Recording Tool\n\n", VERSION);
	printf("CONTROLS:\nQ or ESC to stop recording and begin writing dataset to disk,\nSPACE to start/pause"
           "(warning: if pausing in the middle, may mess up timestamps)\n\n");

    using boost::filesystem::path;
	const path directory_path = 
        argc > 1 ? argv[1] : std::string("./data_path") + getTimeTag(); // modify this
	path depth_path = directory_path / "depth_exr/";
	path rgb_path = directory_path / "rgb/";
	path timestamp_path = directory_path / "timestamp.txt";
	path intrin_path = directory_path / "intrin.txt";
	path imu_path = directory_path / "imu.txt";

    std::vector<MultiCameraFrame> frameList;
    std::vector<ImuPair> imuList;

    D435iCamera camera;
    camera.start();

    std::vector<ImuPair> imuBuffer;
    while (true) {

        MultiCameraFrame frame;
        camera.update(frame);
        frameList.emplace_back(frame);

        imuBuffer.clear();
        camera.getImuToTime(frame.timestamp_,imuBuffer);
        for (const auto &imuPair: imuBuffer) {
            imuList.emplace_back(imuPair);
        }
        // visualize results
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
    }

    return 0;
}
