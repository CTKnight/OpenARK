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
using boost::filesystem::path;

std::string getTimeTag()
{
    std::ostringstream oss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}

void saveImg(int id, const cv::Mat &img, const path imgDir)
{
    std::stringstream fileName;
    fileName << std::setw(4) << std::setfill('0') << std::to_string(id) << ".jpg";
    const std::string dst = (imgDir / fileName.str()).string();
    cout << "Writing " << dst << endl;
    cv::imwrite(dst, img);
}

int main(int argc, char **argv)
{
    printf("Welcome to OpenARK v %s Slam Recording Tool\n\n", VERSION);
    printf("CONTROLS:\nQ or ESC to stop recording and begin writing dataset to disk,\nSPACE to start/pause"
           "(warning: if pausing in the middle, may mess up timestamps)\n\n");

    const path directory_path =
        argc > 1 ? argv[1] : std::string("./data_path_") + getTimeTag(); // modify this
    path depth_path = directory_path / "depth/";
    path infrared_path = directory_path / "infrared/";
    path infrared2_path = directory_path / "infrared2/";
    path rgb_path = directory_path / "rgb/";
    path timestamp_path = directory_path / "timestamp.txt";
    path intrin_path = directory_path / "intrin.txt";
    path imu_path = directory_path / "imu.txt";
    std::vector<path> pathList{directory_path, depth_path, infrared_path, infrared2_path, rgb_path};
    for (const auto &p : pathList)
    {
        if (!boost::filesystem::exists(p))
        {
            boost::filesystem::create_directories(p);
        }
    }

    std::ofstream imu_ofs(imu_path.string());

    std::vector<MultiCameraFrame> frameList;
    std::vector<ImuPair> imuList;

    D435iCamera camera;
    camera.start();

    std::vector<ImuPair> imuBuffer;
    bool paused = true;
    int frameNum = 0;
    single_consumer_queue<std::shared_ptr<MultiCameraFrame> > img_queue;
    while (true)
    {
        // 0: infrared
        // 1: infrared2
        // 2: depth
        // 3: rgb
        auto frame = std::make_shared<MultiCameraFrame>();
        camera.update(*frame);
        // frameList.emplace_back(frame);

        imuBuffer.clear();
        camera.getImuToTime(frame->timestamp_, imuBuffer);

        // for (const auto &imuPair: imuBuffer) {
        //     imuList.emplace_back(imuPair);
        // }

        if (paused)
        {
            const std::string NO_SIGNAL_STR = "PAUSED";
            const cv::Scalar RECT_COLOR = cv::Scalar(0, 160, 255);
            const int RECT_WID = 120, RECT_HI = 40;

            for (auto &img : frame->images_)
            {
                const cv::Point STR_POS(img.cols / 2 - 50, img.rows / 2 + 7);
                cv::Rect rect(img.cols / 2 - RECT_WID / 2,
                              img.rows / 2 - RECT_HI / 2,
                              RECT_WID, RECT_HI);
                cv::rectangle(img, rect, RECT_COLOR, -1);
                cv::putText(img, NO_SIGNAL_STR, STR_POS, 0, 0.8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }

        const auto frameId = frame->frameId_;
        const auto &infrared = frame->images_[0];
        const auto &infrared2 = frame->images_[1];
        const auto &depth = frame->images_[2];
        const auto &rgb = frame->images_[3];
        cv::imshow(camera.getModelName() + " RGB", rgb);
        cv::imshow(camera.getModelName() + " Infrared", infrared);
        cv::imshow(camera.getModelName() + " Depth", depth);
        if (!paused)
        {
            const auto frameId = frame->frameId_;
            saveImg(frameNum, infrared, infrared_path);
            saveImg(frameNum, infrared2, infrared2_path);
            saveImg(frameNum, depth, depth_path);
            saveImg(frameNum, rgb, rgb_path);

            frameNum++;
        }
        // visualize results
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27)
        {
            // 27 is ESC
            break;
        }
        else if (k == ' ')
        {
            paused = !paused;
        }
    }
    return 0;
}
