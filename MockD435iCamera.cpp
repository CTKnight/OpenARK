#include "stdafx.h"
#include "Version.h"
#include "MockD435iCamera.h"
#include "Visualizer.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark
{
MockD435iCamera::MockD435iCamera(path dir) : 
dataDir(dir), imuTxtPath(dir / "imu.txt"), timestampTxtPath(dir / "timestamp.txt"), depthDir(dir / "depth/"), 
rgbDir(dir / "rgb/"), infraredDir(dir / "infrared/"), infrared2Dir(dir / "infrared2/"), firstFrameId(-1)
{

}

MockD435iCamera::~MockD435iCamera()
{
    imuStream.close();
    timestampStream.close();
}

void MockD435iCamera::start()
{
     imuStream = ifstream(imuTxtPath.string());
     timestampStream = ifstream(timestampTxtPath.string());
}

bool MockD435iCamera::getImuToTime(double timestamp, std::vector<ImuPair> &data_out)
{
    std::string line1;
    std::string line2;
    std::string line3;

    double ts = 0;
    double gyro0, gyro1, gyro2, accel0, accel1, accel2;
    for (;!imuStream.eof() && ts < timestamp;) {
        // TODO: refactor
        if (!std::getline(imuStream, line1)) {
            std::cout << "Unable to read form data or data end reached\n";
            return false;
        }
        if (!std::getline(imuStream, line2)) {
            std::cout << "Unable to read form data or data end reached\n";
            return false;
        }
        if (!std::getline(imuStream, line3)) {
            std::cout << "Unable to read form data or data end reached\n";
            return false;
        }
        string placeholder;
        std::stringstream ss1(line1);
        std::stringstream ss2(line2);
        std::stringstream ss3(line3);
        ss1 >> placeholder >> ts;
        ss2 >> placeholder >> gyro0 >> gyro1 >> gyro2;
        ss3 >> placeholder >> accel0 >> accel1 >> accel2;
        ImuPair imu_out { ts, //convert to nanoseconds, for some reason gyro timestamp is in centiseconds
                    Eigen::Vector3d(gyro0, gyro1, gyro2),
                    Eigen::Vector3d(accel0, accel1, accel2)};
        data_out.emplace_back(imu_out);
    }
    return true;
};

const std::string MockD435iCamera::getModelName() const
{
    return "Mock";
}

cv::Size MockD435iCamera::getImageSize() const
{
    return cv::Size(width, height);
}
cv::Mat MockD435iCamera::loadImg(path filename)
{
    return imread(filename.string(), cv::IMREAD_COLOR);
}

void MockD435iCamera::update(MultiCameraFrame &frame)
{
    std::string line;
    if (!std::getline(timestampStream, line)) {
        std::cout << "Unable to read form data or data end reached\n";
        return;
    }
    auto ss = std::stringstream(line);
    int frameId;
    double timestamp;
    ss >> frameId >> timestamp;
    if (firstFrameId < 0) {
        firstFrameId = firstFrameId;
    }
    frame.frameId_ = frameId;
    frame.timestamp_ = timestamp;

    // reading img
    int count = frameId - firstFrameId;
    std::stringstream fileNamess;
    // TODO: extract the naming function
    fileNamess << std::setw(5) << std::setfill('0') << std::to_string(count) << ".jpg";
    std::string fileName = fileNamess.str();

    std::vector<path> pathList {infraredDir, infrared2Dir, depthDir, rgbDir};
    frame.images_.resize(pathList.size());

    for (auto i = 0; i < pathList.size(); i++) {
        frame.images_[i] = loadImg(pathList[i] / fileName);
    }
    // TODO: should we mock the block time as well?
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    printf("frame %d\n", frameId);      
}

} // namespace ark
