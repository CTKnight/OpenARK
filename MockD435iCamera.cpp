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
MockD435iCamera::MockD435iCamera(path dir) : dataDir(dir), imuTxtPath(dir / "imu.txt"), timestampTxtPath(dir / "timestamp.txt"), depthDir(dir / "depth/"), rgbDir(dir / "rgb/"), infraredDir(dir / "infrared/"), infrared2Dir(dir / "infrared2/")
{
}

MockD435iCamera::~MockD435iCamera()
{
}

void MockD435iCamera::start()
{
}

bool MockD435iCamera::getImuToTime(double timestamp, std::vector<ImuPair> &data_out)
{
    
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

void MockD435iCamera::update(MultiCameraFrame &frame)
{

}

} // namespace ark
