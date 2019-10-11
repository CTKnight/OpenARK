#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include "concurrency.h"
#include <atomic>

// OpenARK Libraries
#include "CameraSetup.h"
using boost::filesystem::path;
namespace ark {
    /**
    * Mock camera for replaying data
    */
    class MockD435iCamera : public CameraSetup
    {
    public:

        /**
        * config the input dir
        */
        explicit MockD435iCamera(path dir);

        /**
        * Destructor
        */
        ~MockD435iCamera() override;

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const override;

        /**
         * Get image size
         */
        cv::Size getImageSize() const;

        /** 
         * Dummy method
         */
        void start() override;
        /**
        * get a frame per time
        */
        void update(MultiCameraFrame & frame) override;

        bool getImuToTime(double timestamp, std::vector<ImuPair>& data_out);

        std::vector<ImuPair> getAllImu();

    protected:
    
        path dataDir;
        path imuTxtPath;
        path timestampTxtPath;
        path depthDir;
        path rgbDir;
        path infraredDir;
        path infrared2Dir;
        int width, height;

    };
}
