#pragma once
#include "Types.h"

namespace ark{

    class CameraSetup{
    public:
        virtual ~CameraSetup(){};

        virtual const std::string getModelName() const
        {
            return "CameraSetup";
        }

        virtual void start() =0;

        virtual void update(MultiCameraFrame & frame) =0;

    }; //CameraSetup
    class ImuCamera: public CameraSetup {
    public:
        virtual bool getImuToTime(double timestamp, std::vector<ImuPair>& data_out) = 0;
    };
} //ark
