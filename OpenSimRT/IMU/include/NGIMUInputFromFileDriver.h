#pragma once
#include "InputDriver.h"
#include "NGIMUData.h"

/**
 * @brief xio NGIMU Input driver for streaming data from file.
 */

namespace OpenSimRT {

class IMU_API NGIMUInputFromFileDriver : public InputDriver<NGIMUData> {
    typedef std::pair<double, std::vector<NGIMUData>> IMUDataFrame;

 public:
    NGIMUInputFromFileDriver(const std::string& fileName);

    virtual void startListening() override;
    virtual void stopListening() override;
    virtual IMUDataFrame getFrame() override;

 private:
    OpenSim::TimeSeriesTable table;
    int i;
};
} // namespace OpenSimRT
