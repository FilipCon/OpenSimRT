#pragma once
#include "InputDriver.h"
#include "NGIMUData.h"

/**
 * @brief xio NGIMU Input driver for streaming data from file.
 */

namespace OpenSimRT {

class IMU_API NGIMUInputFromFileDriver : public InputDriver<NGIMUData> {
    typedef std::pair<double, SimTK::RowVectorView> IMUDataFrameAsVector;

 public:
    typedef std::pair<double, std::vector<NGIMUData>> IMUDataFrame;
    NGIMUInputFromFileDriver(const std::string& fileName);

    virtual void startListening() override;
    virtual void stopListening() override;
    virtual IMUDataFrame getFrame() override;
    IMUDataFrameAsVector getFrameAsVector();
    std::vector<NGIMUData> fromVector(const SimTK::Vector&);

 private:
    OpenSim::TimeSeriesTable table;
    int i;
};
} // namespace OpenSimRT
