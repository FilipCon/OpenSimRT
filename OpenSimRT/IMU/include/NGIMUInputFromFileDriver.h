#pragma once
#include "InputDriver.h"
#include "NGIMUData.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include <condition_variable>
#include <thread>

/**
 * @brief xio NGIMU Input driver for streaming data from file.
 */

namespace OpenSimRT {

class IMU_API NGIMUInputFromFileDriver : public InputDriver<NGIMUData> {
    typedef std::pair<double, SimTK::Vector> IMUDataFrameAsVector;

 public:
    NGIMUInputFromFileDriver(const std::string& fileName, const double& rate);
    ~NGIMUInputFromFileDriver();
    virtual void startListening() override;
    void stopListening() override {}

    virtual std::vector<NGIMUData> getFrame() override;
    IMUDataFrameAsVector getFrameAsVector();
    std::vector<NGIMUData> fromVector(const SimTK::Vector&);

    OpenSim::TimeSeriesTable initializeLogger();

 private:
    OpenSim::TimeSeriesTable table;
    std::mutex _mutex;
    std::condition_variable _cond;
    SimTK::RowVector _frame;
    double _time;
    bool _newRow = false;
    int i;
    double _rate;
    std::thread t;
};
} // namespace OpenSimRT
