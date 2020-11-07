#pragma once
#include "MoticonData.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <condition_variable>
#include <string>

namespace OpenSimRT {

class Moticon_API MoticonReceiverFromFile {
 public:
 MoticonReceiverFromFile(const std::string& fileName, const double& rate);
 ~MoticonReceiverFromFile();
    std::pair<double, MoticonReceivedBundle> getFrame();
    std::pair<double, SimTK::Vector> getFrameAsVector();

    void startListening();
    OpenSim::TimeSeriesTable initializeLogger();

 private:
    OpenSim::TimeSeriesTable table;
    int i;
    std::mutex _mutex;
    std::condition_variable _cond;
    SimTK::RowVector _frame;
    double _time;
    double _rate;
    bool _newRow = false;
    std::thread t;
};
} // namespace OpenSimRT
