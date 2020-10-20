#pragma once
#include "MoticonData.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <string>

namespace OpenSimRT {

class Moticon_API MoticonReceiverFromFile {
 public:
    MoticonReceiverFromFile(const std::string& fileName);

    std::pair<double, MoticonReceivedBundle> getFrame();
    std::pair<double, SimTK::RowVectorView> getFrameAsVector();
 private:
    OpenSim::TimeSeriesTable table;
    int i;
};
} // namespace OpenSimRT
