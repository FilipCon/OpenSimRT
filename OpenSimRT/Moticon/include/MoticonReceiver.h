#pragma once
#include "MoticonData.h"
#include "internal/MoticonExports.h"
#include "ip/IpEndpointName.h"
#include "ip/UdpSocket.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace OpenSimRT {

class Moticon_API MoticonReceiver {
 public:
    MoticonReceiver(const std::string& ip, const int& port);
    ~MoticonReceiver();
    void setup(const std::string& ip, const int& port);
    MoticonReceivedBundle receiveData();
    OpenSim::TimeSeriesTable initializeLogger();
    void setInsoleSize(const int&);

 private:
    UdpReceiveSocket* socket;
    IpEndpointName* endPoint;
    std::string getStream();
    std::vector<double> splitInputStream(std::string, std::string);
    double initFrameTime;
    MoticonInsoleSizes::Size size;
};
} // namespace OpenSimRT
