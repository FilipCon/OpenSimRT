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
    std::string getDataFromStream();
    std::vector<double> splitInputStream(std::string, std::string);
    void startStream();

    UdpReceiveSocket* socket;
    IpEndpointName* endPoint;

    double initFrameTime;
    MoticonInsoleSizes::Size size;
    std::string dataStream;

    std::thread streamThread;
    std::mutex _mu;
    std::condition_variable _cond;
    bool newData = false;
};
} // namespace OpenSimRT
