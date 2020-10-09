#pragma once
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

struct Moticon_API MoticonInsoleSizes {
    struct Size {
        double length;
        double width;
    };

    // TODO add more sizes
    static Size size4() { return Size{0.2486, 0.089}; }
};

class Moticon_API MoticonReceiver {
 public:

    struct Moticon_API MoticonReceivedBundle {
        double timestamp;
        struct Measurement {
            SimTK::Vec3 acceleration;
            SimTK::Vec3 angularRate;
            SimTK::Vec2 cop;
            SimTK::Vec<16> pressure; // 16 sensors
            double totalForce;
        } left, right;
        static int size() { return 50; };
        SimTK::Vector asVector() const;
    };

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

/**
 * @brief Overwrite ostream operator for MoticonReceivedBundle
 */
std::ostream& operator<<(std::ostream& os,
                         const MoticonReceiver::MoticonReceivedBundle& m);
} // namespace OpenSimRT
