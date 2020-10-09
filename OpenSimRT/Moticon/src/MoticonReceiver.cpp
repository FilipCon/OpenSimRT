#include "MoticonReceiver.h"

#include "Exception.h"
#include "Utils.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <Simulation/Model/Model.h>
#include <cstring>
#include <map>
#include <string>
#include <utility>

using namespace SimTK;
using namespace OpenSimRT;
using namespace std;

#define BUFFER_SIZE 4096 // TODO for large input values might need larger value

typedef MoticonInsoleSizes::Size (*SizeFun)();
SizeFun insoleSizeSelector(const int& size) {
    if (size == 4)
        return &MoticonInsoleSizes::size4;
    else
        THROW_EXCEPTION("Input Size is invalid");
}

void MoticonReceiver::setInsoleSize(const int& x) {
    size = insoleSizeSelector(x)();
}

std::ostream& operator<<(std::ostream& os,
                         const MoticonReceiver::MoticonReceivedBundle& m) {
    os << "Time: " << m.timestamp << " "
       << "L.Acceleration: " << m.left.acceleration << " "
       << "L.AngularRate: " << m.left.angularRate << " "
       << "L.CoP: " << m.left.cop << " "
       << "L.Pressure: " << m.left.pressure << " "
       << "L.TotalForce: " << m.left.totalForce << " "
       << "R.Acceleration: " << m.right.acceleration << " "
       << "R.AngularRate: " << m.right.angularRate << " "
       << "R.CoP: " << m.right.cop << " "
       << "R.Pressure: " << m.right.pressure << " "
       << "R.TotalForce: " << m.right.totalForce;
    return os;
}

Vector MoticonReceiver::MoticonReceivedBundle::asVector() const {
    double v[this->size()];
    int i = 0;
    vecToDouble(left.acceleration, v + i);
    i += 3;
    vecToDouble(left.angularRate, v + i);
    i += 3;
    vecToDouble(left.cop, v + i);
    i += 2;
    vecToDouble(left.pressure, v + i);
    i += 16;
    std::memcpy(v + i, &left.totalForce, sizeof(double));
    i += 1;
    vecToDouble(right.acceleration, v + i);
    i += 3;
    vecToDouble(right.angularRate, v + i);
    i += 3;
    vecToDouble(right.cop, v + i);
    i += 2;
    vecToDouble(right.pressure, v + i);
    i += 16;
    std::memcpy(v + i, &right.totalForce, sizeof(double));
    i += 1;
    return std::move(SimTK::Vector_<double>(this->size(), v, true));
}

MoticonReceiver::MoticonReceiver(const string& ip, const int& port) {
    setup(ip, port);
}
MoticonReceiver::~MoticonReceiver() {
    delete endPoint;
    delete socket;
}
void MoticonReceiver::setup(const string& ip, const int& port) {
    endPoint = new IpEndpointName(ip.c_str(), port);
    socket = new UdpReceiveSocket(*endPoint);
}

string MoticonReceiver::getStream() {
    // if (!socket->IsBound()) THROW_EXCEPTION("Moticon: Socket is not bound");
    char buffer[BUFFER_SIZE];
    socket->ReceiveFrom(*endPoint, buffer, sizeof(buffer));
    return string(buffer);
}

vector<double> MoticonReceiver::splitInputStream(string str, string delimiter) {
    vector<double> result;
    string token;
    size_t pos = 0;
    str += delimiter; // to get the last token
    while ((pos = str.find(delimiter)) != string::npos) {
        token = str.substr(0, pos);
        const char* value = token.c_str();
        char* end;
        result.push_back(strtod(value, &end));
        str.erase(0, pos + delimiter.length());
    }
    return result;
}

OpenSim::TimeSeriesTable MoticonReceiver::initializeLogger() {
    vector<string> columnNames = {
            "L.Acceleration_x", "L.Acceleration_y", "L.Acceleration_z",
            "L.AngularRate_x",  "L.AngularRate_y",  "L.AngularRate_z",
            "L.CoP_x",          "L.CoP_y",          "L.Pressure_1",
            "L.Pressure_2",     "L.Pressure_3",     "L.Pressure_4",
            "L.Pressure_5",     "L.Pressure_6",     "L.Pressure_7",
            "L.Pressure_8",     "L.Pressure_9",     "L.Pressure_10",
            "L.Pressure_11",    "L.Pressure_12",    "L.Pressure_13",
            "L.Pressure_14",    "L.Pressure_15",    "L.Pressure_16",
            "L.TotalForce",     "R.Acceleration_x", "R.Acceleration_y",
            "R.Acceleration_z", "R.AngularRate_x",  "R.AngularRate_y",
            "R.AngularRate_z",  "R.CoP_x",          "R.CoP_y",
            "R.Pressure_1",     "R.Pressure_2",     "R.Pressure_3",
            "R.Pressure_4",     "R.Pressure_5",     "R.Pressure_6",
            "R.Pressure_7",     "R.Pressure_8",     "R.Pressure_9",
            "R.Pressure_10",    "R.Pressure_11",    "R.Pressure_12",
            "R.Pressure_13",    "R.Pressure_14",    "R.Pressure_15",
            "R.Pressure_16",    "R.TotalForce"};

    OpenSim::TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}

MoticonReceiver::MoticonReceivedBundle MoticonReceiver::receiveData() {
    MoticonReceivedBundle data;
    auto vec = splitInputStream(getStream(), " ");
    double* arr = &vec[0];

    // time
    data.timestamp = *(arr + 0);

    // left
    data.left.acceleration = Vec3(arr + 1); // in g
    data.left.angularRate = Vec3(arr + 4);  // in degrees per second (dps)
    data.left.cop = Vec2(arr + 7).elementwiseMultiply(
            Vec2(size.length,
                 size.width)); // cop measurements are in range [-0.5, 0.5]
                               // related to length/width
    data.left.pressure = Vec<16>(arr + 9); // in N/cm^2
    data.left.totalForce = *(arr + 25);    // in N

    // right
    data.right.acceleration = Vec3(arr + 26);
    data.right.angularRate = Vec3(arr + 29);
    data.right.cop =
            Vec2(arr + 32).elementwiseMultiply(Vec2(size.length, size.width));
    data.right.pressure = Vec<16>(arr + 34);
    data.right.totalForce = *(arr + 50);
    return data;
}
