#include "MoticonReceiver.h"

#include "Exception.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>

using namespace SimTK;
using namespace std;

#define BUFFER_SIZE 4096 // TODO for large input values might need larger value

Vector MoticonReceiver::MoticonReceivedBundle::asVector() const {
    SimTK::Vector v(50, 0.0);
    v(0, 3) = Vector(left.acceleration);
    v(3, 3) = Vector(left.angularRate);
    v(6, 2) = Vector(left.cop);
    v(8, 16) = Vector(left.pressure);
    v(24, 1) = Vector(left.totalForce);
    v(25, 3) = Vector(right.acceleration);
    v(28, 3) = Vector(right.angularRate);
    v(31, 2) = Vector(right.cop);
    v(33, 16) = Vector(right.pressure);
    v(49, 1) = Vector(right.totalForce);
    return v;
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
            "L.Acceleration: ", "L.AngularRate: ", "L.CoP: ",
            "L.Pressure: ",     "L.TotalForce: ",  "R.Acceleration: ",
            "R.AngularRate: ",  "R.CoP: ",         "R.Pressure: ",
            "R.TotalForce: "};

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
    data.left.acceleration = Vec3(arr + 1);
    data.left.angularRate = Vec3(arr + 4);
    data.left.cop = Vec2(arr + 7);
    data.left.pressure = Vec<16>(arr + 9);
    data.left.totalForce = *(arr + 25);

    // right
    data.right.acceleration = Vec3(arr + 26);
    data.right.angularRate = Vec3(arr + 29);
    data.right.cop = Vec2(arr + 32);
    data.right.pressure = Vec<16>(arr + 34);
    data.right.totalForce = *(arr + 50);
    return data;
}
