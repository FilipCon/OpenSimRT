#include "MoticonReceiver.h"

using namespace SimTK;
using namespace std;

#define LOCAL_PORT 8888
#define BUFFER_SIZE 4096 // TODO for large input values might need larger value

MoticonReceiver::MoticonReceiver() { socket.Bind(LOCAL_PORT); }

string MoticonReceiver::getStream() {
    char buffer[BUFFER_SIZE];
    sockaddr_in add = socket.RecvFrom(buffer, sizeof(buffer));
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