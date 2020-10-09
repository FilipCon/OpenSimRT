#include "NGIMUInputFromFileDriver.h"

#include <Common/TimeSeriesTable.h>

using namespace OpenSimRT;

std::vector<NGIMUData> imuDataFrameFromVector(const SimTK::Vector& v) {
    std::vector<NGIMUData> frame;
    for (int i = 0; i < v.size(); i += NGIMUData::size()) {
        NGIMUData data;
        data.fromVector(v(i, NGIMUData::size()));
        frame.push_back(data);
    }
    return frame;
}

NGIMUInputFromFileDriver::NGIMUInputFromFileDriver(const std::string& fileName)
    : table(fileName) , i(0){
}

NGIMUInputFromFileDriver::IMUDataFrame NGIMUInputFromFileDriver::getFrame() {
    const auto time = table.getIndependentColumn()[i];
    const auto row = table.getMatrix()[i].getAsVector();

    // dummy delay to simulate real time
    std::this_thread::sleep_for(std::chrono::milliseconds(13));
    ++i;

    return std::make_pair(time, imuDataFrameFromVector(row));
}

void NGIMUInputFromFileDriver::startListening() {}
void NGIMUInputFromFileDriver::stopListening() {}
