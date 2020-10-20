#include "NGIMUInputFromFileDriver.h"

#include "Exception.h"

#include <Common/TimeSeriesTable.h>
#include <iostream>
#include <iterator>

using namespace OpenSimRT;

std::vector<NGIMUData>
NGIMUInputFromFileDriver::fromVector(const SimTK::Vector& v) {
    std::vector<NGIMUData> frame;
    for (size_t i = 0; i < v.size(); i += NGIMUData::size()) {
        NGIMUData data;
        data.fromVector(v(i, NGIMUData::size()));
        frame.push_back(data);
    }
    return frame;
}

NGIMUInputFromFileDriver::NGIMUInputFromFileDriver(const std::string& fileName)
        : table(fileName), i(0) {}

NGIMUInputFromFileDriver::IMUDataFrame NGIMUInputFromFileDriver::getFrame() {
    const auto time = table.getIndependentColumn()[i];
    const auto row = table.getMatrix()[i].getAsVector();

    // increase counter
    if (i < table.getNumRows())
        ++i;
    else
        i = 0;
    return std::move(std::make_pair(time, fromVector(row)));
}

NGIMUInputFromFileDriver::IMUDataFrameAsVector
NGIMUInputFromFileDriver::getFrameAsVector() {
    const auto time = table.getIndependentColumn()[i];
    const auto row = table.getMatrix()[i].getAsRowVectorView();

    // increase counter
    if (i < table.getNumRows())
        ++i;
    else
        i = 0;
    return std::move(std::make_pair(time, row));
}

void NGIMUInputFromFileDriver::startListening() {}
void NGIMUInputFromFileDriver::stopListening() {}
