#include "NGIMUInputFromFileDriver.h"

#include "Exception.h"
#include "NGIMUData.h"

#include <Common/TimeSeriesTable.h>
#include <exception>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>

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

NGIMUInputFromFileDriver::NGIMUInputFromFileDriver(const std::string& fileName,
                                                   const double& rate)
        : table(fileName), _rate(rate), i(0) {}
NGIMUInputFromFileDriver::~NGIMUInputFromFileDriver() { t.join(); }
void NGIMUInputFromFileDriver::startListening() {
    static auto f = [&]() {
        try {
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _time = table.getIndependentColumn()[i];
                    _frame = table.getMatrix()[i];
                    _newRow = true;
                }
                _cond.notify_one();

                // increase counter
                if (i < table.getNumRows())
                    ++i;
                else
                    THROW_EXCEPTION("End of file.");

                std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(1 / _rate * 1000)));
            }
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            exc_ptr = std::current_exception();
        }
    };
    t = std::thread(f);
}

std::vector<NGIMUData> NGIMUInputFromFileDriver::getFrame() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cond.wait(lock, [&]() { return _newRow == true; });
    _newRow = false;
    return std::move(fromVector(_frame.getAsVector()));
}

NGIMUInputFromFileDriver::IMUDataFrameAsVector
NGIMUInputFromFileDriver::getFrameAsVector() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cond.wait(lock, [&]() { return _newRow == true; });
    _newRow = false;
    return std::move(std::make_pair(_time, _frame.getAsVector()));
}

OpenSim::TimeSeriesTable NGIMUInputFromFileDriver::initializeLogger() {
    OpenSim::TimeSeriesTable q;
    q.updTableMetaData() = table.updTableMetaData();
    q.setColumnLabels(table.getColumnLabels());

    // return table
    return q;
}
