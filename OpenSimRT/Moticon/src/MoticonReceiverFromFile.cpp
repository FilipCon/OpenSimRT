#include "MoticonReceiverFromFile.h"

#include "Exception.h"
#include "MoticonData.h"

#include <SimTKcommon/internal/BigMatrix.h>

using namespace OpenSimRT;
MoticonReceiverFromFile::MoticonReceiverFromFile(const std::string& fileName,
                                                 const double& rate)
        : table(fileName), _rate(rate), i(0) {}
MoticonReceiverFromFile::~MoticonReceiverFromFile() { t.join(); }

void MoticonReceiverFromFile::startListening() {
    static auto f = [&]() {
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
    };
    t = std::thread(f);
}

std::pair<double, MoticonReceivedBundle> MoticonReceiverFromFile::getFrame() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cond.wait(lock, [&]() { return _newRow == true; });
    _newRow = false;
    MoticonReceivedBundle bundle;
    bundle.fromVector(_frame.getAsVector());
    return std::move(std::make_pair(_time, bundle));
}

std::pair<double, SimTK::Vector> MoticonReceiverFromFile::getFrameAsVector() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cond.wait(lock, [&]() { return _newRow == true; });
    _newRow = false;
    return std::move(std::make_pair(_time, _frame.getAsVector()));
}

OpenSim::TimeSeriesTable MoticonReceiverFromFile::initializeLogger() {
    OpenSim::TimeSeriesTable q;
    q.updTableMetaData() = table.updTableMetaData();
    q.setColumnLabels(table.getColumnLabels());

    // return table
    return q;
}
