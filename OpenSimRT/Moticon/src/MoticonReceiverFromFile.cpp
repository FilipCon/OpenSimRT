#include "MoticonReceiverFromFile.h"

#include "Exception.h"
#include "MoticonData.h"

#include <SimTKcommon/internal/BigMatrix.h>

using namespace OpenSimRT;
MoticonReceiverFromFile::MoticonReceiverFromFile(const std::string& fileName)
        : table(fileName), i(0) {}

std::pair<double, MoticonReceivedBundle> MoticonReceiverFromFile::getFrame() {
    const auto time = table.getIndependentColumn()[i];
    const auto row = table.getMatrix()[i].getAsVector();

    // increase counter
    ++i;

    MoticonReceivedBundle data;
    data.fromVector(row);
    return std::make_pair(time, data);
}

std::pair<double, SimTK::RowVectorView>
MoticonReceiverFromFile::getFrameAsVector() {
    const auto time = table.getIndependentColumn()[i];
    const auto row = table.getMatrix()[i].getAsRowVectorView();

    // increase counter
    if (i < table.getNumRows())
        ++i;
    else
        i = 0;
    return std::move(std::make_pair(time, row));
}
