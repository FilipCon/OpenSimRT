#include "SyncManager.h"
#include "Utils.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Array.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/NTraits.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/VectorMath.h>
#include <SimTKcommon/internal/negator.h>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

using namespace std;
using namespace OpenSimRT;

void run() {
    SyncManager manager(1000.0, 0.01);

    manager.appendPack(
            // pair
            std::pair<double, SimTK::Vec4>(0, SimTK::Quaternion().asVec4()),
            std::pair<double, SimTK::Vec4>(0, SimTK::Quaternion().asVec4()),

            // container of pairs
            std::vector<std::pair<double, SimTK::Vec4>>{
                    {0, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()},
                    {0, SimTK::Quaternion(1, 0, 0, 0).asVec4()}},

            // pair with larger time
            std::pair<double, SimTK::Vec4>(
                    0, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()),

            // pair with container of vecs
            std::pair<double, std::vector<SimTK::Vec3>>(
                    0,
                    {SimTK::Vec3(0.3, 0.3, 0.3), SimTK::Vec3(0.2, 0.2, 0.2)}));

    manager.getPack();

    manager.appendPack(

            // pair
            std::pair<double, SimTK::Vec4>(
                    0.4, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()),
            std::pair<double, SimTK::Vec4>(0.4, SimTK::Quaternion().asVec4()),

            // container of pairs
            std::vector<std::pair<double, SimTK::Vec4>>{
                    {0.4, SimTK::Quaternion().asVec4()},
                    {0.4, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()}},

            // pair with larger time
            std::pair<double, SimTK::Vec4>(0.4, SimTK::Quaternion().asVec4()),

            // pair with container of vecs
            std::pair<double, std::vector<SimTK::Vec3>>(
                    0.4, {SimTK::Vec3(0), SimTK::Vec3(0)}));

    cout << "Time Column: "
         << SimTK::Array_<double>(manager.getTable().getIndependentColumn())
         << endl;
    cout << "Table data: " << manager.getTable().getMatrix() << endl;

    auto pack = manager.getPack();
    if (!pack.second.empty())
        cout << pack.first << " " << pack.second[0] << endl;
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
