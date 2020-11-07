#pragma once

#include "Exception.h"
#include "TypeHelpers.h"
#include "Utils.h"
#include "internal/CommonExports.h"

#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/internal/Array.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/NTraits.h>
#include <SimTKcommon/internal/VectorMath.h>
#include <SimTKcommon/internal/Vector_.h>
#include <SimTKcommon/internal/negator.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <ostream>
#include <regex>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace OpenSimRT {

template <typename ETX = double> class Common_API SyncManager {
 public:
    SyncManager() = default; // default ctor, in order to have more ctors
    SyncManager(const ETX& samplingRate, const ETX& threshold) {
        _entryThreshold = threshold;
        _samplingRate = samplingRate;
    }
    SyncManager(const SyncManager&) = default;            // copy ctor
    SyncManager(SyncManager&&) = default;                 // move copy ctor
    SyncManager& operator=(const SyncManager&) = default; // assign copy ctor
    SyncManager& operator=(SyncManager&&) = default;      // assign move ctor
    ~SyncManager() = default;                             // dtor

    std::pair<ETX, std::vector<SimTK::Vector_<ETX>>> getPack() {
        size_t delay = 1; // TODO

        const auto& v = _table.getIndependentColumn();
        if (!_isCurrentTimeSet) {
            interpolateNanValues();

            // find the most recent row in matrix with finite values
            auto itr = std::find_if(v.crbegin(), v.crend(), [&](const auto& x) {
                return isVectorFinite(_table.getRow(x).getAsVector());
            });
            if (itr != v.crend()) {
                _currentTime = *itr;
                _isCurrentTimeSet = true;
            }
        } // else _currentTime == -Inf

        auto t = _currentTime;
        auto t1 = t - delay * (1 / _samplingRate);
        auto t2 = t + delay * (1 / _samplingRate);

        // create re-sampled entry @ time 't'
        auto rowNew = createRow(t);
        if (std::find(v.begin(), v.end(), t) == v.end()) {
            _table.appendRow(t, ~rowNew);
            reorderTable();
        } else {
            setRow(t, ~rowNew);
        }

        // initialize output
        size_t columnId = 0;
        std::vector<SimTK::Vector_<ETX>> pack;
        if (isVectorFinite(rowNew) && (t2 <= v.back()) && !SimTK::isInf(t)) {
            // delete all rows from the orifinal table up to the _currentTime
            // entry. We need first to create a copy of the independentColumn
            auto v_cpy(v);
            decltype(v_cpy.rbegin()) it;
            if ((it = std::find_if(v_cpy.rbegin(), v_cpy.rend(),
                                   [&](const auto& x) { return x < t1; })) !=
                v_cpy.rend())
                // NOTE: delete rows in original v based on the copy v_cpy
                deleteRows(v_cpy.begin(), it.base());

            // prepare next sample
            _currentTime += 1 / _samplingRate;

            // create output
            for (size_t i = 0; i < _numOfPacks; ++i) {
                pack.push_back(
                        ~_table.getRow(t)(columnId, _vectorSizePerPack[i]));
                columnId += _vectorSizePerPack[i];
            }
        }
        return std::move(std::make_pair(t, pack));
    }

    template <typename... Args> void appendPack(Args&&... args) {
        // create tuple from args to allow different types
        using ArgsTuple = std::tuple<Args...>;
        ArgsTuple argTuple = {std::forward<Args>(args)...};

        // set the table size from the first appended pack
        if (!_tableSizeIsSet) {
            // apply function for each arg
            map<0, Args...>(argTuple, [&](auto&& p,
                                          const size_t& dummy) { // c++14
                static_assert(
                        is_pair<std::remove_cv_t<
                                std::remove_reference_t<decltype(p)>>>::value,
                        "Only std::pair is allowed");
                typedef std::remove_cv_t<
                        std::remove_reference_t<decltype(p.second)>>
                        pairSecondElement_t;

                // set number of elements in tuple
                _numOfPacks = sizeof...(Args);

                // second is SimTK::Vec
                if constexpr (is_simtk_vec<pairSecondElement_t>::value ||
                              is_simtk_vector<pairSecondElement_t>::value) {
                    _numColumns += p.second.size();
                    _vectorSizePerPack[_packCounter] += p.second.size();

                    // second is Container<SimTK::Vec>
                } else if constexpr (is_container<pairSecondElement_t>::value) {
                    for (auto& v : p.second) {
                        _numColumns += v.size();
                        _vectorSizePerPack[_packCounter] += v.size();
                    }
                } else {
                    throw std::invalid_argument(
                            "Invalid std::pair::second type");
                }
            });

            // create first entry with NaNs
            _table.appendRow(-SimTK::Infinity,
                             std::vector<ETX>(_numColumns, SimTK::NaN));
            _tableSizeIsSet = true;
        }

        // apply function for each arg
        map<0, Args...>(argTuple, [&](auto&& p,
                                      const size_t& columnId) { // c++14
            typedef std::remove_cv_t<
                    std::remove_reference_t<decltype(p.second)>>
                    pairSecondElement_t;
            ETX indRow;
            // append new row or update table rows
            if (isUniqueEntry(p.first, _entryThreshold, indRow)) { // append
                // create new entry with nans
                auto newRow = std::vector<ETX>(_numColumns, SimTK::NaN);

                // std::pair::second is simtk vec
                if constexpr (is_simtk_vec<pairSecondElement_t>::value ||
                              is_simtk_vector<pairSecondElement_t>::value) {
                    for (size_t i = 0; i < p.second.size(); ++i) {
                        newRow[i + columnId] = std::move(p.second[i]);
                    }

                    // std::pair::second is container of simtk vecs
                } else if constexpr (is_container<pairSecondElement_t>::value) {
                    size_t offset = 0;
                    for (const auto& v : p.second) {
                        static_assert(
                                is_simtk_vec<std::remove_cv_t<
                                        std::remove_reference_t<decltype(v)>>>::
                                        value ||
                                is_simtk_vector<std::remove_cv_t<
                                        std::remove_reference_t<decltype(v)>>>::
                                        value);
                        for (size_t i = 0; i < v.size(); ++i) {
                            newRow[i + columnId + offset] = std::move(v[i]);
                        }
                        offset += v.size();
                    }
                } else { // invalid pairs
                    throw std::invalid_argument(
                            "SyncManager: invalid argument");
                }

                // append new row to table
                appendRow(p.first, newRow);

                // after entering a new entry {time, data}, reorder the table
                // rows based on the ordered independentColumn
                reorderTable();

            } else { // update existing row
                // std::pair::second is simtk vec
                if constexpr (is_simtk_vec<pairSecondElement_t>::value ||
                              is_simtk_vector<pairSecondElement_t>::value) {
                    for (int i = 0; i < p.second.size(); ++i) {
                        _table.updRow(indRow).updCol(i + columnId) =
                                std::move(p.second[i]);
                    }
                    // std::pair::second is container of simtk vecs
                } else if constexpr (is_container<pairSecondElement_t>::value) {
                    size_t offset = 0;
                    for (int i = 0; i < p.second.size(); ++i) {
                        auto& v = p.second[i];
                        static_assert(
                                is_simtk_vec<std::remove_cv_t<
                                        std::remove_reference_t<decltype(v)>>>::
                                        value ||
                                is_simtk_vector<std::remove_cv_t<
                                        std::remove_reference_t<decltype(v)>>>::
                                        value);
                        for (int j = 0; j < v.size(); ++j) {
                            _table.updRow(indRow).updCol(
                                    j + columnId + offset) = std::move(v[j]);
                        }
                        offset += v.size();
                    }
                } else { // invalid pairs
                    throw std::invalid_argument(
                            "SyncManager: invalid argument");
                }
            }

            // increase column offset to append the next pair
            _offset += p.second.size();
        });

        // SimTK::Array_<ETX> ar(_table.getIndependentColumn());
        // std::cout << std::setprecision(15) << "Time: " << ar << std::endl;
        // std::cout << "Mat: " << _table.getMatrix() << std::endl;
    }

    // append row to table
    template <typename... Args>
    void appendRow(const ETX& indRow, Args&&... args) {
        _table.appendRow(indRow, std::forward<Args>(args)...);
    }

    // set row in table
    template <typename... Args> void setRow(const ETX& indRow, Args&&... args) {
        _table.setRow(indRow, std::forward<Args>(args)...);
    }

    // set row at inpdex
    template <typename... Args>
    void setRowAtIndex(const size_t& index, Args&&... args) {
        _table.setRowAtIndex(index, std::forward<Args>(args)...);
    }

    // get writable reference to row in table by index
    template <typename... Args>
    void updRowAtIndex(const size_t& index, Args&&... args) {
        _table.updRow(index, std::forward<Args>(args)...);
    }

    // remove row by value in independent column
    void removeRow(const ETX& indRow) { _table.removeRow(indRow); }

    // remove row by index in table
    void removeRowAtIndex(const size_t& i) { _table.removeRowAtIndex(i); }

    // get read only reference to data table
    const OpenSim::DataTable& getTable() { return _table; }

 private:
    // Recursive function for determining if the element in
    // Args... list is an STL-like contairer of std::pairs or an std::pair.
    // Applies given function (f) for every pair in the Args... list (including
    // std::pairs in the STL-like containers)
    template <std::size_t I = 0, typename... Args, typename F>
    inline typename std::enable_if<I == sizeof...(Args), void>::type
    map(std::tuple<Args...>& t, F&& f) { // terminate call
        _offset = 0;
        _packCounter =
                0; // WARNING value is renewed for every appendPack().  Not very
                   // efficient, but not very expensive to compute
    }
    // recursive call
    template <std::size_t I = 0, typename... Args, typename F>
            inline typename std::enable_if <
            I<sizeof...(Args), void>::type map(std::tuple<Args...>& t, F&& f) {
        // alias name for the current tuple-element's type
        typedef typename std::tuple_element<I, std::tuple<Args...>>::type
                tupleElement_t;

        // search for std::pairs or stl-like containers with std::pairs and
        // apply given function on each std::pair in a recursive manner
        const auto& arg = std::get<I>(t); // get tuple element
        if constexpr (is_container<std::remove_cv_t<std::remove_reference_t<
                              tupleElement_t>>>::value) {
            for (const auto& p : arg) {
                if constexpr (!is_pair<std::remove_cv_t<std::remove_reference_t<
                                      decltype(p)>>>::value) {
                    throw std::invalid_argument(
                            "SyncManager: Received invalid argument. Argument "
                            "must "
                            "be either std::pair or an STL-like container of "
                            "std::pairs");
                }
                std::forward<F>(f)(p, I + _offset);
            }
            --_offset; // decrement _offset by 1, since I will be +1 in next
                       // recursion
        } else if constexpr (is_pair<std::remove_cv_t<std::remove_reference_t<
                                     tupleElement_t>>>::value) {
            std::forward<F>(f)(arg, I + _offset);
            --_offset; // decrement _offset by 1, since I will be +1 in next
                       // recursion
        } else {
            throw std::invalid_argument(
                    "SyncManager: Received invalid argument. Argument "
                    "must "
                    "be either std::pair or an STL-like container of "
                    "std::pairs");
        }

        // keep track of the number of elements in Args... list ,
        // i.e. the variety of input "sensors" that exist in a pack
        ++_packCounter;

        // recursive call for other elements in tuple
        map<I + 1, Args...>(t, f);
    }

    bool isUniqueEntry(const double& value, const ETX& th, ETX& indRow) {
        const auto& v = _table.getIndependentColumn();
        if (!std::is_sorted(v.begin(), v.end()))
            throw std::runtime_error("Independent column is not sorted.");

        // find the first position in sorted vector to insert a new value, such
        // that the vector element is >= value.
        auto x2 = std::lower_bound(v.begin(), v.end(), value);

        if (x2 == v.begin())                         // --o-------front
            if (std::abs(v.front() - value) >= th) { // --o--|th--front
                indRow = value;
                return true;
            } else { // --|th---o---front
                indRow = v.front();
                return false;
            }
        else if (x2 == v.end())                     // back--------o--
            if (std::abs(v.back() - value) >= th) { // back---|th--o--
                indRow = value;
                return true;
            } else { // back---o---|th
                indRow = v.back();
                return false;
            }
        else { // front----x1-------o----------x2----back
            // get the previous position of x2
            const auto x1 = x2 - 1;

            // compute distances
            const ETX dist = *x2 - *x1; // element distance (x2-x1)
            const ETX ld = value - *x1; // lower distance (o - x1)
            const ETX ud = *x2 - value; // upper distance (x2 - o)

            if (dist >= 2 * th) { // x1-----|th1------|th2-----x2
                if (ld < th) {    // x1---o--|th1------|th2-----x2
                    indRow = *x1;
                    return false;
                } else if (ud < th) { // x1-----|th1------|th2---o--x2
                    indRow = *x2;
                    return false;
                } else { // x1-----|th1--o----|th2-----x2
                    indRow = value;
                    return true;
                }
            } else {                           // x1-----|th2------|th1-----x2
                if ((ld < th) && (ud >= th)) { // x1--o---|th2------|th1-----x2
                    indRow = *x1;
                } else if ((ud < th) &&
                           (ld >= th)) { // x1-----|th2------|th1--o---x2
                    indRow = *x2;
                } else { // x1-----|th2---o---|th1-----x2
                    indRow = (ld <= ud) ? *x1 : *x2;
                }
                return false;
            }
        }
    }

    // get ordered copy of the data table
    void reorderTable() {
        const auto& v = _table.getIndependentColumn();
        // create range of increasing values
        std::vector<size_t> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);

        // sort the range based on the values in independentColumn.
        // prefer stable_sort to avoid reordering same values
        std::stable_sort(
                idx.begin(), idx.end(),
                [&v](const auto& i1, const auto& i2) { return v[i1] < v[i2]; });

        // create copies of independentColumn and table's matrix
        std::vector<ETX> independentColumn = _table.getIndependentColumn();
        SimTK::Matrix matrix = _table.getMatrix(); // NOTE: auto will not copy

        // update the data in table with the correct order
        for (size_t i = 0; i < _table.getIndependentColumn().size(); ++i) {
            _table.setIndependentValueAtIndex(i, independentColumn[idx[i]]);
            _table.updMatrix().updRow(i) = matrix.row(idx[i]);
        }
    }

    SimTK::Vector createRow(const ETX& t) {
        SimTK::Vector rowNew(_table.getNumColumns(), SimTK::NaN);

        // create row at time (t) from linear interpolation of table columns
        for (size_t col = 0; col < _table.getNumColumns(); ++col) {
            const auto ids = find_if_indexesInVector(
                    _table.getMatrix()(col),
                    [](const auto& x) { return !SimTK::isNaN(x); });
            if (!ids.empty()) {
                SimTK::Vector x(ids.size()), y(ids.size());
                for (size_t i = 0; i < ids.size(); ++i) {
                    x[i] = _table.getIndependentColumn()[ids[i]];
                    y[i] = _table.getMatrix()(col)[ids[i]];
                }
                rowNew[col] = interpolate(x, y, SimTK::Vector(1, t))[0];
            }
        }
        return rowNew;
    }

    // return indexes {col, row} that
    template <typename V, typename F>
    std::vector<size_t> find_if_indexesInVector(V&& vec, F&& function) {
        // TODO add static assertion for function type
        std::vector<size_t> idx;
        for (size_t i = 0; i < vec.size(); ++i) {
            if (std::forward<F>(function)(std::forward<V>(vec)[i]))
                idx.push_back(i);
        }
        return idx;
    }

    void interpolateNanValues() {
        for (size_t col = 0; col < _table.getNumColumns(); ++col) {
            // find indexes in column that have non-nan values
            const auto non_nan_ids = find_if_indexesInVector(
                    _table.getMatrix()(col),
                    [](const auto& x) { return !SimTK::isNaN(x); });

            // find indexes in column that have nan values
            const auto nan_ids = find_if_indexesInVector(
                    _table.getMatrix()(col),
                    [](const auto& x) { return SimTK::isNaN(x); });

            // if there are nan values in the current column...
            if (!nan_ids.empty()) {
                // initialize vector sizes
                SimTK::Vector x(non_nan_ids.size()), y(non_nan_ids.size()),
                        x_nan(nan_ids.size());

                // create known vectors
                for (size_t i = 0; i < non_nan_ids.size(); ++i) {
                    x[i] = _table.getIndependentColumn()[non_nan_ids[i]];
                    y[i] = _table.getMatrix()(col)[non_nan_ids[i]];
                }

                // create unknown vector
                for (size_t i = 0; i < nan_ids.size(); ++i) {
                    x_nan[i] = _table.getIndependentColumn()[nan_ids[i]];
                }

                // interpolate
                auto temp = interpolate(x, y, x_nan);

                // update in table
                for (size_t i = 0; i < nan_ids.size(); ++i) {
                    const auto& row = _table.getIndependentColumn()[nan_ids[i]];
                    _table.updRow(row)[col] = std::move(temp[i]);
                }
            }
        }
    }

    SimTK::Vector interpolate(const SimTK::Vector& x, const SimTK::Vector& y,
                              const SimTK::Vector& newX) {
        if (x.size() != y.size())
            throw std::runtime_error("X and Y are not the same size.");

        // PiecewiseLinearFunction requires at least 2 points
        // columns with only one point are skipped
        if (x.size() < 2 || y.size() < 2)
            return SimTK::Vector(newX.size(), SimTK::NaN);

        OpenSim::PiecewiseLinearFunction linearFun((int) x.size(), &x[0],
                                                   &y[0]);

        SimTK::Vector yNew(newX.size(), SimTK::NaN);
        for (size_t i = 0; i < newX.size(); ++i) {
            if (x[0] <= newX[i] && newX[i] <= x[x.size() - 1])
                yNew[i] = linearFun.calcValue(SimTK::Vector(1, newX[i]));
        }
        return yNew;
    }

    template <typename InputIterator>
    void deleteRows(InputIterator first, InputIterator last) {
        while (first != last) { removeRow(*first++); }
    }

    bool _tableSizeIsSet = false;
    ETX _entryThreshold = 0;
    size_t _numColumns = 0;
    size_t _offset = 0;
    size_t _numOfPacks = 0;
    size_t _packCounter = 0;
    ETX _samplingRate = 0;
    double _currentTime = -SimTK::Infinity;
    bool _isCurrentTimeSet = false;
    std::map<size_t, size_t> _vectorSizePerPack;
    OpenSim::DataTable _table;
};
} // namespace OpenSimRT
