#pragma once

#include "Exception.h"
#include "TypeHelpers.h"
#include "internal/CommonExports.h"

#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/NTraits.h>
#include <SimTKcommon/internal/VectorMath.h>
#include <SimTKcommon/internal/Vector_.h>
#include <SimTKcommon/internal/negator.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace OpenSimRT {

template <typename ETX = double> class Common_API SyncManager {
 public:
    SyncManager() = default; // default ctor, in order to have more ctors
    SyncManager(const ETX& threshold) { _entryThreshold = threshold; }
    SyncManager(const SyncManager&) = default;            // copy ctor
    SyncManager(SyncManager&&) = default;                 // move copy ctor
    SyncManager& operator=(const SyncManager&) = default; // assign copy ctor
    SyncManager& operator=(SyncManager&&) = default;      // assign move ctor
    ~SyncManager() = default;                             // dtor

    std::pair<ETX, SimTK::Vector_<ETX>> getPack(const size_t& packId,
                                                const size_t& row = 0) {
        if (((packId >= _numOfPacks) && (packId < 0)) ||
            ((row >= _table.getNumRows()) && (row < 0)))
            throw std::runtime_error("Invalid Input");
        size_t columnId = 0;
        for (size_t i = 0; i < packId; ++i) columnId += _vectorSizePerPack[i];
        double t = _table.getIndependentColumn()[row];
        SimTK::Vector v = ~_table.getMatrix().row(row)(
                columnId, _vectorSizePerPack[packId]);
        removeRowAtIndex(row);
        return std::move(std::make_pair(t, v));
    }

    template <typename... Args> void appendPack(Args&&... args) {
        // create tuple from args to allow different types
        using ArgsTuple = std::tuple<Args...>;
        ArgsTuple argTuple = {std::forward<Args>(args)...};

        // set the table size from the first appended pack
        if (!_tableSizeIsSet) {
            // apply function for each arg
            apply<0, Args...>(argTuple, [&](auto&& p,
                                            const size_t& dummy) { // c++14
                static_assert(
                        is_pair<std::remove_cv_t<
                                std::remove_reference_t<decltype(p)>>>::value,
                        "Only std::pair is allowed");
                typedef std::remove_cv_t<
                        std::remove_reference_t<decltype(p.second)>>
                        pairSecondElement_t;

                // second is SimTK::Vec
                if constexpr (is_simtk_vec<pairSecondElement_t>::value ||
                              is_simtk_vector<pairSecondElement_t>::value) {
                    _numColumns += p.second.size();
                    _vectorSizePerPack[_numOfPacks] = p.second.size();

                    // second is Container<SimTK::Vec>
                } else if (is_container<pairSecondElement_t>::value) {
                    for (auto& v : p.second) {
                        _numColumns += v.size();
                        _vectorSizePerPack[_numOfPacks] += v.size();
                    }
                } else {
                    throw std::invalid_argument(
                            "Invalid std::pair::second type");
                }
            });

            // create first entry with NaNs
            _table.appendRow(ETX(), std::vector<ETX>(_numColumns, SimTK::NaN));
            _tableSizeIsSet = true;
        }

        // apply function for each arg
        apply<0, Args...>(argTuple, [&](auto&& p,
                                        const size_t& columnId) { // c++14
            typedef std::remove_cv_t<
                    std::remove_reference_t<decltype(p.second)>>
                    pairSecondElement_t;
            ETX entry;
            // append new row or update table rows
            if (isUniqueEntry(p.first, _entryThreshold, entry)) { // append
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
                appendRow(entry, newRow);

            } else { // update existing row

                // std::pair::second is simtk vec
                if constexpr (is_simtk_vec<pairSecondElement_t>::value ||
                              is_simtk_vector<pairSecondElement_t>::value) {
                    for (int i = 0; i < p.second.size(); ++i) {
                        _table.updRow(entry).updCol(i + columnId) =
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
                            _table.updRow(entry).updCol(j + columnId + offset) =
                                    std::move(v[j]);
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

            // after entering a new entry {time, data}, reorder the table rows
            // based on the ordered independentColumn
            reorderTable();
        });

        // fill nan values in columns using linear interpolation // TODO
        interpolateColumns();
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

    // set row at index
    template <typename... Args>
    void setRowAtIndex(const size_t& index, Args&&... args) {
        _table.setRowAtIndex(index, std::forward<Args>(args)...);
    }

    // get writable reference to row in table
    template <typename... Args> void updRow(const ETX& indRow, Args&&... args) {
        _table.updRow(indRow, std::forward<Args>(args)...);
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
    apply(std::tuple<Args...>& t, F&& f) { // terminate call
        _offset = 0;
        _numOfPacks = 0; // WARNING value is renewed for every appendPack(). Not
                         // very efficient, but not very expensive to compute
    }
    // recursive call
    template <std::size_t I = 0, typename... Args, typename F>
            inline typename std::enable_if <
            I<sizeof...(Args), void>::type apply(std::tuple<Args...>& t,
                                                 F&& f) {
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
        } else if (is_pair<std::remove_cv_t<
                           std::remove_reference_t<tupleElement_t>>>::value) {
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
        ++_numOfPacks;

        // recursive call for other elements in tuple
        apply<I + 1, Args...>(t, f);
    }

    bool isUniqueEntry(const double& value, const ETX& th, ETX& entry) {
        const auto& vec = _table.getIndependentColumn();

        // assumes ordered vector
        auto x2 = std::lower_bound(vec.begin(), vec.end(), value);
        decltype(x2) x1;
        if (x2 == vec.end()) {
            entry = ((value - vec.back()) >= th) ? value : vec.back();
            return ((value - vec.back()) >= th) ? true : false;
        } else if (x2 == vec.begin())
            x1 = x2;
        else
            x1 = x2 - 1; // get previous element
        while (SimTK::isNaN(*x2)) std::advance(x2, 1);

        const ETX ld = value - *x1;
        const ETX ud = *x2 - value;

        bool isValueInBetweenX1X2 = ((ld >= 0) && (ud >= 0));
        bool isRangeGreaterOrEqualThanDoubleThreshold = ((*x2 - *x1) >= 2 * th);

        if (isValueInBetweenX1X2 && !isRangeGreaterOrEqualThanDoubleThreshold) {
            if ((ld < th) && (ud >= th)) {
                entry = *x1;
            } else if ((ud < th) && (ld >= th)) {
                entry = *x2;
            } else {
                entry = (ld <= ud) ? *x1 : *x2;
            }
            return false;
        } else if (isValueInBetweenX1X2 &&
                   isRangeGreaterOrEqualThanDoubleThreshold) {
            if (ld < th) {
                entry = *x1;
                return false;
            } else if (ud < th) {
                entry = *x2;
                return false;
            } else {
                entry = value;
                return true;
            }
        } else if (!isValueInBetweenX1X2) {
            if ((ld < 0) && (std::abs(ld) >= th)) {
                entry = value;
                return true;
            } else {
                entry = *x1;
                return false;
            }
            if ((ud < 0) && (std::abs(ud) >= th)) {
                entry = value;
                return true;
            } else {
                entry = *x2;
                return false;
            }
        } else
            throw std::runtime_error("Something went wrong");
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

    void interpolateColumns() {
        for (size_t col = 0; col < _table.getMatrix().ncol(); ++col) {
            std::vector<ETX> x;
            std::vector<ETX> y;
            std::vector<ETX> newX;
            std::vector<size_t> newX_id;

            for (size_t row = 0; row < _table.getMatrix().nrow(); ++row) {
                if (!SimTK::isNaN(_table.getMatrix().col(col)[row])) {
                    x.push_back(_table.getIndependentColumn()[row]);
                    y.push_back(_table.getMatrix().col(col)[row]);
                } else {
                    newX.push_back(_table.getIndependentColumn()[row]);
                    newX_id.push_back(row);
                }
            }

            // PiecewiseLinearFunction requires at least 2 points
            // columns with only one point are skipped
            if (x.size() < 2 || y.size() < 2) continue;

            OpenSim::PiecewiseLinearFunction function((int) x.size(), &x[0],
                                                      &y[0]);
            SimTK::Vector newY(newX.size(), SimTK::NaN);
            for (size_t i = 0; i < newX.size(); ++i) {
                const auto& newXi = newX[i];
                const auto& row = newX_id[i];
                // if (x[0] <= newXi && newXi <= x[x.size() - 1]) {
                _table.updMatrix().updCol(col).updRow(row) =
                        function.calcValue(SimTK::Vector(1, newXi));
                // }
            }
        }
    }

    // TODO
    void resampleTable(const ETX& samplingRate) {}

    bool _tableSizeIsSet = false;
    ETX _entryThreshold = 0;
    size_t _numColumns = 0;
    size_t _offset = 0;
    size_t _numOfPacks = 0;
    std::map<size_t, size_t> _vectorSizePerPack;
    OpenSim::DataTable _table;
};
} // namespace OpenSimRT
