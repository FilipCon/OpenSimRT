#pragma once

#include "internal/CommonExports.h"

#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vector_.h>
#include <SimTKcommon/internal/common.h>
#include <type_traits>
#include <utility>

namespace OpenSimRT {

// SFINAE type trait to detect whether T::const_iterator exists.
struct sfinae_base {
    using yes = char;
    using no = yes[2];
};

template <typename T> struct has_const_iterator : private sfinae_base {
 private:
    template <typename C> static yes& test(typename C::const_iterator*);
    template <typename C> static no& test(...);

 public:
    static const bool value = sizeof(test<T>(nullptr)) == sizeof(yes);
    using type = T;

    void dummy(); // for GCC to supress -Wctor-dtor-privacy
};

template <typename T> struct has_begin_end : private sfinae_base {
 private:
    template <typename C>
    static yes&
    f(typename std::enable_if<std::is_same<
              decltype(static_cast<typename C::const_iterator (C::*)() const>(
                      &C::begin)),
              typename C::const_iterator (C::*)() const>::value>::type*);

    template <typename C> static no& f(...);

    template <typename C>
    static yes&
    g(typename std::enable_if<
            std::is_same<decltype(static_cast<typename C::const_iterator (
                                          C::*)() const>(&C::end)),
                         typename C::const_iterator (C::*)() const>::value,
            void>::type*);

    template <typename C> static no& g(...);

 public:
    static bool const beg_value = sizeof(f<T>(nullptr)) == sizeof(yes);
    static bool const end_value = sizeof(g<T>(nullptr)) == sizeof(yes);

    void dummy(); // for GCC to supress -Wctor-dtor-privacy
};

template <typename T>
struct is_container
        : public std::integral_constant<bool,
                                        has_const_iterator<T>::value &&
                                                has_begin_end<T>::beg_value &&
                                                has_begin_end<T>::end_value> {};
// test if type is pair
template <typename> struct is_pair : std::false_type {};
template <typename T, typename U>
struct is_pair<std::pair<T, U>> : std::true_type {};

// simtk vec
template <typename> struct is_simtk_vec : std::false_type {};
template <int M, class ELT, int STRIDE>
struct is_simtk_vec<SimTK::Vec<M, ELT, STRIDE>> : std::true_type {};

// simtk vectors
// template <typename> struct is_simtk_row_vector_view : std::false_type {};
// template <class ELT>
// struct is_simtk_row_vector_view<SimTK::RowVectorView_<ELT>> : std::true_type {};

template <typename> struct is_simtk_vector : std::false_type {};
template <class ELT>
struct is_simtk_vector<SimTK::Vector_<ELT>> : std::true_type {};

} // namespace OpenSimRT
