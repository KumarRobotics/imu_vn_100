// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides the structure Coordinate2.
#ifndef _COORDINATE2_H_
#define _COORDINATE2_H_

#include "vn/int.h"

namespace vn {
namespace math {

/// \brief Represents a coordinate in a 2-dimensional space.
template<typename T>
struct Coordinate2
{
	// Members ////////////////////////////////////////////////////////////////

public:

	/// \brief The X component of the coordinate.
	T X;

	/// \brief The Y component of the coordinate.
	T Y;

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new /ref Coordinate2 with uninitialized X and Y.
	Coordinate2() { }

	/// \brief Creates a new /ref Coordinate2 initialized with the provided X
	///     and Y values.
	///
	/// \param[in] x The initial value for /ref X.
	/// \param[in] y The initial value for /ref Y.
	Coordinate2(T x, T y) : X(x), Y(y) { }

};

}
}

#endif
