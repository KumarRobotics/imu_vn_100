// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#ifndef _VN_MATH_YPR_H_
#define _VN_MATH_YPR_H_

#include "vn/math/conversions.h"
#include "vn/math/vector.h"

namespace vn {
namespace math {

template <typename T = float>
struct ypr
{
	// Public Members /////////////////////////////////////////////////////////
public:

	union
	{
		T yaw;
		T y;
	};

	union
	{
		T pitch;
		T p;
	};

	union
	{
		T roll;
		T r;
	};

	// Helper Methods /////////////////////////////////////////////////////////
public:

	/// \brief Yaw, pitch, roll representing no rotation.
	///
	/// \return No rotation.
	static ypr noRotation()
	{
		return ypr<T>(0, 0, 0);
	}

	static ypr fromDegs(T yawInDegs, T pitchInDegs, T rollInDegs)
	{
		return ypr(deg2rad(yawInDegs), deg2rad(pitchInDegs), deg2rad(rollInDegs));
	}

	static ypr fromDegs(vec3 valuesInDegs)
	{
		return fromDegs(valuesInDegs.x, valuesInDegs.y, valuesInDegs.z);
	}

	// Constructors ///////////////////////////////////////////////////////////

public:

	ypr(T yawInRads, T pitchInRads, T rollInRads) :
		yaw(yawInRads),
		pitch(pitchInRads),
		roll(rollInRads)
	{ }

	explicit ypr(vec3 valuesInRads) :
		yaw(valuesInRads.x),
		pitch(valuesInRads.y),
		roll(valuesInRads.z)
	{ }

	// Methods ////////////////////////////////////////////////////////////////

public:

	vec3 toVec3()
	{
		return vec3(yaw, pitch, roll);
	}

};

typedef ypr<float> yprf;

}
}

#endif
