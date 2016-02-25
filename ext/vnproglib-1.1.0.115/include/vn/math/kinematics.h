// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides the structure quat.
#ifndef _VN_KINAMATICS_H_
#define _VN_KINAMATICS_H_

namespace vn {
namespace math {
namespace kinematics {

/// \brief Represents orientation in yaw, pitch, roll.
template<typename T>
struct ypr
{

	// Members ////////////////////////////////////////////////////////////////

public:

	/// \brief The yaw value in radians.
	T yaw;

	/// \brief The pitch value in radians.
	T pitch;

	/// \brief The roll value in radians.
	T roll;

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new uninitialized \ref ypr.
	ypr() { }

	/// \brief Creates a \ref ypr initialized to the provided values.
	///
	/// \param[in] yawInRads The initial \ref yaw value in radians.
	/// \param[in] pitchInRads The initial \ref pitch value in radians.
	/// \param[in] rollInRads The initial \ref roll value in radians.
	ypr(T yawInRads, T pitchInRads, T rollInRads) :
		yaw(yawInRads),
		pitch(pitchInRads),
		roll(rollInRads) { }

	// Helper Methods /////////////////////////////////////////////////////////

	/// \brief Returns a \ref ypr initialized with the provided values
	///     in degrees.
	///
	/// \param[in] yawInDegs The yaw value in degrees.
	/// \param[in] pitchInDegs The pitch value in degrees.
	/// \param[in] rollInDegs The roll value in degrees.
	/// \return The new \ref ypr initialized from the provided values.
	static ypr fromDegs(
		T yawInDegs,
		T pitchInDegs,
		T rollInDegs)
	{
		return ypr(
			vn::math::deg2rad(yawInDegs),
			vn::math::deg2rad(pitchInDegs),
			vn::math::deg2rad(rollInDegs));
	}

	/// \brief Returns a \ref ypr representing no rotation.
	///
	/// \return The \ref ypr representing no rotation.
	static ypr noRotation()
	{
		return ypr(0, 0, 0);
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Returns the \ref yaw value in degrees.
	///
	/// \return The \ref yaw value in degrees.
	T yawDegs() const
	{
		return math::rad2deg(Yyaw);
	}

	/// \brief Returns the \ref pitch value in degrees.
	///
	/// \return The \ref pitch value in degrees.
	T pitchDegs() const
	{
		return math::rad2deg(pitch);
	}

	/// \brief Returns the \ref roll value in degrees.
	///
	/// \return The \ref roll value in degrees.
	T rollDegs() const
	{
		return math::rad2deg(roll);
	}

	/// \brief Sets the \ref yaw to the value provided in degrees.
	///
	/// \param[in] yawInDegs The \ref yaw value in degrees.
	void setYawDegs(T yawInDegs)
	{
		yaw = math::deg2rad(yawInDegs);
	}

	/// \brief Sets the \ref pitch to the value provided in degrees.
	///
	/// \param[in] pitchInDegs The \ref pitch value in degrees.
	void setPitchDegs(T pitchInDegs)
	{
		pitch = math::deg2rad(pitchInDegs);
	}

	/// \brief Sets the \ref roll to the value provided in degrees.
	///
	/// \param[in] rollInDegs The \ref roll value in degrees.
	void setRollDegs(T rollInDegs)
	{
		roll = math::deg2rad(rollInDegs);
	}

	/// \brief Returns the \ref YawPitchRoll as a
	///     \ref vn::math::linearalgebra::Vector3.
	///
	/// \return The \ref YawPitchRoll as a
	///     \ref vn::math::linearalgebra::Vector3 with the components in
	///     radians.
/*	vn::math::linearalgebra::Vector3<T> ToVector3() const
	{
		return vn::math::linearalgebra::Vector3<T>(Yaw, Pitch, Roll);
	}*/

	/// \brief Returns the \ref YawPitchRoll as a
	///     \ref vn::math::linearalgebra::Vector3.
	///
	/// \return The \ref YawPitchRoll as a
	///     \ref vn::math::linearalgebra::Vector3 with the components in
	///     degrees.
/*	vn::math::linearalgebra::Vector3<T> ToVector3InDegs() const
	{
		return vn::math::linearalgebra::Vector3<T>(
			MConv::RadsToDegs(Yaw),
			MConv::RadsToDegs(Pitch),
			MConv::RadsToDegs(Roll));
	}*/
};

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief Data type representing a \ref ypr using <c>float</c> as the
///     underlying data type.
typedef ypr<float> yprf;

/// \brief Data type representing a \ref ypr using <c>double</c> as
///     the underlying data type.
typedef ypr<double> yprd;

/// \brief Data type representing a \ref ypr using <c>long double</c>
///     as the underlying data type.
typedef ypr<long double> yprld;

/// \brief Represents orientation as a quaternion.
template<typename T>
struct quat
{

	// Members ////////////////////////////////////////////////////////////////

public:

	/// \brief The x value.
	T x;

	/// \brief The y value.
	T y;

	/// \brief The z value.
	T z;

	/// \brief The w value.
	T w;

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new uninitialized \ref quat.
	quat() { }

	/// \brief Creates a \ref quat initialized to the provided values.
	///
	/// \param[in] x The initial \ref x value.
	/// \param[in] y The initial \ref y value.
	/// \param[in] z The initial \ref z value.
	/// \param[in] w The initial \ref w value.
	quat(T xIn, T yIn, T zIn, T wIn) :
		x(xIn),
		y(yIn),
		z(zIn),
		w(wIn) 
	{ }

	// Helper Methods /////////////////////////////////////////////////////////

public:

#if 0
	/// \brief Returns a \ref YawPitchRoll from the values in degrees.
	///
	/// \param[in] yawInDegs The yaw value in degrees.
	/// \param[in] pitchInDegs The pitch value in degrees.
	/// \param[in] rollInDegs The roll value in degrees.
	/// \return The new \ref YawPitchRoll initialized from the provided values.
	static YawPitchRoll CreateFromDegs(
		T yawInDegs,
		T pitchInDegs,
		T rollInDegs)
	{
		return YawPitchRoll(
			vn::math::MConv::DegsToRads(yawInDegs),
			vn::math::MConv::DegsToRads(pitchInDegs),
			vn::math::MConv::DegsToRads(rollInDegs));
	}
#endif

	/// \brief Returns a \ref quat representing no rotation.
	///
	/// \return The \ref quat representing no rotation.
	static quat noRotation()
	{
		return quat(0, 0, 0, 1);
	}

};

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief Data type representing a \ref quat using <c>float</c> as the
///     underlying data type.
typedef quat<float> quatf;

/// \brief Data type representing a \ref quat using <c>double</c> as
///     the underlying data type.
typedef quat<double> quatd;

/// \brief Data type representing a \ref quat using <c>long double</c>
///     as the underlying data type.
typedef quat<long double> quatld;

}
}
}

#endif
