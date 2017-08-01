// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#include "vn/util/boostpython.h"

#include "vn/sensors/sensors.h"
#include "vn/sensors/ezasyncdata.h"
#include "vn/sensors/compositedata.h"
#include "vn/math/position.h"

using namespace std;
using namespace boost::python;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(send_overloads, send, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSettings_overloads, writeSettings, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(restoreFactorySettings_overloads, restoreFactorySettings, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(reset_overloads, reset, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSerialBaudRate_overloads2, writeSerialBaudRate, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAsyncDataOutputType_overloads2, writeAsyncDataOutputType, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAsyncDataOutputFrequency_overloads2, writeAsyncDataOutputFrequency, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsBasicConfigurationVn200_overloads, writeInsBasicConfigurationVn200, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsBasicConfigurationVn300_overloads, writeInsBasicConfigurationVn300, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsBasicConfigurationVn200NoStruct_overloads, writeInsBasicConfigurationVn200, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsBasicConfigurationVn300NoStruct_overloads, writeInsBasicConfigurationVn300, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeUserTag_overloads, writeUserTag, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSerialBaudRate_overloads, writeSerialBaudRate, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAsyncDataOutputType_overloads, writeAsyncDataOutputType, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAsyncDataOutputFrequency_overloads, writeAsyncDataOutputFrequency, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagneticAndGravityReferenceVectors_overloads, writeMagneticAndGravityReferenceVectors, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagneticAndGravityReferenceVectorsNoStruct_overloads, writeMagneticAndGravityReferenceVectors, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterMeasurementsVarianceParameters_overloads, writeFilterMeasurementsVarianceParameters, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterMeasurementsVarianceParametersNoStruct_overloads, writeFilterMeasurementsVarianceParameters, 4, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagnetometerCompensation_overloads, writeMagnetometerCompensation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagnetometerCompensationNoStruct_overloads, writeMagnetometerCompensation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterActiveTuningParameters_overloads, writeFilterActiveTuningParameters, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterActiveTuningParametersNoStruct_overloads, writeFilterActiveTuningParameters, 4, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAccelerationCompensation_overloads, writeAccelerationCompensation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeAccelerationCompensationNoStruct_overloads, writeAccelerationCompensation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeReferenceFrameRotation_overloads, writeReferenceFrameRotation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeCommunicationProtocolControl_overloads, writeCommunicationProtocolControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeCommunicationProtocolControlNoStruct_overloads, writeCommunicationProtocolControl, 7, 8)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSynchronizationControl_overloads, writeSynchronizationControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSynchronizationControlNoStruct_overloads, writeSynchronizationControl, 7, 8)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSynchronizationStatus_overloads, writeSynchronizationStatus, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeSynchronizationStatusNoStruct_overloads, writeSynchronizationStatus, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterBasicControl_overloads, writeFilterBasicControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterBasicControlNoStruct_overloads, writeFilterBasicControl, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeBasicControl_overloads, writeVpeBasicControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeBasicControlNoStruct_overloads, writeVpeBasicControl, 4, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeMagnetometerBasicTuning_overloads, writeVpeMagnetometerBasicTuning, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeMagnetometerBasicTuningNoStruct_overloads, writeVpeMagnetometerBasicTuning, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeMagnetometerAdvancedTuning_overloads, writeVpeMagnetometerAdvancedTuning, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeMagnetometerAdvancedTuningNoStruct_overloads, writeVpeMagnetometerAdvancedTuning, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeAccelerometerBasicTuning_overloads, writeVpeAccelerometerBasicTuning, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeAccelerometerBasicTuningNoStruct_overloads, writeVpeAccelerometerBasicTuning, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeAccelerometerAdvancedTuning_overloads, writeVpeAccelerometerAdvancedTuning, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeAccelerometerAdvancedTuningNoStruct_overloads, writeVpeAccelerometerAdvancedTuning, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeGryoBasicTuning_overloads, writeVpeGryoBasicTuning, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVpeGryoBasicTuningNoStruct_overloads, writeVpeGryoBasicTuning, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeFilterStartupGyroBias_overloads, writeFilterStartupGyroBias, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagnetometerCalibrationControl_overloads, writeMagnetometerCalibrationControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeMagnetometerCalibrationControlNoStruct_overloads, writeMagnetometerCalibrationControl, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeIndoorHeadingModeControl_overloads, writeIndoorHeadingModeControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVelocityCompensationMeasurement_overloads, writeVelocityCompensationMeasurement, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVelocityCompensationControl_overloads, writeVelocityCompensationControl, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeVelocityCompensationControlNoStruct_overloads, writeVelocityCompensationControl, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGpsConfiguration_overloads, writeGpsConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGpsConfigurationNoStruct_overloads, writeGpsConfiguration, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGpsAntennaOffset_overloads, writeGpsAntennaOffset, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsAdvancedConfiguration_overloads, writeInsAdvancedConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeInsAdvancedConfigurationNoStruct_overloads, writeInsAdvancedConfiguration, 15, 16)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeStartupFilterBiasEstimate_overloads, writeStartupFilterBiasEstimate, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeStartupFilterBiasEstimateNoStruct_overloads, writeStartupFilterBiasEstimate, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput1_overloads, writeBinaryOutput1, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput1NoStruct_overloads, writeBinaryOutput1, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput2_overloads, writeBinaryOutput2, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput2NoStruct_overloads, writeBinaryOutput2, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput3_overloads, writeBinaryOutput3, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput3NoStruct_overloads, writeBinaryOutput3, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput4_overloads, writeBinaryOutput4, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput4NoStruct_overloads, writeBinaryOutput4, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput5_overloads, writeBinaryOutput5, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeBinaryOutput5NoStruct_overloads, writeBinaryOutput5, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeDeltaThetaAndDeltaVelocityConfiguration_overloads, writeDeltaThetaAndDeltaVelocityConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeDeltaThetaAndDeltaVelocityConfigurationNoStruct_overloads, writeDeltaThetaAndDeltaVelocityConfiguration, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeReferenceVectorConfiguration_overloads, writeReferenceVectorConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeReferenceVectorConfigurationNoStruct_overloads, writeReferenceVectorConfiguration, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGyroCompensation_overloads, writeGyroCompensation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGyroCompensationNoStruct_overloads, writeGyroCompensation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeImuFilteringConfiguration_overloads, writeImuFilteringConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeImuFilteringConfigurationNoStruct_overloads, writeImuFilteringConfiguration, 10, 11)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGpsCompassBaseline_overloads, writeGpsCompassBaseline, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeGpsCompassBaselineNoStruct_overloads, writeGpsCompassBaseline, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeImuRateConfiguration_overloads, writeImuRateConfiguration, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(writeImuRateConfigurationNoStruct_overloads, writeImuRateConfiguration, 4, 5)

void sensorErrorTranslator(sensor_error const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

BOOST_PYTHON_MODULE(_sensors)
{
	PyEval_InitThreads();

	docstring_options local_docstring_options(true, true, false);

	register_exception_translator<sensor_error>(&sensorErrorTranslator);

	class_<CompositeData>("CompositeData")
		.def("parse", static_cast<CompositeData (*)(Packet&)>(&CompositeData::parse)).staticmethod("parse")
		.def_readonly("has_any_attitude", &CompositeData::hasAnyAttitude)
		.def_readonly("any_attitude", &CompositeData::anyAttitude)
		.def_readonly("has_yaw_pitch_roll", &CompositeData::hasYawPitchRoll)
		.def_readonly("yaw_pitch_roll", &CompositeData::yawPitchRoll)
		.def_readonly("has_quaternion", &CompositeData::hasQuaternion)
		.def_readonly("quaternion", &CompositeData::quaternion)
		.def_readonly("has_direction_cosine_matrix", &CompositeData::hasDirectionCosineMatrix)
		.def_readonly("direction_cosine_matrix", &CompositeData::directionCosineMatrix)
		.def_readonly("reset", &CompositeData::reset)
		.def_readonly("has_any_magnetic", &CompositeData::hasAnyMagnetic)
		.def_readonly("any_magnetic", &CompositeData::anyMagnetic)
		.def_readonly("has_magnetic", &CompositeData::hasMagnetic)
		.def_readonly("magnetic", &CompositeData::magnetic)
		.def_readonly("has_magnetic_uncompensated", &CompositeData::hasMagneticUncompensated)
		.def_readonly("magnetic_uncompensated", &CompositeData::magneticUncompensated)
		.def_readonly("has_magnetic_ned", &CompositeData::hasMagneticNed)
		.def_readonly("magnetic_ned", &CompositeData::magneticNed)
		.def_readonly("has_magnetic_ecef", &CompositeData::hasMagneticEcef)
		.def_readonly("magnetic_ecef", &CompositeData::magneticEcef)
		.def_readonly("has_magnetic_raw", &CompositeData::hasMagneticRaw)
		.def_readonly("magnetic_raw", &CompositeData::magneticRaw)
		.def_readonly("has_any_acceleration", &CompositeData::hasAnyAcceleration)
		.def_readonly("any_acceleration", &CompositeData::anyAcceleration)
		.def_readonly("has_acceleration", &CompositeData::hasAcceleration)
		.def_readonly("acceleration", &CompositeData::acceleration)
		.def_readonly("has_acceleration_linear_body", &CompositeData::hasAccelerationLinearBody)
		.def_readonly("acceleration_linear_body", &CompositeData::accelerationLinearBody)
		.def_readonly("has_acceleration_uncompensated", &CompositeData::hasAccelerationUncompenated)
		.def_readonly("acceleration_uncompensated", &CompositeData::accelerationUncompensated)
		.def_readonly("has_acceleration_linear_ned", &CompositeData::hasAccelerationLinearNed)
		.def_readonly("acceleration_linear_ned", &CompositeData::accelerationLinearNed)
		.def_readonly("has_acceleration_linear_ecef", &CompositeData::hasAccelerationLinearEcef)
		.def_readonly("acceleration_linear_ecef", &CompositeData::accelerationLinearEcef)
		.def_readonly("has_acceleration_ned", &CompositeData::hasAccelerationNed)
		.def_readonly("acceleration_ned", &CompositeData::accelerationNed)
		.def_readonly("has_acceleration_ecef", &CompositeData::hasAccelerationEcef)
		.def_readonly("acceleration_ecef", &CompositeData::accelerationEcef)
		.def_readonly("has_acceleration_raw", &CompositeData::hasAccelerationRaw)
		.def_readonly("acceleration_raw", &CompositeData::accelerationRaw)
		.def_readonly("has_any_angular_rate", &CompositeData::hasAnyAngularRate)
		.def_readonly("any_angular_rate", &CompositeData::anyAngularRate)
		.def_readonly("has_angular_rate", &CompositeData::hasAngularRate)
		.def_readonly("angular_rate", &CompositeData::angularRate)
		.def_readonly("has_angular_rate_uncompensated", &CompositeData::hasAngularRateUncompensated)
		.def_readonly("angular_rate_uncompensated", &CompositeData::angularRateUncompensated)
		.def_readonly("has_angular_rate_raw", &CompositeData::hasAngularRateRaw)
		.def_readonly("angular_rate_raw", &CompositeData::angularRateRaw)
		.def_readonly("has_any_temperature", &CompositeData::hasAnyTemperature)
		.def_readonly("any_temperature", &CompositeData::anyTemperature)
		.def_readonly("has_temperature", &CompositeData::hasTemperature)
		.def_readonly("temperature", &CompositeData::temperature)
		.def_readonly("has_temperature_raw", &CompositeData::hasTemperatureRaw)
		.def_readonly("temperature_raw", &CompositeData::temperatureRaw)
		.def_readonly("has_any_pressure", &CompositeData::hasAnyPressure)
		.def_readonly("any_pressure", &CompositeData::anyPressure)
		.def_readonly("has_pressure", &CompositeData::hasPressure)
		.def_readonly("pressure", &CompositeData::pressure)
		.def_readonly("has_any_position", &CompositeData::hasAnyPosition)
		.def_readonly("any_position", &CompositeData::anyPosition)
		.def_readonly("has_position_gps_lla", &CompositeData::hasPositionGpsLla)
		.def_readonly("position_gps_lla", &CompositeData::positionGpsLla)
		.def_readonly("has_position_gps_ecef", &CompositeData::hasPositionGpsEcef)
		.def_readonly("position_gps_ecef", &CompositeData::positionGpsEcef)
		.def_readonly("has_position_estimated_lla", &CompositeData::hasPositionEstimatedLla)
		.def_readonly("position_estimated_lla", &CompositeData::positionEstimatedLla)
		.def_readonly("has_position_estimated_ecef", &CompositeData::hasPositionEstimatedEcef)
		.def_readonly("position_estimated_ecef", &CompositeData::positionEstimatedEcef)
		.def_readonly("has_any_velocity", &CompositeData::hasAnyVelocity)
		.def_readonly("any_velocity", &CompositeData::anyVelocity)
		.def_readonly("has_velocity_gps_ned", &CompositeData::hasVelocityGpsNed)
		.def_readonly("velocity_gps_ned", &CompositeData::velocityGpsNed)
		.def_readonly("has_velocity_gps_ecef", &CompositeData::hasVelocityGpsEcef)
		.def_readonly("velocity_gps_ecef", &CompositeData::velocityGpsEcef)
		.def_readonly("has_velocity_estimated_ned", &CompositeData::hasVelocityEstimatedNed)
		.def_readonly("velocity_estimated_ned", &CompositeData::velocityEstimatedNed)
		.def_readonly("has_velocity_estimated_ecef", &CompositeData::hasVelocityEstimatedEcef)
		.def_readonly("velocity_estimated_ecef", &CompositeData::velocityEstimatedEcef)
		.def_readonly("has_velocity_estimated_body", &CompositeData::hasVelocityEstimatedBody)
		.def_readonly("velocity_estimated_body", &CompositeData::velocityEstimatedBody)
		.def_readonly("has_delta_time", &CompositeData::hasDeltaTime)
		.def_readonly("delta_time", &CompositeData::deltaTime)
		.def_readonly("has_delta_theta", &CompositeData::hasDeltaTheta)
		.def_readonly("delta_theta", &CompositeData::deltaTheta)
		.def_readonly("has_delta_velocity", &CompositeData::hasDeltaVelocity)
		.def_readonly("delta_velocity", &CompositeData::deltaVelocity)
		.def_readonly("has_time_startup", &CompositeData::hasTimeStartup)
		.def_readonly("time_startup", &CompositeData::timeStartup)
		.def_readonly("has_time_gps", &CompositeData::hasTimeGps)
		.def_readonly("time_gps", &CompositeData::timeGps)
		.def_readonly("has_tow", &CompositeData::hasTow)
		.def_readonly("tow", &CompositeData::tow)
		.def_readonly("has_week", &CompositeData::hasWeek)
		.def_readonly("week", &CompositeData::week)
		.def_readonly("has_num_sats", &CompositeData::hasNumSats)
		.def_readonly("num_sats", &CompositeData::numSats)
		.def_readonly("has_time_sync_in", &CompositeData::hasTimeSyncIn)
		.def_readonly("time_sync_in", &CompositeData::timeSyncIn)
		.def_readonly("has_vpe_status", &CompositeData::hasVpeStatus)
		.def_readonly("vpe_status", &CompositeData::vpeStatus)
		.def_readonly("has_ins_status", &CompositeData::hasInsStatus)
		.def_readonly("ins_status", &CompositeData::insStatus)
		.def_readonly("has_sync_in_cnt", &CompositeData::hasSyncInCnt)
		.def_readonly("sync_in_cnt", &CompositeData::syncInCnt)
		.def_readonly("has_time_gps_pps", &CompositeData::hasTimeGpsPps)
		.def_readonly("time_gps_pps", &CompositeData::timeGpsPps)
		.def_readonly("has_gps_tow", &CompositeData::hasGpsTow)
		.def_readonly("gps_tow", &CompositeData::gpsTow)
		.def_readonly("has_time_utc", &CompositeData::hasTimeUtc)
		.def_readonly("time_utc", &CompositeData::timeUtc)
		.def_readonly("has_sens_sat", &CompositeData::hasSensSat)
		.def_readonly("sens_sat", &CompositeData::sensSat)
		.def_readonly("has_fix", &CompositeData::hasFix)
		.def_readonly("fix", &CompositeData::fix)
		.def_readonly("has_any_position_uncertainty", &CompositeData::hasAnyPositionUncertainty)
		.def_readonly("any_position_uncertainty", &CompositeData::anyPositionUncertainty)
		.def_readonly("has_position_uncertainty_gps_ned", &CompositeData::hasPositionUncertaintyGpsNed)
		.def_readonly("position_uncertainty_gps_ned", &CompositeData::positionUncertaintyGpsNed)
		.def_readonly("has_position_uncertainty_gps_ecef", &CompositeData::hasPositionUncertaintyGpsEcef)
		.def_readonly("position_uncertainty_gps_ecef", &CompositeData::positionUncertaintyGpsEcef)
		.def_readonly("has_position_uncertainty_estimated", &CompositeData::hasPositionUncertaintyEstimated)
		.def_readonly("position_uncertainty_estimated", &CompositeData::positionUncertaintyEstimated)
		.def_readonly("has_any_velocity_uncertainty", &CompositeData::hasAnyVelocityUncertainty)
		.def_readonly("any_velocity_uncertainty", &CompositeData::anyVelocityUncertainty)
		.def_readonly("has_velocity_uncertainty_gps", &CompositeData::hasVelocityUncertaintyGps)
		.def_readonly("velocity_uncertainty_gps", &CompositeData::velocityUncertaintyGps)
		.def_readonly("has_velocity_uncertainty_estimated", &CompositeData::hasVelocityUncertaintyEstimated)
		.def_readonly("velocity_uncertainty_estimated", &CompositeData::velocityUncertaintyEstimated)
		.def_readonly("has_time_uncertainty", &CompositeData::hasTimeUncertainty)
		.def_readonly("time_uncertainty", &CompositeData::timeUncertainty)
		.def_readonly("has_attitude_uncertainty", &CompositeData::hasAttitudeUncertainty)
		.def_readonly("attitude_uncertainty", &CompositeData::attitudeUncertainty)
		.def_readonly("has_ypr_rates", &CompositeData::hasYprRates)
		.def_readonly("ypr_rates", &CompositeData::yprRates)
		;

	class_<BinaryOutputRegister>("BinaryOutputRegister", "Class representing the fields of the Binary Output registers.")
		.def(init<uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t>())
		.def_readwrite("async_mode", &BinaryOutputRegister::asyncMode)
		.def_readwrite("rate_divisor", &BinaryOutputRegister::rateDivisor)
		.def_readwrite("common_field", &BinaryOutputRegister::commonField)
		.def_readwrite("time_field", &BinaryOutputRegister::timeField)
		.def_readwrite("imu_field", &BinaryOutputRegister::imuField)
		.def_readwrite("gps_field", &BinaryOutputRegister::gpsField)
		.def_readwrite("attitude_field", &BinaryOutputRegister::attitudeField)
		.def_readwrite("ins_field", &BinaryOutputRegister::insField)
		;

	class_<VnSensor::AsyncPacketReceivedEvent, boost::noncopyable>("AsyncPacketReceivedEvent", "")
		.def(self += other<PyObject*>())
		.def("add", &VnSensor::AsyncPacketReceivedEvent::add)
		.def("remove", &VnSensor::AsyncPacketReceivedEvent::remove)
		.def("fire", &VnSensor::AsyncPacketReceivedEvent::fire)
		;

	// Generate wrappers for our register structures.
	class_<QuaternionMagneticAccelerationAndAngularRatesRegister>("QuaternionMagneticAccelerationAndAngularRatesRegister", "Class representing the Quaternion, Magnetic, Acceleration and Angular Rates register.")
		.def(init<vec4f, vec3f, vec3f, vec3f>())
		.def_readwrite("quat", &QuaternionMagneticAccelerationAndAngularRatesRegister::quat)
		.def_readwrite("mag", &QuaternionMagneticAccelerationAndAngularRatesRegister::mag)
		.def_readwrite("accel", &QuaternionMagneticAccelerationAndAngularRatesRegister::accel)
		.def_readwrite("gyro", &QuaternionMagneticAccelerationAndAngularRatesRegister::gyro)
		;
	class_<MagneticAccelerationAndAngularRatesRegister>("MagneticAccelerationAndAngularRatesRegister", "Class representing the Magnetic, Acceleration and Angular Rates register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("mag", &MagneticAccelerationAndAngularRatesRegister::mag)
		.def_readwrite("accel", &MagneticAccelerationAndAngularRatesRegister::accel)
		.def_readwrite("gyro", &MagneticAccelerationAndAngularRatesRegister::gyro)
		;
	class_<MagneticAndGravityReferenceVectorsRegister>("MagneticAndGravityReferenceVectorsRegister", "Class representing the Magnetic and Gravity Reference Vectors register.")
		.def(init<vec3f, vec3f>())
		.def_readwrite("mag_ref", &MagneticAndGravityReferenceVectorsRegister::magRef)
		.def_readwrite("acc_ref", &MagneticAndGravityReferenceVectorsRegister::accRef)
		;
	class_<FilterMeasurementsVarianceParametersRegister>("FilterMeasurementsVarianceParametersRegister", "Class representing the Filter Measurements Variance Parameters register.")
		.def(init<float, vec3f, vec3f, vec3f>())
		.def_readwrite("angular_walk_variance", &FilterMeasurementsVarianceParametersRegister::angularWalkVariance)
		.def_readwrite("angular_rate_variance", &FilterMeasurementsVarianceParametersRegister::angularRateVariance)
		.def_readwrite("magnetic_variance", &FilterMeasurementsVarianceParametersRegister::magneticVariance)
		.def_readwrite("acceleration_variance", &FilterMeasurementsVarianceParametersRegister::accelerationVariance)
		;
	class_<MagnetometerCompensationRegister>("MagnetometerCompensationRegister", "Class representing the Magnetometer Compensation register.")
		.def(init<mat3f, vec3f>())
		.def_readwrite("c", &MagnetometerCompensationRegister::c)
		.def_readwrite("b", &MagnetometerCompensationRegister::b)
		;
	class_<FilterActiveTuningParametersRegister>("FilterActiveTuningParametersRegister", "Class representing the Filter Active Tuning Parameters register.")
		.def(init<float, float, float, float>())
		.def_readwrite("magnetic_disturbance_gain", &FilterActiveTuningParametersRegister::magneticDisturbanceGain)
		.def_readwrite("acceleration_disturbance_gain", &FilterActiveTuningParametersRegister::accelerationDisturbanceGain)
		.def_readwrite("magnetic_disturbance_memory", &FilterActiveTuningParametersRegister::magneticDisturbanceMemory)
		.def_readwrite("acceleration_disturbance_memory", &FilterActiveTuningParametersRegister::accelerationDisturbanceMemory)
		;
	class_<AccelerationCompensationRegister>("AccelerationCompensationRegister", "Class representing the Acceleration Compensation register.")
		.def(init<mat3f, vec3f>())
		.def_readwrite("c", &AccelerationCompensationRegister::c)
		.def_readwrite("b", &AccelerationCompensationRegister::b)
		;
	class_<YawPitchRollMagneticAccelerationAndAngularRatesRegister>("YawPitchRollMagneticAccelerationAndAngularRatesRegister", "Class representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.")
		.def(init<vec3f, vec3f, vec3f, vec3f>())
		.def_readwrite("yaw_pitch_roll", &YawPitchRollMagneticAccelerationAndAngularRatesRegister::yawPitchRoll)
		.def_readwrite("mag", &YawPitchRollMagneticAccelerationAndAngularRatesRegister::mag)
		.def_readwrite("accel", &YawPitchRollMagneticAccelerationAndAngularRatesRegister::accel)
		.def_readwrite("gyro", &YawPitchRollMagneticAccelerationAndAngularRatesRegister::gyro)
		;
	class_<CommunicationProtocolControlRegister>("CommunicationProtocolControlRegister", "Class representing the Communication Protocol Control register.")
		.def(init<CountMode, StatusMode, CountMode, StatusMode, ChecksumMode, ChecksumMode, ErrorMode>())
		.def_readwrite("serial_count", &CommunicationProtocolControlRegister::serialCount)
		.def_readwrite("serial_status", &CommunicationProtocolControlRegister::serialStatus)
		.def_readwrite("spi_count", &CommunicationProtocolControlRegister::spiCount)
		.def_readwrite("spi_status", &CommunicationProtocolControlRegister::spiStatus)
		.def_readwrite("serial_checksum", &CommunicationProtocolControlRegister::serialChecksum)
		.def_readwrite("spi_checksum", &CommunicationProtocolControlRegister::spiChecksum)
		.def_readwrite("error_mode", &CommunicationProtocolControlRegister::errorMode)
		;
	class_<SynchronizationControlRegister>("SynchronizationControlRegister", "Class representing the Synchronization Control register.")
		.def(init<SyncInMode, SyncInEdge, uint16_t, SyncOutMode, SyncOutPolarity, uint16_t, uint32_t>())
		.def_readwrite("sync_in_mode", &SynchronizationControlRegister::syncInMode)
		.def_readwrite("sync_in_edge", &SynchronizationControlRegister::syncInEdge)
		.def_readwrite("sync_in_skip_factor", &SynchronizationControlRegister::syncInSkipFactor)
		.def_readwrite("sync_out_mode", &SynchronizationControlRegister::syncOutMode)
		.def_readwrite("sync_out_polarity", &SynchronizationControlRegister::syncOutPolarity)
		.def_readwrite("sync_out_skip_factor", &SynchronizationControlRegister::syncOutSkipFactor)
		.def_readwrite("sync_out_pulse_width", &SynchronizationControlRegister::syncOutPulseWidth)
		;
	class_<SynchronizationStatusRegister>("SynchronizationStatusRegister", "Class representing the Synchronization Status register.")
		.def(init<uint32_t, uint32_t, uint32_t>())
		.def_readwrite("sync_in_count", &SynchronizationStatusRegister::syncInCount)
		.def_readwrite("sync_in_time", &SynchronizationStatusRegister::syncInTime)
		.def_readwrite("sync_out_count", &SynchronizationStatusRegister::syncOutCount)
		;
	class_<FilterBasicControlRegister>("FilterBasicControlRegister", "Class representing the Filter Basic Control register.")
		.def(init<MagneticMode, ExternalSensorMode, ExternalSensorMode, ExternalSensorMode, vec3f>())
		.def_readwrite("mag_mode", &FilterBasicControlRegister::magMode)
		.def_readwrite("ext_mag_mode", &FilterBasicControlRegister::extMagMode)
		.def_readwrite("ext_acc_mode", &FilterBasicControlRegister::extAccMode)
		.def_readwrite("ext_gyro_mode", &FilterBasicControlRegister::extGyroMode)
		.def_readwrite("gyro_limit", &FilterBasicControlRegister::gyroLimit)
		;
	class_<VpeBasicControlRegister>("VpeBasicControlRegister", "Class representing the VPE Basic Control register.")
		.def(init<VpeEnable, HeadingMode, VpeMode, VpeMode>())
		.def_readwrite("enable", &VpeBasicControlRegister::enable)
		.def_readwrite("heading_mode", &VpeBasicControlRegister::headingMode)
		.def_readwrite("filtering_mode", &VpeBasicControlRegister::filteringMode)
		.def_readwrite("tuning_mode", &VpeBasicControlRegister::tuningMode)
		;
	class_<VpeMagnetometerBasicTuningRegister>("VpeMagnetometerBasicTuningRegister", "Class representing the VPE Magnetometer Basic Tuning register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("base_tuning", &VpeMagnetometerBasicTuningRegister::baseTuning)
		.def_readwrite("adaptive_tuning", &VpeMagnetometerBasicTuningRegister::adaptiveTuning)
		.def_readwrite("adaptive_filtering", &VpeMagnetometerBasicTuningRegister::adaptiveFiltering)
		;
	class_<VpeMagnetometerAdvancedTuningRegister>("VpeMagnetometerAdvancedTuningRegister", "Class representing the VPE Magnetometer Advanced Tuning register.")
		.def(init<vec3f, vec3f, float, float, float>())
		.def_readwrite("min_filtering", &VpeMagnetometerAdvancedTuningRegister::minFiltering)
		.def_readwrite("max_filtering", &VpeMagnetometerAdvancedTuningRegister::maxFiltering)
		.def_readwrite("max_adapt_rate", &VpeMagnetometerAdvancedTuningRegister::maxAdaptRate)
		.def_readwrite("disturbance_window", &VpeMagnetometerAdvancedTuningRegister::disturbanceWindow)
		.def_readwrite("max_tuning", &VpeMagnetometerAdvancedTuningRegister::maxTuning)
		;
	class_<VpeAccelerometerBasicTuningRegister>("VpeAccelerometerBasicTuningRegister", "Class representing the VPE Accelerometer Basic Tuning register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("base_tuning", &VpeAccelerometerBasicTuningRegister::baseTuning)
		.def_readwrite("adaptive_tuning", &VpeAccelerometerBasicTuningRegister::adaptiveTuning)
		.def_readwrite("adaptive_filtering", &VpeAccelerometerBasicTuningRegister::adaptiveFiltering)
		;
	class_<VpeAccelerometerAdvancedTuningRegister>("VpeAccelerometerAdvancedTuningRegister", "Class representing the VPE Accelerometer Advanced Tuning register.")
		.def(init<vec3f, vec3f, float, float, float>())
		.def_readwrite("min_filtering", &VpeAccelerometerAdvancedTuningRegister::minFiltering)
		.def_readwrite("max_filtering", &VpeAccelerometerAdvancedTuningRegister::maxFiltering)
		.def_readwrite("max_adapt_rate", &VpeAccelerometerAdvancedTuningRegister::maxAdaptRate)
		.def_readwrite("disturbance_window", &VpeAccelerometerAdvancedTuningRegister::disturbanceWindow)
		.def_readwrite("max_tuning", &VpeAccelerometerAdvancedTuningRegister::maxTuning)
		;
	class_<VpeGryoBasicTuningRegister>("VpeGryoBasicTuningRegister", "Class representing the VPE Gryo Basic Tuning register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("angular_walk_variance", &VpeGryoBasicTuningRegister::angularWalkVariance)
		.def_readwrite("base_tuning", &VpeGryoBasicTuningRegister::baseTuning)
		.def_readwrite("adaptive_tuning", &VpeGryoBasicTuningRegister::adaptiveTuning)
		;
	class_<MagnetometerCalibrationControlRegister>("MagnetometerCalibrationControlRegister", "Class representing the Magnetometer Calibration Control register.")
		.def(init<HsiMode, HsiOutput, uint8_t>())
		.def_readwrite("hsi_mode", &MagnetometerCalibrationControlRegister::hsiMode)
		.def_readwrite("hsi_output", &MagnetometerCalibrationControlRegister::hsiOutput)
		.def_readwrite("converge_rate", &MagnetometerCalibrationControlRegister::convergeRate)
		;
	class_<CalculatedMagnetometerCalibrationRegister>("CalculatedMagnetometerCalibrationRegister", "Class representing the Calculated Magnetometer Calibration register.")
		.def(init<mat3f, vec3f>())
		.def_readwrite("c", &CalculatedMagnetometerCalibrationRegister::c)
		.def_readwrite("b", &CalculatedMagnetometerCalibrationRegister::b)
		;
	class_<VelocityCompensationControlRegister>("VelocityCompensationControlRegister", "Class representing the Velocity Compensation Control register.")
		.def(init<VelocityCompensationMode, float, float>())
		.def_readwrite("mode", &VelocityCompensationControlRegister::mode)
		.def_readwrite("velocity_tuning", &VelocityCompensationControlRegister::velocityTuning)
		.def_readwrite("rate_tuning", &VelocityCompensationControlRegister::rateTuning)
		;
	class_<VelocityCompensationStatusRegister>("VelocityCompensationStatusRegister", "Class representing the Velocity Compensation Status register.")
		.def(init<float, float, vec3f, vec3f>())
		.def_readwrite("x", &VelocityCompensationStatusRegister::x)
		.def_readwrite("x_dot", &VelocityCompensationStatusRegister::xDot)
		.def_readwrite("accel_offset", &VelocityCompensationStatusRegister::accelOffset)
		.def_readwrite("omega", &VelocityCompensationStatusRegister::omega)
		;
	class_<ImuMeasurementsRegister>("ImuMeasurementsRegister", "Class representing the IMU Measurements register.")
		.def(init<vec3f, vec3f, vec3f, float, float>())
		.def_readwrite("mag", &ImuMeasurementsRegister::mag)
		.def_readwrite("accel", &ImuMeasurementsRegister::accel)
		.def_readwrite("gyro", &ImuMeasurementsRegister::gyro)
		.def_readwrite("temp", &ImuMeasurementsRegister::temp)
		.def_readwrite("pressure", &ImuMeasurementsRegister::pressure)
		;
	class_<GpsConfigurationRegister>("GpsConfigurationRegister", "Class representing the GPS Configuration register.")
		.def(init<GpsMode, PpsSource>())
		.def_readwrite("mode", &GpsConfigurationRegister::mode)
		.def_readwrite("pps_source", &GpsConfigurationRegister::ppsSource)
		;
	class_<GpsSolutionLlaRegister>("GpsSolutionLlaRegister", "Class representing the GPS Solution - LLA register.")
		.def(init<double, uint16_t, GpsFix, uint8_t, vec3d, vec3f, vec3f, float, float>())
		.def_readwrite("time", &GpsSolutionLlaRegister::time)
		.def_readwrite("week", &GpsSolutionLlaRegister::week)
		.def_readwrite("gps_fix", &GpsSolutionLlaRegister::gpsFix)
		.def_readwrite("num_sats", &GpsSolutionLlaRegister::numSats)
		.def_readwrite("lla", &GpsSolutionLlaRegister::lla)
		.def_readwrite("ned_vel", &GpsSolutionLlaRegister::nedVel)
		.def_readwrite("ned_acc", &GpsSolutionLlaRegister::nedAcc)
		.def_readwrite("speed_acc", &GpsSolutionLlaRegister::speedAcc)
		.def_readwrite("time_acc", &GpsSolutionLlaRegister::timeAcc)
		;
	class_<GpsSolutionEcefRegister>("GpsSolutionEcefRegister", "Class representing the GPS Solution - ECEF register.")
		.def(init<double, uint16_t, GpsFix, uint8_t, vec3d, vec3f, vec3f, float, float>())
		.def_readwrite("tow", &GpsSolutionEcefRegister::tow)
		.def_readwrite("week", &GpsSolutionEcefRegister::week)
		.def_readwrite("gps_fix", &GpsSolutionEcefRegister::gpsFix)
		.def_readwrite("num_sats", &GpsSolutionEcefRegister::numSats)
		.def_readwrite("position", &GpsSolutionEcefRegister::position)
		.def_readwrite("velocity", &GpsSolutionEcefRegister::velocity)
		.def_readwrite("pos_acc", &GpsSolutionEcefRegister::posAcc)
		.def_readwrite("speed_acc", &GpsSolutionEcefRegister::speedAcc)
		.def_readwrite("time_acc", &GpsSolutionEcefRegister::timeAcc)
		;
	class_<InsSolutionLlaRegister>("InsSolutionLlaRegister", "Class representing the INS Solution - LLA register.")
		.def(init<double, uint16_t, uint16_t, vec3f, vec3d, vec3f, float, float, float>())
		.def_readwrite("time", &InsSolutionLlaRegister::time)
		.def_readwrite("week", &InsSolutionLlaRegister::week)
		.def_readwrite("status", &InsSolutionLlaRegister::status)
		.def_readwrite("yaw_pitch_roll", &InsSolutionLlaRegister::yawPitchRoll)
		.def_readwrite("position", &InsSolutionLlaRegister::position)
		.def_readwrite("ned_vel", &InsSolutionLlaRegister::nedVel)
		.def_readwrite("att_uncertainty", &InsSolutionLlaRegister::attUncertainty)
		.def_readwrite("pos_uncertainty", &InsSolutionLlaRegister::posUncertainty)
		.def_readwrite("vel_uncertainty", &InsSolutionLlaRegister::velUncertainty)
		;
	class_<InsSolutionEcefRegister>("InsSolutionEcefRegister", "Class representing the INS Solution - ECEF register.")
		.def(init<double, uint16_t, uint16_t, vec3f, vec3d, vec3f, float, float, float>())
		.def_readwrite("time", &InsSolutionEcefRegister::time)
		.def_readwrite("week", &InsSolutionEcefRegister::week)
		.def_readwrite("status", &InsSolutionEcefRegister::status)
		.def_readwrite("yaw_pitch_roll", &InsSolutionEcefRegister::yawPitchRoll)
		.def_readwrite("position", &InsSolutionEcefRegister::position)
		.def_readwrite("velocity", &InsSolutionEcefRegister::velocity)
		.def_readwrite("att_uncertainty", &InsSolutionEcefRegister::attUncertainty)
		.def_readwrite("pos_uncertainty", &InsSolutionEcefRegister::posUncertainty)
		.def_readwrite("vel_uncertainty", &InsSolutionEcefRegister::velUncertainty)
		;
	class_<InsBasicConfigurationRegisterVn200>("InsBasicConfigurationRegisterVn200", "Class representing the INS Basic Configuration register for a VN-200 sensor.")
		.def(init<Scenario, uint8_t>())
		.def_readwrite("scenario", &InsBasicConfigurationRegisterVn200::scenario)
		.def_readwrite("ahrs_aiding", &InsBasicConfigurationRegisterVn200::ahrsAiding)
		;
	class_<InsBasicConfigurationRegisterVn300>("InsBasicConfigurationRegisterVn300", "Class representing the INS Basic Configuration register for a VN-300 sensor.")
		.def(init<Scenario, uint8_t, uint8_t>())
		.def_readwrite("scenario", &InsBasicConfigurationRegisterVn300::scenario)
		.def_readwrite("ahrs_aiding", &InsBasicConfigurationRegisterVn300::ahrsAiding)
		.def_readwrite("est_baseline", &InsBasicConfigurationRegisterVn300::estBaseline)
		;
	class_<InsAdvancedConfigurationRegister>("InsAdvancedConfigurationRegister", "Class representing the INS Advanced Configuration register.")
		.def_readwrite("use_mag", &InsAdvancedConfigurationRegister::useMag)
		.def_readwrite("use_pres", &InsAdvancedConfigurationRegister::usePres)
		.def_readwrite("pos_att", &InsAdvancedConfigurationRegister::posAtt)
		.def_readwrite("vel_att", &InsAdvancedConfigurationRegister::velAtt)
		.def_readwrite("vel_bias", &InsAdvancedConfigurationRegister::velBias)
		.def_readwrite("use_foam", &InsAdvancedConfigurationRegister::useFoam)
		.def_readwrite("gps_cov_type", &InsAdvancedConfigurationRegister::gpsCovType)
		.def_readwrite("vel_count", &InsAdvancedConfigurationRegister::velCount)
		.def_readwrite("vel_init", &InsAdvancedConfigurationRegister::velInit)
		.def_readwrite("move_origin", &InsAdvancedConfigurationRegister::moveOrigin)
		.def_readwrite("gps_timeout", &InsAdvancedConfigurationRegister::gpsTimeout)
		.def_readwrite("delta_limit_pos", &InsAdvancedConfigurationRegister::deltaLimitPos)
		.def_readwrite("delta_limit_vel", &InsAdvancedConfigurationRegister::deltaLimitVel)
		.def_readwrite("min_pos_uncertainty", &InsAdvancedConfigurationRegister::minPosUncertainty)
		.def_readwrite("min_vel_uncertainty", &InsAdvancedConfigurationRegister::minVelUncertainty)
		;
	class_<InsStateLlaRegister>("InsStateLlaRegister", "Class representing the INS State - LLA register.")
		.def(init<vec3f, vec3d, vec3f, vec3f, vec3f>())
		.def_readwrite("yaw_pitch_roll", &InsStateLlaRegister::yawPitchRoll)
		.def_readwrite("position", &InsStateLlaRegister::position)
		.def_readwrite("velocity", &InsStateLlaRegister::velocity)
		.def_readwrite("accel", &InsStateLlaRegister::accel)
		.def_readwrite("angular_rate", &InsStateLlaRegister::angularRate)
		;
	class_<InsStateEcefRegister>("InsStateEcefRegister", "Class representing the INS State - ECEF register.")
		.def(init<vec3f, vec3d, vec3f, vec3f, vec3f>())
		.def_readwrite("yaw_pitch_roll", &InsStateEcefRegister::yawPitchRoll)
		.def_readwrite("position", &InsStateEcefRegister::position)
		.def_readwrite("velocity", &InsStateEcefRegister::velocity)
		.def_readwrite("accel", &InsStateEcefRegister::accel)
		.def_readwrite("angular_rate", &InsStateEcefRegister::angularRate)
		;
	class_<StartupFilterBiasEstimateRegister>("StartupFilterBiasEstimateRegister", "Class representing the Startup Filter Bias Estimate register.")
		.def(init<vec3f, vec3f, float>())
		.def_readwrite("gyro_bias", &StartupFilterBiasEstimateRegister::gyroBias)
		.def_readwrite("accel_bias", &StartupFilterBiasEstimateRegister::accelBias)
		.def_readwrite("pressure_bias", &StartupFilterBiasEstimateRegister::pressureBias)
		;
	class_<DeltaThetaAndDeltaVelocityRegister>("DeltaThetaAndDeltaVelocityRegister", "Class representing the Delta Theta and Delta Velocity register.")
		.def(init<float, vec3f, vec3f>())
		.def_readwrite("delta_time", &DeltaThetaAndDeltaVelocityRegister::deltaTime)
		.def_readwrite("delta_theta", &DeltaThetaAndDeltaVelocityRegister::deltaTheta)
		.def_readwrite("delta_velocity", &DeltaThetaAndDeltaVelocityRegister::deltaVelocity)
		;
	class_<DeltaThetaAndDeltaVelocityConfigurationRegister>("DeltaThetaAndDeltaVelocityConfigurationRegister", "Class representing the Delta Theta and Delta Velocity Configuration register.")
		.def(init<IntegrationFrame, CompensationMode, CompensationMode>())
		.def_readwrite("integration_frame", &DeltaThetaAndDeltaVelocityConfigurationRegister::integrationFrame)
		.def_readwrite("gyro_compensation", &DeltaThetaAndDeltaVelocityConfigurationRegister::gyroCompensation)
		.def_readwrite("accel_compensation", &DeltaThetaAndDeltaVelocityConfigurationRegister::accelCompensation)
		;
	class_<ReferenceVectorConfigurationRegister>("ReferenceVectorConfigurationRegister", "Class representing the Reference Vector Configuration register.")
		.def(init<uint8_t, uint8_t, uint32_t, float, vec3d>())
		.def_readwrite("use_mag_model", &ReferenceVectorConfigurationRegister::useMagModel)
		.def_readwrite("use_gravity_model", &ReferenceVectorConfigurationRegister::useGravityModel)
		.def_readwrite("recalc_threshold", &ReferenceVectorConfigurationRegister::recalcThreshold)
		.def_readwrite("year", &ReferenceVectorConfigurationRegister::year)
		.def_readwrite("position", &ReferenceVectorConfigurationRegister::position)
		;
	class_<GyroCompensationRegister>("GyroCompensationRegister", "Class representing the Gyro Compensation register.")
		.def(init<mat3f, vec3f>())
		.def_readwrite("c", &GyroCompensationRegister::c)
		.def_readwrite("b", &GyroCompensationRegister::b)
		;
	class_<ImuFilteringConfigurationRegister>("ImuFilteringConfigurationRegister", "Class representing the IMU Filtering Configuration register.")
		.def(init<uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, FilterMode, FilterMode, FilterMode, FilterMode, FilterMode>())
		.def_readwrite("mag_window_size", &ImuFilteringConfigurationRegister::magWindowSize)
		.def_readwrite("accel_window_size", &ImuFilteringConfigurationRegister::accelWindowSize)
		.def_readwrite("gyro_window_size", &ImuFilteringConfigurationRegister::gyroWindowSize)
		.def_readwrite("temp_window_size", &ImuFilteringConfigurationRegister::tempWindowSize)
		.def_readwrite("pres_window_size", &ImuFilteringConfigurationRegister::presWindowSize)
		.def_readwrite("mag_filter_mode", &ImuFilteringConfigurationRegister::magFilterMode)
		.def_readwrite("accel_filter_mode", &ImuFilteringConfigurationRegister::accelFilterMode)
		.def_readwrite("gyro_filter_mode", &ImuFilteringConfigurationRegister::gyroFilterMode)
		.def_readwrite("temp_filter_mode", &ImuFilteringConfigurationRegister::tempFilterMode)
		.def_readwrite("pres_filter_mode", &ImuFilteringConfigurationRegister::presFilterMode)
		;
	class_<GpsCompassBaselineRegister>("GpsCompassBaselineRegister", "Class representing the GPS Compass Baseline register.")
		.def(init<vec3f, vec3f>())
		.def_readwrite("position", &GpsCompassBaselineRegister::position)
		.def_readwrite("uncertainty", &GpsCompassBaselineRegister::uncertainty)
		;
	class_<GpsCompassEstimatedBaselineRegister>("GpsCompassEstimatedBaselineRegister", "Class representing the GPS Compass Estimated Baseline register.")
		.def(init<uint8_t, uint16_t, vec3f, vec3f>())
		.def_readwrite("est_baseline_used", &GpsCompassEstimatedBaselineRegister::estBaselineUsed)
		.def_readwrite("num_meas", &GpsCompassEstimatedBaselineRegister::numMeas)
		.def_readwrite("position", &GpsCompassEstimatedBaselineRegister::position)
		.def_readwrite("uncertainty", &GpsCompassEstimatedBaselineRegister::uncertainty)
		;
	class_<ImuRateConfigurationRegister>("ImuRateConfigurationRegister", "Class representing the IMU Rate Configuration register.")
		.def(init<uint16_t, uint16_t, float, float>())
		.def_readwrite("imu_rate", &ImuRateConfigurationRegister::imuRate)
		.def_readwrite("nav_divisor", &ImuRateConfigurationRegister::navDivisor)
		.def_readwrite("filter_target_rate", &ImuRateConfigurationRegister::filterTargetRate)
		.def_readwrite("filter_min_rate", &ImuRateConfigurationRegister::filterMinRate)
		;
	class_<YawPitchRollTrueBodyAccelerationAndAngularRatesRegister>("YawPitchRollTrueBodyAccelerationAndAngularRatesRegister", "Class representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("yaw_pitch_roll", &YawPitchRollTrueBodyAccelerationAndAngularRatesRegister::yawPitchRoll)
		.def_readwrite("body_accel", &YawPitchRollTrueBodyAccelerationAndAngularRatesRegister::bodyAccel)
		.def_readwrite("gyro", &YawPitchRollTrueBodyAccelerationAndAngularRatesRegister::gyro)
		;
	class_<YawPitchRollTrueInertialAccelerationAndAngularRatesRegister>("YawPitchRollTrueInertialAccelerationAndAngularRatesRegister", "Class representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.")
		.def(init<vec3f, vec3f, vec3f>())
		.def_readwrite("yaw_pitch_roll", &YawPitchRollTrueInertialAccelerationAndAngularRatesRegister::yawPitchRoll)
		.def_readwrite("inertial_accel", &YawPitchRollTrueInertialAccelerationAndAngularRatesRegister::inertialAccel)
		.def_readwrite("gyro", &YawPitchRollTrueInertialAccelerationAndAngularRatesRegister::gyro)
		;

	class_<VnSensor, boost::noncopyable>("VnSensor", "Base class for VectorNav sensors.")
		.def("connect",
			static_cast<void (VnSensor::*)(const string&, uint32_t)>(&VnSensor::connect),
			args("port_name", "baud_rate"),
			"Connects to a VectorNav sensor with the provided connection parameters.")
		//.def("connect",
		//	static_cast<void(VnSensor::*)(common::ISimplePort*)>(&VnSensor::connect))
		#if !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1
		.def("halt", &VnSensor::stopRequest)
		.def("shutdown", &VnSensor::shutdownRequest)
		.def("isStopped", &VnSensor::threadStopped)
		.def("resume", &VnSensor::goRequest)
		.def("unregister", &VnSensor::unregisterListners)
		#endif
		.def("disconnect",
			&VnSensor::disconnect,
			"Disconnects from the sensor.")
		.def("baudrate",
			&VnSensor::baudrate,
			"Returns the connected baudrate of the serial port.")
		//.def("changeBaudrate", &VnSensor::changeBaudrate, "Changes the serial ports connected baudrate.")
		.add_property("response_timeout_ms", &VnSensor::responseTimeoutMs, &VnSensor::setResponseTimeoutMs)
		.def("send",
			&VnSensor::send,
			send_overloads(args("toSend", "waitForReply", "errorDetectionMode"),
			"Writes a raw data string to the sensor, normally appending an appropriate error detection checksum."))
		.def("write_settings",
			&VnSensor::writeSettings,
			writeSettings_overloads(args("waitForReply"),
			"Issues a Write Settings command to the VectorNav Sensor."))
		.def("restore_factory_settings",
			&VnSensor::restoreFactorySettings,
			restoreFactorySettings_overloads(args("waitForReply"),
			"Issues a Restore Factory Settings command to the VectorNav sensor."))
		.def("reset",
			&VnSensor::reset,
			reset_overloads(args("waitForReply"),
			"Issues a Reset command to the VectorNav sensor."))
		.def("transaction",
			&VnSensor::transaction,
			"Sends a command to the sensor, computing and attaching checksum and line termination charaters as needed, and returns the response.")
		.def("supported_baudrates",
			&VnSensor::supportedBaudrates,
			"The list of baudrates supported by VectorNav sensors.")
		.def("tare",
			&VnSensor::tare,
			"Sends a command to tare the sensor")
		.def("set_gyro_bias",
			&VnSensor::setGyroBias,
			"Sends a command to set the gyro's bias")
		.def("magnetic_disturbance_present",
			&VnSensor::magneticDisturbancePresent,
			"Sends a command to inform the sensor that a magnetic disturbance is present.")
		.def("acceleration_disturbance_present",
			&VnSensor::accelerationDisturbancePresent,
			"Sends a command to inform the sensor that an acceleration disturbance is present.")

		#if PL150
		.def_readonly("event_async_packet_received", &VnSensor::eventAsyncPacketReceived)
		#else
		// TODO: PL-150 would like to remove this low-level callback mechanism with a more
		//       implemented Event structure.
//		.def("register_async_packet_received_handler",
//			static_cast<PyObject* (VnSensor::*)(PyObject*)>(&VnSensor::registerAsyncPacketReceivedHandler))
		//		.def("registerAsyncPacketReceivedHandler", registerAsyncPacketReceivedHandlerPyObject)
		//		.def("register_raw_data_received_handler", registerRawDataReceivedHandlerPyObject)
		//.def("register_async_packet_received_handler",
		//	&vnSensorRegisterAsyncPacketReceivedHandler)
		#endif

		.def("read_serial_baud_rate",
			static_cast<uint32_t (VnSensor::*)()>(&VnSensor::readSerialBaudRate),
			"Reads the Serial Baud Rate register.")
		.def("read_serial_baud_rate",
			static_cast<uint32_t (VnSensor::*)(uint8_t)>(&VnSensor::readSerialBaudRate),
			"Reads the Serial Baud Rate register for the specified port ID.")
		.def("write_serial_baud_rate",
			static_cast<void (VnSensor::*)(const uint32_t&, bool)>(&VnSensor::writeSerialBaudRate),
			writeSerialBaudRate_overloads(args("sensor", "baudrate", "waitForReply"),
			"Writes the Serial Baud Rate register."))
		.def("write_serial_baud_rate",
			static_cast<void (VnSensor::*)(const uint32_t&, uint8_t, bool)>(&VnSensor::writeSerialBaudRate),
			writeSerialBaudRate_overloads2(args("sensor", "baudrate", "waitForReply"),
			"Writes the Serial Baud Rate register for the specified port ID."))
		.def("read_async_data_output_type",
			static_cast<AsciiAsync (VnSensor::*)()>(&VnSensor::readAsyncDataOutputType),
			"Reads the Async Data Output Type register.")
		.def("read_async_data_output_type",
			static_cast<AsciiAsync(VnSensor::*)(uint8_t)>(&VnSensor::readAsyncDataOutputType),
			"Reads the Async Data Output Type register for the specified port ID.")
		.def("write_async_data_output_type",
			static_cast<void (VnSensor::*)(AsciiAsync, bool)>(&VnSensor::writeAsyncDataOutputType),
			writeAsyncDataOutputType_overloads(args("sensor", "ador", "waitForReply"),
			"Writes the Async Data Output Type register."))
		.def("write_async_data_output_type",
			static_cast<void (VnSensor::*)(AsciiAsync, uint8_t, bool)>(&VnSensor::writeAsyncDataOutputType),
			writeAsyncDataOutputType_overloads2(args("sensor", "ador", "waitForReply"),
			"Writes the Async Data Output Type register."))
		.def("read_async_data_output_frequency",
			static_cast<uint32_t (VnSensor::*)()>(&VnSensor::readAsyncDataOutputFrequency),
			"Reads the Async Data Output Frequency register.")
		.def("read_async_data_output_frequency",
			static_cast<uint32_t (VnSensor::*)(uint8_t)>(&VnSensor::readAsyncDataOutputFrequency),
			"Reads the Async Data Output Frequency register for the specified port ID.")
		.def("write_async_data_output_frequency",
			static_cast<void (VnSensor::*)(const uint32_t&, bool)>(&VnSensor::writeAsyncDataOutputFrequency),
			writeAsyncDataOutputFrequency_overloads(args("sensor", "adof", "waitForReply"),
			"Writes the Async Data Output Frequency register."))
		.def("write_async_data_output_frequency",
			static_cast<void (VnSensor::*)(const uint32_t&, uint8_t, bool)>(&VnSensor::writeAsyncDataOutputFrequency),
			writeAsyncDataOutputFrequency_overloads2(args("sensor", "adof", "waitForReply"),
			"Writes the Async Data Output Frequency register for the specified port ID."))
		.def("write_binary_output_1",
			&VnSensor::writeBinaryOutput1,
			writeBinaryOutput1_overloads("Writes the Binary Output 1 register."))
		.def("write_binary_output_2",
			&VnSensor::writeBinaryOutput2,
			writeBinaryOutput2_overloads("Writes the Binary Output 2 register."))
		.def("write_binary_output_3",
			&VnSensor::writeBinaryOutput3,
			writeBinaryOutput3_overloads("Writes the Binary Output 3 register."))
		.def("read_binary_output_1",
			&VnSensor::readBinaryOutput1,
			"Reads the Binary Output 1 register.")
		.def("read_binary_output_2",
			&VnSensor::readBinaryOutput2,
			"Reads the Binary Output 2 register.")
		.def("read_binary_output_3",
			&VnSensor::readBinaryOutput3,
			"Reads the Binary Output 3 register.")
		.def("write_ins_basic_configuration_vn200",
			static_cast<void (VnSensor::*)(InsBasicConfigurationRegisterVn200&, bool)>(&VnSensor::writeInsBasicConfigurationVn200),
			writeInsBasicConfigurationVn200_overloads(args("sensor", "fields", "waitForReply"),
			"Writes the INS Basic Configuration register for a VN-200 sensor."))
		.def("write_ins_basic_configuration_vn200",
			static_cast<void (VnSensor::*)(Scenario, const uint8_t&, bool)>(&VnSensor::writeInsBasicConfigurationVn200),
			writeInsBasicConfigurationVn200NoStruct_overloads())
		.def("write_ins_basic_configuration_vn300",
			static_cast<void (VnSensor::*)(InsBasicConfigurationRegisterVn300&, bool)>(&VnSensor::writeInsBasicConfigurationVn300),
			writeInsBasicConfigurationVn300_overloads(args("sensor", "fields", "waitForReply"),
			"Writes the INS Basic Configuration register for a VN-300 sensor."))
		.def("write_ins_basic_configuration_vn300",
			static_cast<void (VnSensor::*)(Scenario, const uint8_t&, const uint8_t&, bool)>(&VnSensor::writeInsBasicConfigurationVn300),
			writeInsBasicConfigurationVn300NoStruct_overloads())
		.def("read_user_tag", &VnSensor::readUserTag, "Reads the User Tag register.")
		.def("write_user_tag", &VnSensor::writeUserTag, writeUserTag_overloads(args("sensor", "tag", "waitForReply"), "Writes the User Tag register."))
		.def("read_model_number", &VnSensor::readModelNumber, "Reads the Model Number register.")
		.def("read_hardware_revision", &VnSensor::readHardwareRevision, "Reads the Hardware Revision register.")
		.def("read_serial_number", &VnSensor::readSerialNumber, "Reads the Serial Number register.")
		.def("read_firmware_version", &VnSensor::readFirmwareVersion, "Reads the Firmware Version register.")
		.def("read_yaw_pitch_roll", &VnSensor::readYawPitchRoll, "Reads the Yaw Pitch Roll register.")
		.def("read_attitude_quaternion", &VnSensor::readAttitudeQuaternion, "Reads the Attitude Quaternion register.")
		.def("read_quaternion_magnetic_acceleration_and_angular_rates", &VnSensor::readQuaternionMagneticAccelerationAndAngularRates, "Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.")
		.def("read_magnetic_measurements", &VnSensor::readMagneticMeasurements, "Reads the Magnetic Measurements register.")
		.def("read_acceleration_measurements", &VnSensor::readAccelerationMeasurements, "Reads the Acceleration Measurements register.")
		.def("read_angular_rate_measurements", &VnSensor::readAngularRateMeasurements, "Reads the Angular Rate Measurements register.")
		.def("read_magnetic_acceleration_and_angular_rates", &VnSensor::readMagneticAccelerationAndAngularRates, "Reads the Magnetic, Acceleration and Angular Rates register.")
		.def("read_magnetic_and_gravity_reference_vectors", &VnSensor::readMagneticAndGravityReferenceVectors, "Reads the Magnetic and Gravity Reference Vectors register.")
		.def("write_magnetic_and_gravity_reference_vectors",
			static_cast<void (VnSensor::*)(MagneticAndGravityReferenceVectorsRegister&, bool)>(&VnSensor::writeMagneticAndGravityReferenceVectors),
			writeMagneticAndGravityReferenceVectors_overloads(args("sensor", "fields", "waitForReply"), "Write the Magnetic and Gravity Reference Vectors register."))
		.def("write_magnetic_and_gravity_reference_vectors",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, bool)>(&VnSensor::writeMagneticAndGravityReferenceVectors),
			writeMagneticAndGravityReferenceVectorsNoStruct_overloads(args("sensor", "magRef", "accRef", "waitForReply"), "Writes the Magnetic and Gravity Reference Vectors register."))
		.def("read_filter_measurements_variance_parameters", &VnSensor::readFilterMeasurementsVarianceParameters, "Reads the Filter Measurements Variance Parameters register.")
		.def("write_filter_measurements_variance_parameters",
			static_cast<void (VnSensor::*)(FilterMeasurementsVarianceParametersRegister&, bool)>(&VnSensor::writeFilterMeasurementsVarianceParameters),
			writeFilterMeasurementsVarianceParameters_overloads(args("sensor", "fields", "waitForReply"), "Write the Filter Measurements Variance Parameters register."))
		.def("write_filter_measurements_variance_parameters",
			static_cast<void (VnSensor::*)(const float&, const vec3f&, const vec3f&, const vec3f&, bool)>(&VnSensor::writeFilterMeasurementsVarianceParameters),
			writeFilterMeasurementsVarianceParametersNoStruct_overloads(args("sensor", "angularWalkVariance", "angularRateVariance", "magneticVariance", "accelerationVariance", "waitForReply"), "Writes the Filter Measurements Variance Parameters register."))
		.def("read_magnetometer_compensation", &VnSensor::readMagnetometerCompensation, "Reads the Magnetometer Compensation register.")
		.def("write_magnetometer_compensation",
			static_cast<void (VnSensor::*)(MagnetometerCompensationRegister&, bool)>(&VnSensor::writeMagnetometerCompensation),
			writeMagnetometerCompensation_overloads(args("sensor", "fields", "waitForReply"), "Write the Magnetometer Compensation register."))
		.def("write_magnetometer_compensation",
			static_cast<void (VnSensor::*)(const mat3f&, const vec3f&, bool)>(&VnSensor::writeMagnetometerCompensation),
			writeMagnetometerCompensationNoStruct_overloads(args("sensor", "c", "b", "waitForReply"), "Writes the Magnetometer Compensation register."))
		.def("read_filter_active_tuning_parameters", &VnSensor::readFilterActiveTuningParameters, "Reads the Filter Active Tuning Parameters register.")
		.def("write_filter_active_tuning_parameters",
			static_cast<void (VnSensor::*)(FilterActiveTuningParametersRegister&, bool)>(&VnSensor::writeFilterActiveTuningParameters),
			writeFilterActiveTuningParameters_overloads(args("sensor", "fields", "waitForReply"), "Write the Filter Active Tuning Parameters register."))
		.def("write_filter_active_tuning_parameters",
			static_cast<void (VnSensor::*)(const float&, const float&, const float&, const float&, bool)>(&VnSensor::writeFilterActiveTuningParameters),
			writeFilterActiveTuningParametersNoStruct_overloads(args("sensor", "magneticDisturbanceGain", "accelerationDisturbanceGain", "magneticDisturbanceMemory", "accelerationDisturbanceMemory", "waitForReply"), "Writes the Filter Active Tuning Parameters register."))
		.def("read_acceleration_compensation", &VnSensor::readAccelerationCompensation, "Reads the Acceleration Compensation register.")
		.def("write_acceleration_compensation",
			static_cast<void (VnSensor::*)(AccelerationCompensationRegister&, bool)>(&VnSensor::writeAccelerationCompensation),
			writeAccelerationCompensation_overloads(args("sensor", "fields", "waitForReply"), "Write the Acceleration Compensation register."))
		.def("write_acceleration_compensation",
			static_cast<void (VnSensor::*)(const mat3f&, const vec3f&, bool)>(&VnSensor::writeAccelerationCompensation),
			writeAccelerationCompensationNoStruct_overloads(args("sensor", "c", "b", "waitForReply"), "Writes the Acceleration Compensation register."))
		.def("read_reference_frame_rotation", &VnSensor::readReferenceFrameRotation, "Reads the Reference Frame Rotation register.")
		.def("write_reference_frame_rotation", &VnSensor::writeReferenceFrameRotation, writeReferenceFrameRotation_overloads(args("sensor", "c", "waitForReply"), "Writes the Reference Frame Rotation register."))
		.def("read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates", &VnSensor::readYawPitchRollMagneticAccelerationAndAngularRates, "Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.")
		.def("read_communication_protocol_control", &VnSensor::readCommunicationProtocolControl, "Reads the Communication Protocol Control register.")
		.def("write_communication_protocol_control",
			static_cast<void (VnSensor::*)(CommunicationProtocolControlRegister&, bool)>(&VnSensor::writeCommunicationProtocolControl),
			writeCommunicationProtocolControl_overloads(args("sensor", "fields", "waitForReply"), "Write the Communication Protocol Control register."))
		.def("write_communication_protocol_control",
			static_cast<void (VnSensor::*)(CountMode, StatusMode, CountMode, StatusMode, ChecksumMode, ChecksumMode, ErrorMode, bool)>(&VnSensor::writeCommunicationProtocolControl),
			writeCommunicationProtocolControlNoStruct_overloads(args("sensor", "serialCount", "serialStatus", "spiCount", "spiStatus", "serialChecksum", "spiChecksum", "errorMode", "waitForReply"), "Writes the Communication Protocol Control register."))
		.def("read_synchronization_control", &VnSensor::readSynchronizationControl, "Reads the Synchronization Control register.")
		.def("write_synchronization_control",
			static_cast<void (VnSensor::*)(SynchronizationControlRegister&, bool)>(&VnSensor::writeSynchronizationControl),
			writeSynchronizationControl_overloads(args("sensor", "fields", "waitForReply"), "Write the Synchronization Control register."))
		.def("write_synchronization_control",
			static_cast<void (VnSensor::*)(SyncInMode, SyncInEdge, const uint16_t&, SyncOutMode, SyncOutPolarity, const uint16_t&, const uint32_t&, bool)>(&VnSensor::writeSynchronizationControl),
			writeSynchronizationControlNoStruct_overloads(args("sensor", "syncInMode", "syncInEdge", "syncInSkipFactor", "syncOutMode", "syncOutPolarity", "syncOutSkipFactor", "syncOutPulseWidth", "waitForReply"), "Writes the Synchronization Control register."))
		.def("read_synchronization_status", &VnSensor::readSynchronizationStatus, "Reads the Synchronization Status register.")
		.def("write_synchronization_status",
			static_cast<void (VnSensor::*)(SynchronizationStatusRegister&, bool)>(&VnSensor::writeSynchronizationStatus),
			writeSynchronizationStatus_overloads(args("sensor", "fields", "waitForReply"), "Write the Synchronization Status register."))
		.def("write_synchronization_status",
			static_cast<void (VnSensor::*)(const uint32_t&, const uint32_t&, const uint32_t&, bool)>(&VnSensor::writeSynchronizationStatus),
			writeSynchronizationStatusNoStruct_overloads(args("sensor", "syncInCount", "syncInTime", "syncOutCount", "waitForReply"), "Writes the Synchronization Status register."))
		.def("read_filter_basic_control", &VnSensor::readFilterBasicControl, "Reads the Filter Basic Control register.")
		.def("write_filter_basic_control",
			static_cast<void (VnSensor::*)(FilterBasicControlRegister&, bool)>(&VnSensor::writeFilterBasicControl),
			writeFilterBasicControl_overloads(args("sensor", "fields", "waitForReply"), "Write the Filter Basic Control register."))
		.def("write_filter_basic_control",
			static_cast<void (VnSensor::*)(MagneticMode, ExternalSensorMode, ExternalSensorMode, ExternalSensorMode, const vec3f&, bool)>(&VnSensor::writeFilterBasicControl),
			writeFilterBasicControlNoStruct_overloads(args("sensor", "magMode", "extMagMode", "extAccMode", "extGyroMode", "gyroLimit", "waitForReply"), "Writes the Filter Basic Control register."))
		.def("read_vpe_basic_control", &VnSensor::readVpeBasicControl, "Reads the VPE Basic Control register.")
		.def("write_vpe_basic_control",
			static_cast<void (VnSensor::*)(VpeBasicControlRegister&, bool)>(&VnSensor::writeVpeBasicControl),
			writeVpeBasicControl_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Basic Control register."))
		.def("write_vpe_basic_control",
			static_cast<void (VnSensor::*)(VpeEnable, HeadingMode, VpeMode, VpeMode, bool)>(&VnSensor::writeVpeBasicControl),
			writeVpeBasicControlNoStruct_overloads(args("sensor", "enable", "headingMode", "filteringMode", "tuningMode", "waitForReply"), "Writes the VPE Basic Control register."))
		.def("read_vpe_magnetometer_basic_tuning", &VnSensor::readVpeMagnetometerBasicTuning, "Reads the VPE Magnetometer Basic Tuning register.")
		.def("write_vpe_magnetometer_basic_tuning",
			static_cast<void (VnSensor::*)(VpeMagnetometerBasicTuningRegister&, bool)>(&VnSensor::writeVpeMagnetometerBasicTuning),
			writeVpeMagnetometerBasicTuning_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Magnetometer Basic Tuning register."))
		.def("write_vpe_magnetometer_basic_tuning",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const vec3f&, bool)>(&VnSensor::writeVpeMagnetometerBasicTuning),
			writeVpeMagnetometerBasicTuningNoStruct_overloads(args("sensor", "baseTuning", "adaptiveTuning", "adaptiveFiltering", "waitForReply"), "Writes the VPE Magnetometer Basic Tuning register."))
		.def("read_vpe_magnetometer_advanced_tuning", &VnSensor::readVpeMagnetometerAdvancedTuning, "Reads the VPE Magnetometer Advanced Tuning register.")
		.def("write_vpe_magnetometer_advanced_tuning",
			static_cast<void (VnSensor::*)(VpeMagnetometerAdvancedTuningRegister&, bool)>(&VnSensor::writeVpeMagnetometerAdvancedTuning),
			writeVpeMagnetometerAdvancedTuning_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Magnetometer Advanced Tuning register."))
		.def("write_vpe_magnetometer_advanced_tuning",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const float&, const float&, const float&, bool)>(&VnSensor::writeVpeMagnetometerAdvancedTuning),
			writeVpeMagnetometerAdvancedTuningNoStruct_overloads(args("sensor", "minFiltering", "maxFiltering", "maxAdaptRate", "disturbanceWindow", "maxTuning", "waitForReply"), "Writes the VPE Magnetometer Advanced Tuning register."))
		.def("read_vpe_accelerometer_basic_tuning", &VnSensor::readVpeAccelerometerBasicTuning, "Reads the VPE Accelerometer Basic Tuning register.")
		.def("write_vpe_accelerometer_basic_tuning",
			static_cast<void (VnSensor::*)(VpeAccelerometerBasicTuningRegister&, bool)>(&VnSensor::writeVpeAccelerometerBasicTuning),
			writeVpeAccelerometerBasicTuning_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Accelerometer Basic Tuning register."))
		.def("write_vpe_accelerometer_basic_tuning",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const vec3f&, bool)>(&VnSensor::writeVpeAccelerometerBasicTuning),
			writeVpeAccelerometerBasicTuningNoStruct_overloads(args("sensor", "baseTuning", "adaptiveTuning", "adaptiveFiltering", "waitForReply"), "Writes the VPE Accelerometer Basic Tuning register."))
		.def("read_vpe_accelerometer_advanced_tuning", &VnSensor::readVpeAccelerometerAdvancedTuning, "Reads the VPE Accelerometer Advanced Tuning register.")
		.def("write_vpe_accelerometer_advanced_tuning",
			static_cast<void (VnSensor::*)(VpeAccelerometerAdvancedTuningRegister&, bool)>(&VnSensor::writeVpeAccelerometerAdvancedTuning),
			writeVpeAccelerometerAdvancedTuning_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Accelerometer Advanced Tuning register."))
		.def("write_vpe_accelerometer_advanced_tuning",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const float&, const float&, const float&, bool)>(&VnSensor::writeVpeAccelerometerAdvancedTuning),
			writeVpeAccelerometerAdvancedTuningNoStruct_overloads(args("sensor", "minFiltering", "maxFiltering", "maxAdaptRate", "disturbanceWindow", "maxTuning", "waitForReply"), "Writes the VPE Accelerometer Advanced Tuning register."))
		.def("read_vpe_gryo_basic_tuning", &VnSensor::readVpeGryoBasicTuning, "Reads the VPE Gryo Basic Tuning register.")
		.def("write_vpe_gryo_basic_tuning",
			static_cast<void (VnSensor::*)(VpeGryoBasicTuningRegister&, bool)>(&VnSensor::writeVpeGryoBasicTuning),
			writeVpeGryoBasicTuning_overloads(args("sensor", "fields", "waitForReply"), "Write the VPE Gryo Basic Tuning register."))
		.def("write_vpe_gryo_basic_tuning",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const vec3f&, bool)>(&VnSensor::writeVpeGryoBasicTuning),
			writeVpeGryoBasicTuningNoStruct_overloads(args("sensor", "angularWalkVariance", "baseTuning", "adaptiveTuning", "waitForReply"), "Writes the VPE Gryo Basic Tuning register."))
		.def("read_filter_startup_gyro_bias", &VnSensor::readFilterStartupGyroBias, "Reads the Filter Startup Gyro Bias register.")
		.def("write_filter_startup_gyro_bias", &VnSensor::writeFilterStartupGyroBias, writeFilterStartupGyroBias_overloads(args("sensor", "bias", "waitForReply"), "Writes the Filter Startup Gyro Bias register."))
		.def("read_magnetometer_calibration_control", &VnSensor::readMagnetometerCalibrationControl, "Reads the Magnetometer Calibration Control register.")
		.def("write_magnetometer_calibration_control",
			static_cast<void (VnSensor::*)(MagnetometerCalibrationControlRegister&, bool)>(&VnSensor::writeMagnetometerCalibrationControl),
			writeMagnetometerCalibrationControl_overloads(args("sensor", "fields", "waitForReply"), "Write the Magnetometer Calibration Control register."))
		.def("write_magnetometer_calibration_control",
			static_cast<void (VnSensor::*)(HsiMode, HsiOutput, const uint8_t&, bool)>(&VnSensor::writeMagnetometerCalibrationControl),
			writeMagnetometerCalibrationControlNoStruct_overloads(args("sensor", "hsiMode", "hsiOutput", "convergeRate", "waitForReply"), "Writes the Magnetometer Calibration Control register."))
		.def("read_calculated_magnetometer_calibration", &VnSensor::readCalculatedMagnetometerCalibration, "Reads the Calculated Magnetometer Calibration register.")
		.def("read_indoor_heading_mode_control", &VnSensor::readIndoorHeadingModeControl, "Reads the Indoor Heading Mode Control register.")
		.def("write_indoor_heading_mode_control", &VnSensor::writeIndoorHeadingModeControl, writeIndoorHeadingModeControl_overloads(args("sensor", "maxRateError", "waitForReply"), "Writes the Indoor Heading Mode Control register."))
		.def("read_velocity_compensation_measurement", &VnSensor::readVelocityCompensationMeasurement, "Reads the Velocity Compensation Measurement register.")
		.def("write_velocity_compensation_measurement", &VnSensor::writeVelocityCompensationMeasurement, writeVelocityCompensationMeasurement_overloads(args("sensor", "velocity", "waitForReply"), "Writes the Velocity Compensation Measurement register."))
		.def("read_velocity_compensation_control", &VnSensor::readVelocityCompensationControl, "Reads the Velocity Compensation Control register.")
		.def("write_velocity_compensation_control",
			static_cast<void (VnSensor::*)(VelocityCompensationControlRegister&, bool)>(&VnSensor::writeVelocityCompensationControl),
			writeVelocityCompensationControl_overloads(args("sensor", "fields", "waitForReply"), "Write the Velocity Compensation Control register."))
		.def("write_velocity_compensation_control",
			static_cast<void (VnSensor::*)(VelocityCompensationMode, const float&, const float&, bool)>(&VnSensor::writeVelocityCompensationControl),
			writeVelocityCompensationControlNoStruct_overloads(args("sensor", "mode", "velocityTuning", "rateTuning", "waitForReply"), "Writes the Velocity Compensation Control register."))
		.def("read_velocity_compensation_status", &VnSensor::readVelocityCompensationStatus, "Reads the Velocity Compensation Status register.")
		.def("read_imu_measurements", &VnSensor::readImuMeasurements, "Reads the IMU Measurements register.")
		.def("read_gps_configuration", &VnSensor::readGpsConfiguration, "Reads the GPS Configuration register.")
		.def("write_gps_configuration",
			static_cast<void (VnSensor::*)(GpsConfigurationRegister&, bool)>(&VnSensor::writeGpsConfiguration),
			writeGpsConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the GPS Configuration register."))
		.def("write_gps_configuration",
			static_cast<void (VnSensor::*)(GpsMode, PpsSource, bool)>(&VnSensor::writeGpsConfiguration),
			writeGpsConfigurationNoStruct_overloads(args("sensor", "mode", "ppsSource", "waitForReply"), "Writes the GPS Configuration register."))
		.def("read_gps_antenna_offset", &VnSensor::readGpsAntennaOffset, "Reads the GPS Antenna Offset register.")
		.def("write_gps_antenna_offset", &VnSensor::writeGpsAntennaOffset, writeGpsAntennaOffset_overloads(args("sensor", "position", "waitForReply"), "Writes the GPS Antenna Offset register."))
		.def("read_gps_solution-lla", &VnSensor::readGpsSolutionLla, "Reads the GPS Solution - LLA register.")
		.def("read_gps_solution-ecef", &VnSensor::readGpsSolutionEcef, "Reads the GPS Solution - ECEF register.")
		.def("read_ins_solution-lla", &VnSensor::readInsSolutionLla, "Reads the INS Solution - LLA register.")
		.def("read_ins_solution-ecef", &VnSensor::readInsSolutionEcef, "Reads the INS Solution - ECEF register.")
		.def("read_ins_basic_configuration_vn200", &VnSensor::readInsBasicConfigurationVn200, "Reads the INS Basic Configuration register for a VN-200 sensor.")
		.def("read_ins_basic_configuration_vn300", &VnSensor::readInsBasicConfigurationVn300, "Reads the INS Basic Configuration register for a VN-300 sensor.")
		.def("read_ins_advanced_configuration", &VnSensor::readInsAdvancedConfiguration, "Reads the INS Advanced Configuration register.")
		.def("write_ins_advanced_configuration",
			static_cast<void (VnSensor::*)(InsAdvancedConfigurationRegister&, bool)>(&VnSensor::writeInsAdvancedConfiguration),
			writeInsAdvancedConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the INS Advanced Configuration register."))
		.def("read_ins_state-lla", &VnSensor::readInsStateLla, "Reads the INS State - LLA register.")
		.def("read_ins_state-ecef", &VnSensor::readInsStateEcef, "Reads the INS State - ECEF register.")
		.def("read_startup_filter_bias_estimate", &VnSensor::readStartupFilterBiasEstimate, "Reads the Startup Filter Bias Estimate register.")
		.def("write_startup_filter_bias_estimate",
			static_cast<void (VnSensor::*)(StartupFilterBiasEstimateRegister&, bool)>(&VnSensor::writeStartupFilterBiasEstimate),
			writeStartupFilterBiasEstimate_overloads(args("sensor", "fields", "waitForReply"), "Write the Startup Filter Bias Estimate register."))
		.def("write_startup_filter_bias_estimate",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, const float&, bool)>(&VnSensor::writeStartupFilterBiasEstimate),
			writeStartupFilterBiasEstimateNoStruct_overloads(args("sensor", "gyroBias", "accelBias", "pressureBias", "waitForReply"), "Writes the Startup Filter Bias Estimate register."))
		.def("read_delta_theta_and_delta_velocity", &VnSensor::readDeltaThetaAndDeltaVelocity, "Reads the Delta Theta and Delta Velocity register.")
		.def("read_delta_theta_and_delta_velocity_configuration", &VnSensor::readDeltaThetaAndDeltaVelocityConfiguration, "Reads the Delta Theta and Delta Velocity Configuration register.")
		.def("write_delta_theta_and_delta_velocity_configuration",
			static_cast<void (VnSensor::*)(DeltaThetaAndDeltaVelocityConfigurationRegister&, bool)>(&VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration),
			writeDeltaThetaAndDeltaVelocityConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the Delta Theta and Delta Velocity Configuration register."))
		.def("write_delta_theta_and_delta_velocity_configuration",
			static_cast<void (VnSensor::*)(IntegrationFrame, CompensationMode, CompensationMode, bool)>(&VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration),
			writeDeltaThetaAndDeltaVelocityConfigurationNoStruct_overloads(args("sensor", "integrationFrame", "gyroCompensation", "accelCompensation", "waitForReply"), "Writes the Delta Theta and Delta Velocity Configuration register."))
		.def("read_reference_vector_configuration", &VnSensor::readReferenceVectorConfiguration, "Reads the Reference Vector Configuration register.")
		.def("write_reference_vector_configuration",
			static_cast<void (VnSensor::*)(ReferenceVectorConfigurationRegister&, bool)>(&VnSensor::writeReferenceVectorConfiguration),
			writeReferenceVectorConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the Reference Vector Configuration register."))
		.def("write_reference_vector_configuration",
			static_cast<void (VnSensor::*)(const uint8_t&, const uint8_t&, const uint32_t&, const float&, const vec3d&, bool)>(&VnSensor::writeReferenceVectorConfiguration),
			writeReferenceVectorConfigurationNoStruct_overloads(args("sensor", "useMagModel", "useGravityModel", "recalcThreshold", "year", "position", "waitForReply"), "Writes the Reference Vector Configuration register."))
		.def("read_gyro_compensation", &VnSensor::readGyroCompensation, "Reads the Gyro Compensation register.")
		.def("write_gyro_compensation",
			static_cast<void (VnSensor::*)(GyroCompensationRegister&, bool)>(&VnSensor::writeGyroCompensation),
			writeGyroCompensation_overloads(args("sensor", "fields", "waitForReply"), "Write the Gyro Compensation register."))
		.def("write_gyro_compensation",
			static_cast<void (VnSensor::*)(const mat3f&, const vec3f&, bool)>(&VnSensor::writeGyroCompensation),
			writeGyroCompensationNoStruct_overloads(args("sensor", "c", "b", "waitForReply"), "Writes the Gyro Compensation register."))
		.def("read_imu_filtering_configuration", &VnSensor::readImuFilteringConfiguration, "Reads the IMU Filtering Configuration register.")
		.def("write_imu_filtering_configuration",
			static_cast<void (VnSensor::*)(ImuFilteringConfigurationRegister&, bool)>(&VnSensor::writeImuFilteringConfiguration),
			writeImuFilteringConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the IMU Filtering Configuration register."))
		.def("write_imu_filtering_configuration",
			static_cast<void (VnSensor::*)(const uint16_t&, const uint16_t&, const uint16_t&, const uint16_t&, const uint16_t&, FilterMode, FilterMode, FilterMode, FilterMode, FilterMode, bool)>(&VnSensor::writeImuFilteringConfiguration),
			writeImuFilteringConfigurationNoStruct_overloads(args("sensor", "magWindowSize", "accelWindowSize", "gyroWindowSize", "tempWindowSize", "presWindowSize", "magFilterMode", "accelFilterMode", "gyroFilterMode", "tempFilterMode", "presFilterMode", "waitForReply"), "Writes the IMU Filtering Configuration register."))
		.def("read_gps_compass_baseline", &VnSensor::readGpsCompassBaseline, "Reads the GPS Compass Baseline register.")
		.def("write_gps_compass_baseline",
			static_cast<void (VnSensor::*)(GpsCompassBaselineRegister&, bool)>(&VnSensor::writeGpsCompassBaseline),
			writeGpsCompassBaseline_overloads(args("sensor", "fields", "waitForReply"), "Write the GPS Compass Baseline register."))
		.def("write_gps_compass_baseline",
			static_cast<void (VnSensor::*)(const vec3f&, const vec3f&, bool)>(&VnSensor::writeGpsCompassBaseline),
			writeGpsCompassBaselineNoStruct_overloads(args("sensor", "position", "uncertainty", "waitForReply"), "Writes the GPS Compass Baseline register."))
		.def("read_gps_compass_estimated_baseline", &VnSensor::readGpsCompassEstimatedBaseline, "Reads the GPS Compass Estimated Baseline register.")
		.def("read_imu_rate_configuration", &VnSensor::readImuRateConfiguration, "Reads the IMU Rate Configuration register.")
		.def("write_imu_rate_configuration",
			static_cast<void (VnSensor::*)(ImuRateConfigurationRegister&, bool)>(&VnSensor::writeImuRateConfiguration),
			writeImuRateConfiguration_overloads(args("sensor", "fields", "waitForReply"), "Write the IMU Rate Configuration register."))
		.def("write_imu_rate_configuration",
			static_cast<void (VnSensor::*)(const uint16_t&, const uint16_t&, const float&, const float&, bool)>(&VnSensor::writeImuRateConfiguration),
			writeImuRateConfigurationNoStruct_overloads(args("sensor", "imuRate", "navDivisor", "filterTargetRate", "filterMinRate", "waitForReply"), "Writes the IMU Rate Configuration register."))
		.def("read_yaw_pitch_roll_true_body_acceleration_and_angular_rates", &VnSensor::readYawPitchRollTrueBodyAccelerationAndAngularRates, "Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.")
		.def("read_yaw_pitch_roll_true_inertial_acceleration_and_angular_rates", &VnSensor::readYawPitchRollTrueInertialAccelerationAndAngularRates, "Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.")
		;

		class_<EzAsyncData, boost::noncopyable>("EzAsyncData", "Provides easy and reliable access to asynchronous data from a VectorNav sensor at the cost of a slight performance hit.", no_init)
			.def("connect",
				&EzAsyncData::connect,
				//return_value_policy<const_reference>(),
				return_value_policy<manage_new_object>(),
				args("port_name", "baud_rate"),
				"Connects to a VectorNav sensor with the provided connection parameters.")
				.staticmethod("connect")
			.def("disconnect", &EzAsyncData::disconnect)
			.def_readonly("current_data", &EzAsyncData::currentData)

			// TODO: Would like to make the ez.sensor access as a property instead of a function.
			.def("sensor", &EzAsyncData::sensor, return_internal_reference<>())
			//.def_readonly
			//	("sensor",
			//	&EzAsyncData::sensor,
			//	return_value_policy<manage_new_object>())
			//.def_readonly("sensor", &EzAsyncData::sensor, return_internal_reference<>())
			//.def_readonly("sensor", &EzAsyncData::sensor, return_value_policy<reference_existing_object>())
			//.def_readonly("sensor", &EzAsyncData::sensor)

			.def("get_next_data", static_cast<CompositeData (EzAsyncData::*)()>(&EzAsyncData::getNextData))
			.def("get_next_data", static_cast<CompositeData (EzAsyncData::*)(int)>(&EzAsyncData::getNextData))
			;
}
