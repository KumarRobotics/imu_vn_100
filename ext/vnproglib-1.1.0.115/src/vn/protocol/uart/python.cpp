// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#include "boost/python.hpp"

#include "vn/protocol/uart/packet.h"
#include "vn/protocol/uart/packetfinder.h"

using namespace std;
using namespace boost::python;
using namespace vn::math;
using namespace vn::protocol::uart;

// This group of identity_ functions allow converting a number in Python into
// the corresponding enum which is allow printing of the enum as a textual
// string representation instead of just returning a number.
AsciiAsync identity_(AsciiAsync aa) { return aa; }
BinaryGroup identity_(BinaryGroup bg) { return bg; }
CommonGroup identity_(CommonGroup cg) { return cg; }
TimeGroup identity_(TimeGroup tg) { return tg; }
ImuGroup identity_(ImuGroup ig) { return ig; }
GpsGroup identity_(GpsGroup gg) { return gg; }
AttitudeGroup identity_(AttitudeGroup cg) { return cg; }
InsGroup identity_(InsGroup ig) { return ig; }

//boost::shared_ptr<Packet> packetConstructor(object const &p)
/*boost::shared_ptr<Packet> packetConstructor(std::string const &p)
{
	return boost::make_shared<Packet>()
	cout << "It's working!!!" << flush << endl;
	return boost::make_shared<Packet>();
}*/

#if PYTHON_OLD

// This used to be a Python compatible constructor for the Packet class.
// However, I am in the process of removing all Python related code from
// the raw C++ library class and moving this fuctionality into the Python
// wrappers themselves. -Paul

Packet::Packet(Type type, boost::python::list packet_data) :
_isPacketDataMine(true),
_length(len(packet_data)),
_data(new char[_length]),
_curExtractLoc(0)
{
	for (auto i = 0u; i < _length; i++)
		_data[i] = bp::extract<uint8_t>(packet_data[i]);
}

#endif

vec3f packetParseVNYPR(Packet& packet)
{
	vec3f holder;
	packet.parseVNYPR(&holder);

	return holder;
}

bool isCompatible(Packet& packet,
	              uint32_t commonGroup,
				  uint32_t timeGroup,
				  uint32_t imuGroup,
				  uint32_t gpsGroup,
				  uint32_t attitudeGroup,
				  uint32_t insGroup)
{
	return packet.isCompatible(static_cast<CommonGroup>(commonGroup),
			static_cast<TimeGroup>(timeGroup),
			static_cast<ImuGroup>(imuGroup),
			static_cast<GpsGroup>(gpsGroup),
			static_cast<AttitudeGroup>(attitudeGroup),
			static_cast<InsGroup>(insGroup));
}

string packetParseUserTag(Packet& packet)
{
	char holder[21];
	packet.parseUserTag(holder);

	return string(holder);
}

string packetParseModelNumber(Packet& packet)
{
	char holder[25];
	packet.parseModelNumber(holder);

	return string(holder);
}

uint32_t packetParseHardwareRevision(Packet& packet)
{
	uint32_t holder;
	packet.parseHardwareRevision(&holder);

	return holder;
}

uint32_t packetParseSerialNumber(Packet& packet)
{
	uint32_t holder;
	packet.parseSerialNumber(&holder);

	return holder;
}

string packetParseFirmwareVersion(Packet& packet)
{
	char holder[16];
	packet.parseFirmwareVersion(holder);

	return string(holder);
}

uint32_t packetParseSerialBaudRate(Packet& packet)
{
	uint32_t holder;
	packet.parseSerialBaudRate(&holder);

	return holder;
}

uint32_t packetParseAsyncDataOutputType(Packet& packet)
{
	uint32_t holder;
	packet.parseAsyncDataOutputType(&holder);

	return holder;
}

uint32_t packetParseAsyncDataOutputFrequency(Packet& packet)
{
	uint32_t holder;
	packet.parseAsyncDataOutputFrequency(&holder);

	return holder;
}

vec3f packetParseYawPitchRoll(Packet& packet)
{
	vec3f holder;
	packet.parseYawPitchRoll(&holder);

	return holder;
}

vec4f packetParseAttitudeQuaternion(Packet& packet)
{
	vec4f holder;
	packet.parseAttitudeQuaternion(&holder);

	return holder;
}

list packetParseQuaternionMagneticAccelerationAndAngularRates(Packet& packet)
{
	list l;

	vec4f quat;
	vec3f mag;
	vec3f accel;
	vec3f gyro;

	packet.parseQuaternionMagneticAccelerationAndAngularRates(
		&quat,
		&mag,
		&accel,
		&gyro);

	l.append(quat);
	l.append(mag);
	l.append(accel);
	l.append(gyro);

	return l;
}

vec3f packetParseMagneticMeasurements(Packet& packet)
{
	vec3f holder;
	packet.parseMagneticMeasurements(&holder);

	return holder;
}

vec3f packetParseAccelerationMeasurements(Packet& packet)
{
	vec3f holder;
	packet.parseAccelerationMeasurements(&holder);

	return holder;
}

vec3f packetParseAngularRateMeasurements(Packet& packet)
{
	vec3f holder;
	packet.parseAngularRateMeasurements(&holder);

	return holder;
}

list packetParseMagneticAccelerationAndAngularRates(Packet& packet)
{
	list l;

	vec3f mag;
	vec3f accel;
	vec3f gyro;

	packet.parseMagneticAccelerationAndAngularRates(
		&mag,
		&accel,
		&gyro);

	l.append(mag);
	l.append(accel);
	l.append(gyro);

	return l;
}

list packetParseMagneticAndGravityReferenceVectors(Packet& packet)
{
	list l;

	vec3f magRef;
	vec3f accRef;

	packet.parseMagneticAndGravityReferenceVectors(
		&magRef,
		&accRef);

	l.append(magRef);
	l.append(accRef);

	return l;
}

list packetParseFilterMeasurementsVarianceParameters(Packet& packet)
{
	list l;

	float angularWalkVariance;
	vec3f angularRateVariance;
	vec3f magneticVariance;
	vec3f accelerationVariance;

	packet.parseFilterMeasurementsVarianceParameters(
		&angularWalkVariance,
		&angularRateVariance,
		&magneticVariance,
		&accelerationVariance);

	l.append(angularWalkVariance);
	l.append(angularRateVariance);
	l.append(magneticVariance);
	l.append(accelerationVariance);

	return l;
}

list packetParseMagnetometerCompensation(Packet& packet)
{
	list l;

	mat3f c;
	vec3f b;

	packet.parseMagnetometerCompensation(
		&c,
		&b);

	l.append(c);
	l.append(b);

	return l;
}

list packetParseFilterActiveTuningParameters(Packet& packet)
{
	list l;

	float magneticDisturbanceGain;
	float accelerationDisturbanceGain;
	float magneticDisturbanceMemory;
	float accelerationDisturbanceMemory;

	packet.parseFilterActiveTuningParameters(
		&magneticDisturbanceGain,
		&accelerationDisturbanceGain,
		&magneticDisturbanceMemory,
		&accelerationDisturbanceMemory);

	l.append(magneticDisturbanceGain);
	l.append(accelerationDisturbanceGain);
	l.append(magneticDisturbanceMemory);
	l.append(accelerationDisturbanceMemory);

	return l;
}

list packetParseAccelerationCompensation(Packet& packet)
{
	list l;

	mat3f c;
	vec3f b;

	packet.parseAccelerationCompensation(
		&c,
		&b);

	l.append(c);
	l.append(b);

	return l;
}

mat3f packetParseReferenceFrameRotation(Packet& packet)
{
	mat3f holder;
	packet.parseReferenceFrameRotation(&holder);

	return holder;
}

list packetParseYawPitchRollMagneticAccelerationAndAngularRates(Packet& packet)
{
	list l;

	vec3f yawPitchRoll;
	vec3f mag;
	vec3f accel;
	vec3f gyro;

	packet.parseYawPitchRollMagneticAccelerationAndAngularRates(
		&yawPitchRoll,
		&mag,
		&accel,
		&gyro);

	l.append(yawPitchRoll);
	l.append(mag);
	l.append(accel);
	l.append(gyro);

	return l;
}

list packetParseCommunicationProtocolControl(Packet& packet)
{
	list l;

	uint8_t serialCount;
	uint8_t serialStatus;
	uint8_t spiCount;
	uint8_t spiStatus;
	uint8_t serialChecksum;
	uint8_t spiChecksum;
	uint8_t errorMode;

	packet.parseCommunicationProtocolControl(
		&serialCount,
		&serialStatus,
		&spiCount,
		&spiStatus,
		&serialChecksum,
		&spiChecksum,
		&errorMode);

	l.append(serialCount);
	l.append(serialStatus);
	l.append(spiCount);
	l.append(spiStatus);
	l.append(serialChecksum);
	l.append(spiChecksum);
	l.append(errorMode);

	return l;
}

list packetParseSynchronizationControl(Packet& packet)
{
	list l;

	uint8_t syncInMode;
	uint8_t syncInEdge;
	uint16_t syncInSkipFactor;
	uint8_t syncOutMode;
	uint8_t syncOutPolarity;
	uint16_t syncOutSkipFactor;
	uint32_t syncOutPulseWidth;

	packet.parseSynchronizationControl(
		&syncInMode,
		&syncInEdge,
		&syncInSkipFactor,
		&syncOutMode,
		&syncOutPolarity,
		&syncOutSkipFactor,
		&syncOutPulseWidth);

	l.append(syncInMode);
	l.append(syncInEdge);
	l.append(syncInSkipFactor);
	l.append(syncOutMode);
	l.append(syncOutPolarity);
	l.append(syncOutSkipFactor);
	l.append(syncOutPulseWidth);

	return l;
}

list packetParseSynchronizationStatus(Packet& packet)
{
	list l;

	uint32_t syncInCount;
	uint32_t syncInTime;
	uint32_t syncOutCount;

	packet.parseSynchronizationStatus(
		&syncInCount,
		&syncInTime,
		&syncOutCount);

	l.append(syncInCount);
	l.append(syncInTime);
	l.append(syncOutCount);

	return l;
}

list packetParseFilterBasicControl(Packet& packet)
{
	list l;

	uint8_t magMode;
	uint8_t extMagMode;
	uint8_t extAccMode;
	uint8_t extGyroMode;
	vec3f gyroLimit;

	packet.parseFilterBasicControl(
		&magMode,
		&extMagMode,
		&extAccMode,
		&extGyroMode,
		&gyroLimit);

	l.append(magMode);
	l.append(extMagMode);
	l.append(extAccMode);
	l.append(extGyroMode);
	l.append(gyroLimit);

	return l;
}

list packetParseVpeBasicControl(Packet& packet)
{
	list l;

	uint8_t enable;
	uint8_t headingMode;
	uint8_t filteringMode;
	uint8_t tuningMode;

	packet.parseVpeBasicControl(
		&enable,
		&headingMode,
		&filteringMode,
		&tuningMode);

	l.append(enable);
	l.append(headingMode);
	l.append(filteringMode);
	l.append(tuningMode);

	return l;
}

list packetParseVpeMagnetometerBasicTuning(Packet& packet)
{
	list l;

	vec3f baseTuning;
	vec3f adaptiveTuning;
	vec3f adaptiveFiltering;

	packet.parseVpeMagnetometerBasicTuning(
		&baseTuning,
		&adaptiveTuning,
		&adaptiveFiltering);

	l.append(baseTuning);
	l.append(adaptiveTuning);
	l.append(adaptiveFiltering);

	return l;
}

list packetParseVpeMagnetometerAdvancedTuning(Packet& packet)
{
	list l;

	vec3f minFiltering;
	vec3f maxFiltering;
	float maxAdaptRate;
	float disturbanceWindow;
	float maxTuning;

	packet.parseVpeMagnetometerAdvancedTuning(
		&minFiltering,
		&maxFiltering,
		&maxAdaptRate,
		&disturbanceWindow,
		&maxTuning);

	l.append(minFiltering);
	l.append(maxFiltering);
	l.append(maxAdaptRate);
	l.append(disturbanceWindow);
	l.append(maxTuning);

	return l;
}

list packetParseVpeAccelerometerBasicTuning(Packet& packet)
{
	list l;

	vec3f baseTuning;
	vec3f adaptiveTuning;
	vec3f adaptiveFiltering;

	packet.parseVpeAccelerometerBasicTuning(
		&baseTuning,
		&adaptiveTuning,
		&adaptiveFiltering);

	l.append(baseTuning);
	l.append(adaptiveTuning);
	l.append(adaptiveFiltering);

	return l;
}

list packetParseVpeAccelerometerAdvancedTuning(Packet& packet)
{
	list l;

	vec3f minFiltering;
	vec3f maxFiltering;
	float maxAdaptRate;
	float disturbanceWindow;
	float maxTuning;

	packet.parseVpeAccelerometerAdvancedTuning(
		&minFiltering,
		&maxFiltering,
		&maxAdaptRate,
		&disturbanceWindow,
		&maxTuning);

	l.append(minFiltering);
	l.append(maxFiltering);
	l.append(maxAdaptRate);
	l.append(disturbanceWindow);
	l.append(maxTuning);

	return l;
}

list packetParseVpeGryoBasicTuning(Packet& packet)
{
	list l;

	vec3f angularWalkVariance;
	vec3f baseTuning;
	vec3f adaptiveTuning;

	packet.parseVpeGryoBasicTuning(
		&angularWalkVariance,
		&baseTuning,
		&adaptiveTuning);

	l.append(angularWalkVariance);
	l.append(baseTuning);
	l.append(adaptiveTuning);

	return l;
}

vec3f packetParseFilterStartupGyroBias(Packet& packet)
{
	vec3f holder;
	packet.parseFilterStartupGyroBias(&holder);

	return holder;
}

list packetParseMagnetometerCalibrationControl(Packet& packet)
{
	list l;

	uint8_t hsiMode;
	uint8_t hsiOutput;
	uint8_t convergeRate;

	packet.parseMagnetometerCalibrationControl(
		&hsiMode,
		&hsiOutput,
		&convergeRate);

	l.append(hsiMode);
	l.append(hsiOutput);
	l.append(convergeRate);

	return l;
}

list packetParseCalculatedMagnetometerCalibration(Packet& packet)
{
	list l;

	mat3f c;
	vec3f b;

	packet.parseCalculatedMagnetometerCalibration(
		&c,
		&b);

	l.append(c);
	l.append(b);

	return l;
}

float packetParseIndoorHeadingModeControl(Packet& packet)
{
	float holder;
	packet.parseIndoorHeadingModeControl(&holder);

	return holder;
}

vec3f packetParseVelocityCompensationMeasurement(Packet& packet)
{
	vec3f holder;
	packet.parseVelocityCompensationMeasurement(&holder);

	return holder;
}

list packetParseVelocityCompensationControl(Packet& packet)
{
	list l;

	uint8_t mode;
	float velocityTuning;
	float rateTuning;

	packet.parseVelocityCompensationControl(
		&mode,
		&velocityTuning,
		&rateTuning);

	l.append(mode);
	l.append(velocityTuning);
	l.append(rateTuning);

	return l;
}

list packetParseVelocityCompensationStatus(Packet& packet)
{
	list l;

	float x;
	float xDot;
	vec3f accelOffset;
	vec3f omega;

	packet.parseVelocityCompensationStatus(
		&x,
		&xDot,
		&accelOffset,
		&omega);

	l.append(x);
	l.append(xDot);
	l.append(accelOffset);
	l.append(omega);

	return l;
}

list packetParseImuMeasurements(Packet& packet)
{
	list l;

	vec3f mag;
	vec3f accel;
	vec3f gyro;
	float temp;
	float pressure;

	packet.parseImuMeasurements(
		&mag,
		&accel,
		&gyro,
		&temp,
		&pressure);

	l.append(mag);
	l.append(accel);
	l.append(gyro);
	l.append(temp);
	l.append(pressure);

	return l;
}

list packetParseGpsConfiguration(Packet& packet)
{
	list l;

	uint8_t mode;
	uint8_t ppsSource;

	packet.parseGpsConfiguration(
		&mode,
		&ppsSource);

	l.append(mode);
	l.append(ppsSource);

	return l;
}

vec3f packetParseGpsAntennaOffset(Packet& packet)
{
	vec3f holder;
	packet.parseGpsAntennaOffset(&holder);

	return holder;
}

list packetParseGpsSolutionLla(Packet& packet)
{
	list l;

	double time;
	uint16_t week;
	uint8_t gpsFix;
	uint8_t numSats;
	vec3d lla;
	vec3f nedVel;
	vec3f nedAcc;
	float speedAcc;
	float timeAcc;

	packet.parseGpsSolutionLla(
		&time,
		&week,
		&gpsFix,
		&numSats,
		&lla,
		&nedVel,
		&nedAcc,
		&speedAcc,
		&timeAcc);

	l.append(time);
	l.append(week);
	l.append(gpsFix);
	l.append(numSats);
	l.append(lla);
	l.append(nedVel);
	l.append(nedAcc);
	l.append(speedAcc);
	l.append(timeAcc);

	return l;
}

list packetParseGpsSolutionEcef(Packet& packet)
{
	list l;

	double tow;
	uint16_t week;
	uint8_t gpsFix;
	uint8_t numSats;
	vec3d position;
	vec3f velocity;
	vec3f posAcc;
	float speedAcc;
	float timeAcc;

	packet.parseGpsSolutionEcef(
		&tow,
		&week,
		&gpsFix,
		&numSats,
		&position,
		&velocity,
		&posAcc,
		&speedAcc,
		&timeAcc);

	l.append(tow);
	l.append(week);
	l.append(gpsFix);
	l.append(numSats);
	l.append(position);
	l.append(velocity);
	l.append(posAcc);
	l.append(speedAcc);
	l.append(timeAcc);

	return l;
}

list packetParseInsSolutionLla(Packet& packet)
{
	list l;

	double time;
	uint16_t week;
	uint16_t status;
	vec3f yawPitchRoll;
	vec3d position;
	vec3f nedVel;
	float attUncertainty;
	float posUncertainty;
	float velUncertainty;

	packet.parseInsSolutionLla(
		&time,
		&week,
		&status,
		&yawPitchRoll,
		&position,
		&nedVel,
		&attUncertainty,
		&posUncertainty,
		&velUncertainty);

	l.append(time);
	l.append(week);
	l.append(status);
	l.append(yawPitchRoll);
	l.append(position);
	l.append(nedVel);
	l.append(attUncertainty);
	l.append(posUncertainty);
	l.append(velUncertainty);

	return l;
}

list packetParseInsSolutionEcef(Packet& packet)
{
	list l;

	double time;
	uint16_t week;
	uint16_t status;
	vec3f yawPitchRoll;
	vec3d position;
	vec3f velocity;
	float attUncertainty;
	float posUncertainty;
	float velUncertainty;

	packet.parseInsSolutionEcef(
		&time,
		&week,
		&status,
		&yawPitchRoll,
		&position,
		&velocity,
		&attUncertainty,
		&posUncertainty,
		&velUncertainty);

	l.append(time);
	l.append(week);
	l.append(status);
	l.append(yawPitchRoll);
	l.append(position);
	l.append(velocity);
	l.append(attUncertainty);
	l.append(posUncertainty);
	l.append(velUncertainty);

	return l;
}

list packetParseInsAdvancedConfiguration(Packet& packet)
{
	list l;

	uint8_t useMag;
	uint8_t usePres;
	uint8_t posAtt;
	uint8_t velAtt;
	uint8_t velBias;
	uint8_t useFoam;
	uint8_t gpsCovType;
	uint8_t velCount;
	float velInit;
	float moveOrigin;
	float gpsTimeout;
	float deltaLimitPos;
	float deltaLimitVel;
	float minPosUncertainty;
	float minVelUncertainty;

	packet.parseInsAdvancedConfiguration(
		&useMag,
		&usePres,
		&posAtt,
		&velAtt,
		&velBias,
		&useFoam,
		&gpsCovType,
		&velCount,
		&velInit,
		&moveOrigin,
		&gpsTimeout,
		&deltaLimitPos,
		&deltaLimitVel,
		&minPosUncertainty,
		&minVelUncertainty);

	l.append(useMag);
	l.append(usePres);
	l.append(posAtt);
	l.append(velAtt);
	l.append(velBias);
	l.append(useFoam);
	l.append(gpsCovType);
	l.append(velCount);
	l.append(velInit);
	l.append(moveOrigin);
	l.append(gpsTimeout);
	l.append(deltaLimitPos);
	l.append(deltaLimitVel);
	l.append(minPosUncertainty);
	l.append(minVelUncertainty);

	return l;
}

list packetParseInsStateLla(Packet& packet)
{
	list l;

	vec3f yawPitchRoll;
	vec3d position;
	vec3f velocity;
	vec3f accel;
	vec3f angularRate;

	packet.parseInsStateLla(
		&yawPitchRoll,
		&position,
		&velocity,
		&accel,
		&angularRate);

	l.append(yawPitchRoll);
	l.append(position);
	l.append(velocity);
	l.append(accel);
	l.append(angularRate);

	return l;
}

list packetParseInsStateEcef(Packet& packet)
{
	list l;

	vec3f yawPitchRoll;
	vec3d position;
	vec3f velocity;
	vec3f accel;
	vec3f angularRate;

	packet.parseInsStateEcef(
		&yawPitchRoll,
		&position,
		&velocity,
		&accel,
		&angularRate);

	l.append(yawPitchRoll);
	l.append(position);
	l.append(velocity);
	l.append(accel);
	l.append(angularRate);

	return l;
}

list packetParseStartupFilterBiasEstimate(Packet& packet)
{
	list l;

	vec3f gyroBias;
	vec3f accelBias;
	float pressureBias;

	packet.parseStartupFilterBiasEstimate(
		&gyroBias,
		&accelBias,
		&pressureBias);

	l.append(gyroBias);
	l.append(accelBias);
	l.append(pressureBias);

	return l;
}

list packetParseDeltaThetaAndDeltaVelocity(Packet& packet)
{
	list l;

	float deltaTime;
	vec3f deltaTheta;
	vec3f deltaVelocity;

	packet.parseDeltaThetaAndDeltaVelocity(
		&deltaTime,
		&deltaTheta,
		&deltaVelocity);

	l.append(deltaTime);
	l.append(deltaTheta);
	l.append(deltaVelocity);

	return l;
}

list packetParseDeltaThetaAndDeltaVelocityConfiguration(Packet& packet)
{
	list l;

	uint8_t integrationFrame;
	uint8_t gyroCompensation;
	uint8_t accelCompensation;

	packet.parseDeltaThetaAndDeltaVelocityConfiguration(
		&integrationFrame,
		&gyroCompensation,
		&accelCompensation);

	l.append(integrationFrame);
	l.append(gyroCompensation);
	l.append(accelCompensation);

	return l;
}

list packetParseReferenceVectorConfiguration(Packet& packet)
{
	list l;

	uint8_t useMagModel;
	uint8_t useGravityModel;
	uint32_t recalcThreshold;
	float year;
	vec3d position;

	packet.parseReferenceVectorConfiguration(
		&useMagModel,
		&useGravityModel,
		&recalcThreshold,
		&year,
		&position);

	l.append(useMagModel);
	l.append(useGravityModel);
	l.append(recalcThreshold);
	l.append(year);
	l.append(position);

	return l;
}

list packetParseGyroCompensation(Packet& packet)
{
	list l;

	mat3f c;
	vec3f b;

	packet.parseGyroCompensation(
		&c,
		&b);

	l.append(c);
	l.append(b);

	return l;
}

list packetParseImuFilteringConfiguration(Packet& packet)
{
	list l;

	uint16_t magWindowSize;
	uint16_t accelWindowSize;
	uint16_t gyroWindowSize;
	uint16_t tempWindowSize;
	uint16_t presWindowSize;
	uint8_t magFilterMode;
	uint8_t accelFilterMode;
	uint8_t gyroFilterMode;
	uint8_t tempFilterMode;
	uint8_t presFilterMode;

	packet.parseImuFilteringConfiguration(
		&magWindowSize,
		&accelWindowSize,
		&gyroWindowSize,
		&tempWindowSize,
		&presWindowSize,
		&magFilterMode,
		&accelFilterMode,
		&gyroFilterMode,
		&tempFilterMode,
		&presFilterMode);

	l.append(magWindowSize);
	l.append(accelWindowSize);
	l.append(gyroWindowSize);
	l.append(tempWindowSize);
	l.append(presWindowSize);
	l.append(magFilterMode);
	l.append(accelFilterMode);
	l.append(gyroFilterMode);
	l.append(tempFilterMode);
	l.append(presFilterMode);

	return l;
}

list packetParseGpsCompassBaseline(Packet& packet)
{
	list l;

	vec3f position;
	vec3f uncertainty;

	packet.parseGpsCompassBaseline(
		&position,
		&uncertainty);

	l.append(position);
	l.append(uncertainty);

	return l;
}

list packetParseGpsCompassEstimatedBaseline(Packet& packet)
{
	list l;

	uint8_t estBaselineUsed;
	uint16_t numMeas;
	vec3f position;
	vec3f uncertainty;

	packet.parseGpsCompassEstimatedBaseline(
		&estBaselineUsed,
		&numMeas,
		&position,
		&uncertainty);

	l.append(estBaselineUsed);
	l.append(numMeas);
	l.append(position);
	l.append(uncertainty);

	return l;
}

list packetParseImuRateConfiguration(Packet& packet)
{
	list l;

	uint16_t imuRate;
	uint16_t navDivisor;
	float filterTargetRate;
	float filterMinRate;

	packet.parseImuRateConfiguration(
		&imuRate,
		&navDivisor,
		&filterTargetRate,
		&filterMinRate);

	l.append(imuRate);
	l.append(navDivisor);
	l.append(filterTargetRate);
	l.append(filterMinRate);

	return l;
}

list packetParseYawPitchRollTrueBodyAccelerationAndAngularRates(Packet& packet)
{
	list l;

	vec3f yawPitchRoll;
	vec3f bodyAccel;
	vec3f gyro;

	packet.parseYawPitchRollTrueBodyAccelerationAndAngularRates(
		&yawPitchRoll,
		&bodyAccel,
		&gyro);

	l.append(yawPitchRoll);
	l.append(bodyAccel);
	l.append(gyro);

	return l;
}

list packetParseYawPitchRollTrueInertialAccelerationAndAngularRates(Packet& packet)
{
	list l;

	vec3f yawPitchRoll;
	vec3f inertialAccel;
	vec3f gyro;

	packet.parseYawPitchRollTrueInertialAccelerationAndAngularRates(
		&yawPitchRoll,
		&inertialAccel,
		&gyro);

	l.append(yawPitchRoll);
	l.append(inertialAccel);
	l.append(gyro);

	return l;
}

BOOST_PYTHON_MODULE(_uart)
{
	enum_<AsciiAsync>("AsciiAsync")
		.value("VNOFF", VNOFF)
		.value("VNYPR", VNYPR)
		.value("VNQTN", VNQTN)
		#ifdef INTERNAL
		.value("VNQTM", VNQTM)
		.value("VNQTA", VNQTA)
		.value("VNQTR", VNQTR)
		.value("VNQMA", VNQMA)
		.value("VNQAR", VNQAR)
		#endif
		.value("VNQMR", VNQMR)
		#ifdef INTERNAL
		.value("VNDCM", VNDCM)
		#endif
		.value("VNMAG", VNMAG)
		.value("VNACC", VNACC)
		.value("VNGYR", VNGYR)
		.value("VNMAR", VNMAR)
		.value("VNYMR", VNYMR)
		#ifdef INTERNAL
		.value("VNYCM", VNYCM)
		#endif
		.value("VNYBA", VNYBA)
		.value("VNYIA", VNYIA)
		#ifdef INTERNAL
		.value("VNICM", VNICM)
		#endif
		.value("VNIMU", VNIMU)
		.value("VNGPS", VNGPS)
		.value("VNGPE", VNGPE)
		.value("VNINS", VNINS)
		.value("VNINE", VNINE)
		.value("VNISL", VNISL)
		.value("VNISE", VNISE)
		.value("VNDTV", VNDTV)
		#ifdef INTERNAL
		.value("VNRAW", VNRAW)
		.value("VNCMV", VNCMV)
		.value("VNSTV", VNSTV)
		.value("VNCOV", VNCOV)
		#endif
		.export_values();

	enum_<BinaryGroup>("BinaryGroup")
		.value("common", BINARYGROUP_COMMON)
		.value("time", BINARYGROUP_TIME)
		.value("imu", BINARYGROUP_IMU)
		.value("gps", BINARYGROUP_GPS)
		.value("attitude", BINARYGROUP_ATTITUDE)
		.value("ins", BINARYGROUP_INS)
		.export_values();

	enum_<CommonGroup>("CommonGroup")
		.value("none", COMMONGROUP_NONE)
		.value("time_startup", COMMONGROUP_TIMESTARTUP)
		.value("time_gps", COMMONGROUP_TIMEGPS)
		.value("time_sync_in", COMMONGROUP_TIMESYNCIN)
		.value("yaw_pitch_roll", COMMONGROUP_YAWPITCHROLL)
		.value("quaternion", COMMONGROUP_QUATERNION)
		.value("angular_rate", COMMONGROUP_ANGULARRATE)
		.value("position", COMMONGROUP_POSITION)
		.value("velocity", COMMONGROUP_VELOCITY)
		.value("accel", COMMONGROUP_ACCEL)
		.value("imu", COMMONGROUP_IMU)
		.value("mag_pres", COMMONGROUP_MAGPRES)
		.value("delta_theta", COMMONGROUP_DELTATHETA)
		.value("ins_status", COMMONGROUP_INSSTATUS)
		.value("sync_in_cnt", COMMONGROUP_SYNCINCNT)
		.value("time_gps_pps", COMMONGROUP_TIMEGPSPPS)
		.export_values();
	
	enum_<TimeGroup>("TimeGroup")
		.value("none", TIMEGROUP_NONE)
		.value("time_startup", TIMEGROUP_TIMESTARTUP)
		.value("time_gps", TIMEGROUP_TIMEGPS)
		.value("gps_tow", TIMEGROUP_GPSTOW)
		.value("gps_week", TIMEGROUP_GPSWEEK)
		.value("time_sync_in", TIMEGROUP_TIMESYNCIN)
		.value("time_gps_pps", TIMEGROUP_TIMEGPSPPS)
		.value("time_utc", TIMEGROUP_TIMEUTC)
		.value("sync_in_cnt", TIMEGROUP_SYNCINCNT)
		.export_values();

	enum_<ImuGroup>("ImuGroup")
		.value("none", IMUGROUP_NONE)
		.value("imu_status", IMUGROUP_IMUSTATUS)
		.value("uncomp_mag", IMUGROUP_UNCOMPMAG)
		.value("uncomp_accel", IMUGROUP_UNCOMPACCEL)
		.value("uncomp_gyro", IMUGROUP_UNCOMPGYRO)
		.value("temp", IMUGROUP_TEMP)
		.value("pres", IMUGROUP_PRES)
		.value("delta_theta", IMUGROUP_DELTATHETA)
		.value("delta_vel", IMUGROUP_DELTAVEL)
		.value("mag", IMUGROUP_MAG)
		.value("accel", IMUGROUP_ACCEL)
		.value("angular_rate", IMUGROUP_ANGULARRATE)
		.value("sens_sat", IMUGROUP_SENSSAT)
		.export_values();

	enum_<GpsGroup>("GpsGroup")
		.value("none", GPSGROUP_NONE)
		.value("utc", GPSGROUP_UTC)
		.value("tow", GPSGROUP_TOW)
		.value("week", GPSGROUP_WEEK)
		.value("num_sats", GPSGROUP_NUMSATS)
		.value("fix", GPSGROUP_FIX)
		.value("pos_lla", GPSGROUP_POSLLA)
		.value("pos_ecef", GPSGROUP_POSECEF)
		.value("vel_ned", GPSGROUP_VELNED)
		.value("vel_ecef", GPSGROUP_VELECEF)
		.value("pos_u", GPSGROUP_POSU)
		.value("vel_u", GPSGROUP_VELU)
		.value("time_u", GPSGROUP_TIMEU)
		.export_values();

	enum_<AttitudeGroup>("AttitudeGroup")
		.value("none", ATTITUDEGROUP_NONE)
		.value("vpe_status", ATTITUDEGROUP_VPESTATUS)
		.value("yaw_pitch_roll", ATTITUDEGROUP_YAWPITCHROLL)
		.value("quaternion", ATTITUDEGROUP_QUATERNION)
		.value("dcm", ATTITUDEGROUP_DCM)
		.value("mag_ned", ATTITUDEGROUP_MAGNED)
		.value("accel_ned", ATTITUDEGROUP_ACCELNED)
		.value("linear_accel_body", ATTITUDEGROUP_LINEARACCELBODY)
		.value("linear_accel_ned", ATTITUDEGROUP_LINEARACCELNED)
		.value("ypr_u", ATTITUDEGROUP_YPRU)
		.export_values();

	enum_<InsGroup>("InsGroup")
		.value("none", INSGROUP_NONE)
		.value("ins_status", INSGROUP_INSSTATUS)
		.value("pos_lla", INSGROUP_POSLLA)
		.value("pos_ecef", INSGROUP_POSECEF)
		.value("vel_body", INSGROUP_VELBODY)
		.value("vel_ecef", INSGROUP_VELECEF)
		.value("mag_ecef", INSGROUP_MAGECEF)
		.value("accel_ecef", INSGROUP_ACCELECEF)
		.value("linear_accel_ecef", INSGROUP_LINEARACCELECEF)
		.value("pos_u", INSGROUP_POSU)
		.value("vel_u", INSGROUP_VELU)
		.export_values();

	def("identity", static_cast<AsciiAsync(*)(AsciiAsync)>(identity_));
	def("identity", static_cast<BinaryGroup(*)(BinaryGroup)>(identity_));
	def("identity", static_cast<CommonGroup(*)(CommonGroup)>(identity_));
	def("identity", static_cast<TimeGroup(*)(TimeGroup)>(identity_));
	def("identity", static_cast<ImuGroup(*)(ImuGroup)>(identity_));
	def("identity", static_cast<GpsGroup(*)(GpsGroup)>(identity_));
	def("identity", static_cast<AttitudeGroup(*)(AttitudeGroup)>(identity_));
	def("identity", static_cast<InsGroup(*)(InsGroup)>(identity_));

	enum_<SyncInMode>("SyncInMode")
		#ifdef INTERNAL
		.value("count2", SYNCINMODE_COUNT2)
		.value("adc2", SYNCINMODE_ADC2)
		.value("async2", SYNCINMODE_ASYNC2)
		#endif
		.value("count", SYNCINMODE_COUNT)
		.value("imu", SYNCINMODE_IMU)
		.value("async", SYNCINMODE_ASYNC)
		.export_values();

	enum_<SyncInEdge>("SyncInEdge")
		.value("rising", SYNCINEDGE_RISING)
		.value("falling", SYNCINEDGE_FALLING)
		.export_values();

	enum_<SyncOutMode>("SyncOutMode")
		.value("none", SYNCOUTMODE_NONE)
		.value("item_start", SYNCOUTMODE_ITEMSTART)
		.value("imu_ready", SYNCOUTMODE_IMUREADY)
		.value("ins", SYNCOUTMODE_INS)
		.value("gps_pps", SYNCOUTMODE_GPSPPS)
		.export_values();

	enum_<SyncOutPolarity>("SyncOutPolarity")
		.value("negative", SYNCOUTPOLARITY_NEGATIVE)
		.value("positive", SYNCOUTPOLARITY_POSITIVE)
		.export_values();

	enum_<CountMode>("CountMode")
		.value("none", COUNTMODE_NONE)
		.value("sync_in_count", COUNTMODE_SYNCINCOUNT)
		.value("sync_in_time", COUNTMODE_SYNCINTIME)
		.value("sync_out_counter", COUNTMODE_SYNCOUTCOUNTER)
		.value("gps_pps", COUNTMODE_GPSPPS)
		.export_values();

	enum_<StatusMode>("StatusMode")
		.value("off", STATUSMODE_OFF)
		.value("vpe_status", STATUSMODE_VPESTATUS)
		.value("ins_status", STATUSMODE_INSSTATUS)
		.export_values();

	enum_<ChecksumMode>("ChecksumMode")
		.value("off", CHECKSUMMODE_OFF)
		.value("checksum", CHECKSUMMODE_CHECKSUM)
		.value("crc", CHECKSUMMODE_CRC)
		.export_values();

	enum_<ErrorMode>("ErrorMode")
		.value("ignore", ERRORMODE_IGNORE)
		.value("send", ERRORMODE_SEND)
		.value("send_and_off", ERRORMODE_SENDANDOFF)
		.export_values();

	enum_<FilterMode>("FilterMode")
		.value("no_filtering", FILTERMODE_NOFILTERING)
		.value("only_raw", FILTERMODE_ONLYRAW)
		.value("only_compensated", FILTERMODE_ONLYCOMPENSATED)
		.value("both", FILTERMODE_BOTH)
		.export_values();

	enum_<IntegrationFrame>("IntegrationFrame")
		.value("body", INTEGRATIONFRAME_BODY)
		.value("ned", INTEGRATIONFRAME_NED)
		.export_values();

	enum_<CompensationMode>("CompensationMode")
		.value("none", COMPENSATIONMODE_NONE)
		.value("bias", COMPENSATIONMODE_BIAS)
		.export_values();

	enum_<GpsFix>("GpsFix")
		.value("no_fix", GPSFIX_NOFIX)
		.value("time_only", GPSFIX_TIMEONLY)
		.value("2d", GPSFIX_2D)
		.value("3d", GPSFIX_3D)
		.export_values();

	enum_<GpsMode>("GpsMode")
		.value("on_board_gps", GPSMODE_ONBOARDGPS)
		.value("external_gps", GPSMODE_EXTERNALGPS)
		.value("external_vn200_gps", GPSMODE_EXTERNALVN200GPS)
		.export_values();

	enum_<PpsSource>("PpsSource")
		.value("gps_pps_rising", PPSSOURCE_GPSPPSRISING)
		.value("gps_pps_falling", PPSSOURCE_GPSPPSFALLING)
		.value("sync_in_rising", PPSSOURCE_SYNCINRISING)
		.value("sync_in_falling", PPSSOURCE_SYNCINFALLING)
		.export_values();

	enum_<VpeEnable>("VpeEnable")
		.value("disable", VPEENABLE_DISABLE)
		.value("enable", VPEENABLE_ENABLE)
		.export_values();

	enum_<HeadingMode>("HeadingMode")
		.value("absolute", HEADINGMODE_ABSOLUTE)
		.value("relative", HEADINGMODE_RELATIVE)
		.value("indoor", HEADINGMODE_INDOOR)
		.export_values();

	enum_<VpeMode>("VpeMode")
		.value("off", VPEMODE_OFF)
		.value("mode1", VPEMODE_MODE1)
		.export_values();

	enum_<Scenario>("Scenario")
		.value("ahrs", SCENARIO_AHRS)
		.value("ins_with_pressure", SCENARIO_INSWITHPRESSURE)
		.value("ins_without_pressure", SCENARIO_INSWITHOUTPRESSURE)
		.value("gps_moving_baseline_dynamic", SCENARIO_GPSMOVINGBASELINEDYNAMIC)
		.value("gps_moving_baseline_static", SCENARIO_GPSMOVINGBASELINESTATIC)
		.export_values();

	enum_<HsiMode>("HsiMode")
		.value("off", HSIMODE_OFF)
		.value("run", HSIMODE_RUN)
		.value("reset", HSIMODE_RESET)
		.export_values();

	enum_<HsiOutput>("HsiOutput")
		.value("no_onboard", HSIOUTPUT_NOONBOARD)
		.value("use_onboard", HSIOUTPUT_USEONBOARD)
		.export_values();

	enum_<VelocityCompensationMode>("VelocityCompensationMode")
		.value("disabled", VELOCITYCOMPENSATIONMODE_DISABLED)
		.value("body_measurement", VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT)
		.export_values();

	enum_<MagneticMode>("MagneticMode")
		.value("2d", MAGNETICMODE_2D)
		.value("3d", MAGNETICMODE_3D)
		.export_values();

	enum_<ExternalSensorMode>("ExternalSensorMode")
		.value("internal", EXTERNALSENSORMODE_INTERNAL)
		.value("external_200hz", EXTERNALSENSORMODE_EXTERNAL200HZ)
		.value("external_on_update", EXTERNALSENSORMODE_EXTERNALONUPDATE)
		.export_values();

	enum_<FoamInit>("FoamInit")
		.value("no_foam_init", FOAMINIT_NOFOAMINIT)
		.value("foam_init_pitch_roll", FOAMINIT_FOAMINITPITCHROLL)
		.value("foam_init_heading_pitch_roll", FOAMINIT_FOAMINITHEADINGPITCHROLL)
		.value("foam_init_pitch_roll_covariance", FOAMINIT_FOAMINITPITCHROLLCOVARIANCE)
		.value("foam_init_heading_pitch_roll_covariance", FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE)
		.export_values();

	enum_<AsyncMode>("AsyncMode")
		.value("none", ASYNCMODE_NONE)
		.value("port1", ASYNCMODE_PORT1)
		.value("port2", ASYNCMODE_PORT2)
		.value("both", ASYNCMODE_BOTH)
		.export_values();

	{
		//scope packetScope = class_<Packet, boost::noncopyable>("Packet", "Structure representing a UART packet received from the VectorNav sensor.", no_init)
		scope packetScope = class_<Packet, boost::noncopyable>("Packet", "Structure representing a UART packet received from the VectorNav sensor.")
			//.def(init<Packet::Type, list>())
			//.def("__init__", make_constructor(&packetConstructor))
			.def(init<string>())
			.def("is_valid", &Packet::isValid)
			//.def("type", &Packet::type)
			.def_readonly("type", &Packet::type)
			.def("determine_ascii_async_type", &Packet::determineAsciiAsyncType)
			.def("groups", &Packet::groups)
			.def("group_field", &Packet::groupField)
			.def("extract_uint8", &Packet::extractUint8)
			.def("extract_int8", &Packet::extractInt8)
			.def("extract_uint16", &Packet::extractUint16)
			.def("extract_uint32", &Packet::extractUint32)
			.def("extract_uint64", &Packet::extractUint64)
			.def("extract_float", &Packet::extractFloat)
			.def("extract_vec3f", &Packet::extractVec3f)
			.def("extract_vec3d", &Packet::extractVec3d)
			.def("extract_vec4f", &Packet::extractVec4f)
			.def("extract_mat3f", &Packet::extractMat3f)
			.def("parse_VNYPR", &packetParseVNYPR)
			.def("is_compatible", &isCompatible)
			.def("parse_user_tag", &packetParseUserTag)
			.def("parse_model_number", &packetParseModelNumber)
			.def("parse_hardware_revision", &packetParseHardwareRevision)
			.def("parse_serial_number", &packetParseSerialNumber)
			.def("parse_firmware_version", &packetParseFirmwareVersion)
			.def("parse_serial_baud_rate", &packetParseSerialBaudRate)
			.def("parse_async_data_output_type", &packetParseAsyncDataOutputType)
			.def("parse_async_data_output_frequency", &packetParseAsyncDataOutputFrequency)
			.def("parse_yaw_pitch_roll", &packetParseYawPitchRoll)
			.def("parse_attitude_quaternion", &packetParseAttitudeQuaternion)
			.def("parse_quaternion_magnetic_acceleration_and_angular_rates", &packetParseQuaternionMagneticAccelerationAndAngularRates)
			.def("parse_magnetic_measurements", &packetParseMagneticMeasurements)
			.def("parse_acceleration_measurements", &packetParseAccelerationMeasurements)
			.def("parse_angular_rate_measurements", &packetParseAngularRateMeasurements)
			.def("parse_magnetic_acceleration_and_angular_rates", &packetParseMagneticAccelerationAndAngularRates)
			.def("parse_magnetic_and_gravity_reference_vectors", &packetParseMagneticAndGravityReferenceVectors)
			.def("parse_filter_measurements_variance_parameters", &packetParseFilterMeasurementsVarianceParameters)
			.def("parse_magnetometer_compensation", &packetParseMagnetometerCompensation)
			.def("parse_filter_active_tuning_parameters", &packetParseFilterActiveTuningParameters)
			.def("parse_acceleration_compensation", &packetParseAccelerationCompensation)
			.def("parse_reference_frame_rotation", &packetParseReferenceFrameRotation)
			.def("parse_yaw_pitch_roll_magnetic_acceleration_and_angular_rates", &packetParseYawPitchRollMagneticAccelerationAndAngularRates)
			.def("parse_communication_protocol_control", &packetParseCommunicationProtocolControl)
			.def("parse_synchronization_control", &packetParseSynchronizationControl)
			.def("parse_synchronization_status", &packetParseSynchronizationStatus)
			.def("parse_filter_basic_control", &packetParseFilterBasicControl)
			.def("parse_vpe_basic_control", &packetParseVpeBasicControl)
			.def("parse_vpe_magnetometer_basic_tuning", &packetParseVpeMagnetometerBasicTuning)
			.def("parse_vpe_magnetometer_advanced_tuning", &packetParseVpeMagnetometerAdvancedTuning)
			.def("parse_vpe_accelerometer_basic_tuning", &packetParseVpeAccelerometerBasicTuning)
			.def("parse_vpe_accelerometer_advanced_tuning", &packetParseVpeAccelerometerAdvancedTuning)
			.def("parse_vpe_gryo_basic_tuning", &packetParseVpeGryoBasicTuning)
			.def("parse_filter_startup_gyro_bias", &packetParseFilterStartupGyroBias)
			.def("parse_magnetometer_calibration_control", &packetParseMagnetometerCalibrationControl)
			.def("parse_calculated_magnetometer_calibration", &packetParseCalculatedMagnetometerCalibration)
			.def("parse_indoor_heading_mode_control", &packetParseIndoorHeadingModeControl)
			.def("parse_velocity_compensation_measurement", &packetParseVelocityCompensationMeasurement)
			.def("parse_velocity_compensation_control", &packetParseVelocityCompensationControl)
			.def("parse_velocity_compensation_status", &packetParseVelocityCompensationStatus)
			.def("parse_imu_measurements", &packetParseImuMeasurements)
			.def("parse_gps_configuration", &packetParseGpsConfiguration)
			.def("parse_gps_antenna_offset", &packetParseGpsAntennaOffset)
			.def("parse_gps_solution-lla", &packetParseGpsSolutionLla)
			.def("parse_gps_solution-ecef", &packetParseGpsSolutionEcef)
			.def("parse_ins_solution-lla", &packetParseInsSolutionLla)
			.def("parse_ins_solution-ecef", &packetParseInsSolutionEcef)
			.def("parse_ins_advanced_configuration", &packetParseInsAdvancedConfiguration)
			.def("parse_ins_state-lla", &packetParseInsStateLla)
			.def("parse_ins_state-ecef", &packetParseInsStateEcef)
			.def("parse_startup_filter_bias_estimate", &packetParseStartupFilterBiasEstimate)
			.def("parse_delta_theta_and_delta_velocity", &packetParseDeltaThetaAndDeltaVelocity)
			.def("parse_delta_theta_and_delta_velocity_configuration", &packetParseDeltaThetaAndDeltaVelocityConfiguration)
			.def("parse_reference_vector_configuration", &packetParseReferenceVectorConfiguration)
			.def("parse_gyro_compensation", &packetParseGyroCompensation)
			.def("parse_imu_filtering_configuration", &packetParseImuFilteringConfiguration)
			.def("parse_gps_compass_baseline", &packetParseGpsCompassBaseline)
			.def("parse_gps_compass_estimated_baseline", &packetParseGpsCompassEstimatedBaseline)
			.def("parse_imu_rate_configuration", &packetParseImuRateConfiguration)
			.def("parse_yaw_pitch_roll_true_body_acceleration_and_angular_rates", &packetParseYawPitchRollTrueBodyAccelerationAndAngularRates)
			.def("parse_yaw_pitch_roll_true_inertial_acceleration_and_angular_rates", &packetParseYawPitchRollTrueInertialAccelerationAndAngularRates)
			;

		//.def("determineAsciiAsyncType", &Packet::determineAsciiAsyncType, "Determines the type of ASCII asynchronous message this packet is.")
		//.def("parseVNYPR", &Packet::parseVNYPR, args("The yaw, pitch, roll values in the packet."), "Parses a VNYPR asynchronous packet.")

		enum_<Packet::Type>("Type")
			.value("unknown", Packet::TYPE_UNKNOWN)
			.value("binary", Packet::TYPE_BINARY)
			.value("ascii", Packet::TYPE_ASCII)
			.export_values();
	}

	class_<PacketFinder, boost::noncopyable>("PacketFinder")
		.def("process_received_data", static_cast<void (PacketFinder::*)(list)>(&PacketFinder::processReceivedData))
		//.def("register_packet_found_handler", &PacketFinder::register_packet_found_handler)
		//.def("test_callback", &test_callback2)py
		.def("register_packet_found_handler", &PacketFinder::register_packet_found_handler, return_internal_reference<>())
		;

}
