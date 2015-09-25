#include <stdio.h>
#include <Windows.h>
#include <tchar.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "COM1";
const int BAUD_RATE = 115200;

int _tmain(int argc, _TCHAR* argv[])
{
	VN_ERROR_CODE errorCode;
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;

	Vn200 vn200;
	int i;

	errorCode = vn200_connect(
        &vn200,
        COM_PORT,
        BAUD_RATE);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}

	for (i = 0; i < 10; i++)
	{

		/* Query the InsSolution register of the VN-200. Note this method of
		   retrieving the attitude is blocking since a serial command will be
		   sent to the physical sensor and this program will wait until a
		   response is received. */
		vn200_getInsSolutionLla(
			&vn200,
			&gpsTime,
			&gpsWeek,
			&status,
			&ypr,
			&latitudeLognitudeAltitude,
			&nedVelocity,
			&attitudeUncertainty,
			&positionUncertainty,
			&velocityUncertainty);

		printf("INS Solution:\n"
			"  GPS Time:               %f\n"
			"  GPS Week:               %u\n"
			"  INS Status:             %.4X\n"
			"  YPR.Yaw:                %+#7.2f\n"
			"  YPR.Pitch:              %+#7.2f\n"
			"  YPR.Roll:               %+#7.2f\n"
			"  LLA.Latitude:           %+#7.2f\n"
			"  LLA.Longitude:          %+#7.2f\n"
			"  LLA.Altitude:           %+#7.2f\n"
			"  Velocity.North:         %+#7.2f\n"
			"  Velocity.East:          %+#7.2f\n"
			"  Velocity.Down:          %+#7.2f\n"
			"  Attitude Uncertainty:   %+#7.2f\n"
			"  Position Uncertainty:   %+#7.2f\n"
			"  Velocity Uncertainty:   %+#7.2f\n",
			gpsTime,
			gpsWeek,
			status,
			ypr.c0,
			ypr.c1,
			ypr.c2,
			latitudeLognitudeAltitude.c0,
			latitudeLognitudeAltitude.c1,
			latitudeLognitudeAltitude.c2,
			nedVelocity.c0,
			nedVelocity.c1,
			nedVelocity.c2,
			attitudeUncertainty,
			positionUncertainty,
			velocityUncertainty);

		printf("\n\n");

		Sleep(1000);
	}
	
	errorCode = vn200_disconnect(&vn200);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}

