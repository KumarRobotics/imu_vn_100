#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyS1";
const int BAUD_RATE = 115200;

int main()
{
	VN_ERROR_CODE errorCode;
	Vn200 vn200;
	int i;

	errorCode = vn200_connect(
        &vn200,
        COM_PORT,
        BAUD_RATE);
	
	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {

		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}
	
	/* Configure the VN-200 to output asynchronous data. */
	errorCode = vn200_setAsynchronousDataOutputType(
        &vn200,
        VNASYNC_VNINS,
        true);

	/* Pause to ensure we have received the first asynchronous data record
	   from the sensor. */
	sleep(1);

	for (i = 0; i < 10; i++) {

		VnDeviceCompositeData data;

		/* The library is handling and storing asynchronous data by itself.
		   Calling this function retrieves the most recently processed
		   asynchronous data packet. */
		vn200_getCurrentAsyncData(&vn200, &data);

		printf("INS Solution:\n"
			"  YPR.Yaw:                %+#7.2f\n"
			"  YPR.Pitch:              %+#7.2f\n"
			"  YPR.Roll:               %+#7.2f\n"
			"  LLA.Latitude:           %+#7.2f\n"
			"  LLA.Longitude:          %+#7.2f\n"
			"  LLA.Altitude:           %+#7.2f\n"
			"  Velocity.North:         %+#7.2f\n"
			"  Velocity.East:          %+#7.2f\n"
			"  Velocity.Down:          %+#7.2f\n",
			data.ypr.yaw,
			data.ypr.pitch,
			data.ypr.roll,
			data.latitudeLongitudeAltitude.c0,
			data.latitudeLongitudeAltitude.c1,
			data.latitudeLongitudeAltitude.c2,
			data.velocity.c0,
			data.velocity.c1,
			data.velocity.c2);

		printf("\n\n");

		sleep(1);
	}
	
	errorCode = vn200_disconnect(&vn200);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}
