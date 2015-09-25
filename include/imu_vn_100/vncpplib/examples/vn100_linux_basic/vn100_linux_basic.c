#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyS1";
const int BAUD_RATE = 115200;

int main()
{
	VN_ERROR_CODE errorCode;
	Vn100 vn100;
	VnYpr ypr;
	int i;

	errorCode = vn100_connect(
		&vn100,
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
	
	printf("Yaw, Pitch, Roll\n");

	for (i = 0; i < 10; i++)
	{
		
		/* Query the YawPitchRoll register of the VN-100. Note this method of
		   retrieving the attitude is blocking since a serial command will be
		   sent to the physical sensor and this program will wait until a
		   response is received. */
		errorCode = vn100_getYawPitchRoll(&vn100, &ypr);
		
		printf("  %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		
		/* Wait for 1 second before we query the sensor again. */
		sleep(1);
		
	}
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}

