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
	Vn100 vn100;
	VnYpr ypr;
	int i;

	errorCode = vn100_connect(
		&vn100,
		COM_PORT,
		BAUD_RATE);
	
	if (errorCode != VNERR_NO_ERROR)
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
		errorCode = vn100_getYawPitchRoll(
			&vn100,
			&ypr);
		
		printf("  %+#7.2f %+#7.2f %+#7.2f\n",
			ypr.yaw,
			ypr.pitch,
			ypr.roll);
		
		/* Wait for 1 second before we query the sensor again. */
		Sleep(1000);
		
	}
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}

