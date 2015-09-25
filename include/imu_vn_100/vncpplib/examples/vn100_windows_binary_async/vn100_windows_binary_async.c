#include <stdio.h>
#include <Windows.h>
#include <tchar.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "COM1";
const int BAUD_RATE = 115200;

void asyncDataListener(
	void* sender,
	VnDeviceCompositeData* data);

int _tmain(int argc, _TCHAR* argv[])
{
	VN_ERROR_CODE errorCode;
	Vn100 vn100;

	errorCode = vn100_connect(
		&vn100,
		COM_PORT,
		BAUD_RATE);

	/* Make sure the user has permission to use the COM port. */
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}

	/* Disable ASCII asynchronous messages since we want to demonstrate the
	   the binary asynchronous messages. */
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_OFF,
        true);

	/* Now configure the binary messages output. Notice how the configuration
	   flags can be joined using the binary OR. */
	errorCode = vn100_setBinaryOutput1Configuration(
		&vn100,
		BINARY_ASYNC_MODE_SERIAL_1,		/* Data will be output on serial port 1. This should be the one we are connected to now. */
		200,							/* Outputting binary data at 4 Hz (800 Hz on-board filter / 200 = 4 Hz). */
		BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL,
		BG3_NONE,
		BG5_NONE,
		true);

	printf("Yaw, Pitch, Roll\n");

	/* Now register to receive notifications when a new binary asynchronous
	   packet is received. */
	errorCode = vn100_registerAsyncDataReceivedListener(&vn100, &asyncDataListener);

	/* Sleep for 10 seconds. Data will be received by the asycDataListener
	   during this time. */
	Sleep(10000);

	errorCode = vn100_unregisterAsyncDataReceivedListener(&vn100, &asyncDataListener);
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}

void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
	printf("  %+#7.2f %+#7.2f %+#7.2f\n",
		data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll);
}
