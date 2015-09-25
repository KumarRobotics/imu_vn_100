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
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}
	
	/* Configure the VN-100 to output asynchronous data. */
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_VNYPR,
        true);
	
	printf("Yaw, Pitch, Roll\n");

	/* Now tell the library which function to call when a new asynchronous
	   packet is received. */
	errorCode = vn100_registerAsyncDataReceivedListener(
        &vn100,
		&asyncDataListener);

	/* Pause the current thread for 10 seconds. Data received asynchronously
	   will call the function asyncDataListener on a separate thread. */
	Sleep(10000);

	errorCode = vn100_unregisterAsyncDataReceivedListener(
        &vn100,
        &asyncDataListener);
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;
}

void asyncDataListener(
	void* sender,
	VnDeviceCompositeData* data)
{
	printf("  %+#7.2f %+#7.2f %+#7.2f\n",
		data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll);
}

