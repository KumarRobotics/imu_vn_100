#include <stdio.h>
#if defined(WIN32)
	#include <Windows.h>
	#include <tchar.h>
#endif
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* PORT_NAME = "//dev//ttyUSB0";
const int BAUDRATE = 115200;

void asyncVn100DataListener(void* sender, VnDeviceCompositeData* data);

#if defined(WIN32)
int _tmain(int argc, _TCHAR* argv[])
#else
int main()
#endif
{
	Vn100 vn100;
	int errorCode;
	bool isOptimized;
	
	/* Before attempting to connect to the VN-100 sensor, let's make sure the
	   COM port is optimized. Optimization is only necessary on Windows
	   machines that are connecting to the sensor using a USB cable. */
	errorCode = vncp_comPort_isOptimized(
		PORT_NAME, 
		&isOptimized);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error received when trying to determine if the COM port is optimized.\n");

		return 0;
	}

	if (!isOptimized)
	{
		/* Let's attempt to optimized the COM port. Note that the user must
		   have premission to update the Window's Registry for this to work. */
		
		errorCode = vncp_comPort_optimize(PORT_NAME);

		if (errorCode != VNERR_NO_ERROR)
		{
			printf("Error received when trying to optimize the COM port.\n");

			return 0;
		}
	}

	/* Alright. If we get here, we should have an optimized COM port and can
	   perform some timing tests to make sure everything is running as expected. */

	errorCode = vn100_connect(&vn100, PORT_NAME, BAUDRATE);

	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {

		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error trying to connect to sensor.\n");

		return 0;
	}

	errorCode = vn100_setAsynchronousDataOutputType(
		&vn100,
		VNASYNC_VNYPR,
		true);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when setting the sensor's asynchronous data output type.\n");

		return 0;
	}

	errorCode = vn100_setAsynchronousDataOutputFrequency(
		&vn100,
		100,
		true);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when setting the sensor's asynchronous data output frequency.\n");

		return 0;
	}

	/* We have our sensor configured to output at 100 Hz. Start the timing tests. */

	vncp_startMsTimer();

	vn100_registerAsyncDataReceivedListener(
		&vn100,
		&asyncVn100DataListener);

	/* Pause this thread for 10 seconds. */
	vncp_sleepInMs(10000);

	vn100_unregisterAsyncDataReceivedListener(
		&vn100,
		&asyncVn100DataListener);

	vn100_disconnect(&vn100);

	return 0;
}

void asyncVn100DataListener(void* sender, VnDeviceCompositeData* data)
{
	double diff;

	diff = vncp_stopMsTimer();

	vncp_startMsTimer();

	/* We should be receiving a time difference around 10 ms since we are
	   outputting data at 100 Hz. */
	printf("Time Diff: %.2f\n", diff);
}

