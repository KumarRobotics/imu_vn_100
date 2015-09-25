#include <stdio.h>
#include <Windows.h>
#include <tchar.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* PORT_NAME = "COM1";
const int BAUDRATE = 115200;

void errorCodeReceivedListener(void* sender, VN_ERROR_CODE errorCode);

int _tmain(int argc, _TCHAR* argv[])
{
	Vn200 vn200;
	int errorCode;

	errorCode = vn200_connect(&vn200, PORT_NAME, BAUDRATE);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error trying to connect to sensor.\n");

		return 0;
	}

	errorCode = vn200_registerErrorCodeReceivedListener(
		&vn200,
		&errorCodeReceivedListener);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error attempting to register our errorCodeReceivedListener function.\n");

		return 0;
	}

	/* We will try to set the asynchronous data output frequency to an invalid
	 * value to demonstrate the error code system. */
	errorCode = vn200_setAsynchronousDataOutputFrequency(
		&vn200,
		27,
		true);

	if (errorCode == VNERR_SENSOR_INVALID_PARAMETER)
	{
		printf("Received VNERR_SENSOR_INVALID_PARAMETER in _tmain function.\n");
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		/* This is not the error code the example expected to receive. */
		printf("Received error code %u in _tmain function.\n", errorCode);
	}
	else
	{
		printf("Did not receive an error as expected.\n");
	}
	
	/* Sleep 1 second to make sure our errorCodeReceivedListener function gets the error code. */
	Sleep(1000);

	errorCode = vn200_unregisterErrorCodeReceivedListener(
		&vn200,
		&errorCodeReceivedListener);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Received error when trying to unregister our errorCodeReceivedListener function.\n");

		return 0;
	}
	
	errorCode = vn200_disconnect(&vn200);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Received error when trying to disconnect from the sensor.\n");

		return 0;
	}

	return 0;
}

void errorCodeReceivedListener(void* sender, VN_ERROR_CODE errorCode)
{
	if (errorCode == VNERR_SENSOR_INVALID_PARAMETER)
	{
		printf("Received VNERR_SENSOR_INVALID_PARAMETER in errorCodeReceivedListener function.\n");
	}
	else
	{
		/* This is not the error code the example expected to receive. */
		printf("Received error code %u in errorCodeReceivedListener function.\n", errorCode);
	}
}

