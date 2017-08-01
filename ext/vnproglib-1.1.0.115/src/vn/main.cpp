// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#ifdef _win32
#include <Windows.h>
#include <tchar.h>

#endif

#include <stdio.h>
#include <iostream>

#include "gtest/gtest.h"

int main(int argc, char* argv[])
{
	int i = 0;
	char* c = "Testing1234";

	testing::InitGoogleTest(&i, &c);

	return RUN_ALL_TESTS();
}
