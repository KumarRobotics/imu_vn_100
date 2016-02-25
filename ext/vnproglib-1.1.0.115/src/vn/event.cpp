// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#include "vn/event.h"

/*#if _WIN32
	#include <Windows.h>
	#include <tchar.h>
	#include <setupapi.h>
	#include <devguid.h>
	#if _UNICODE
	#else
		#include <stdio.h>
	#endif
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	#include <fcntl.h>
	#include <errno.h>
	#include <termios.h>
	#include <cstring>
	#include <sys/ioctl.h>
	#include <sys/stat.h>
	#include <unistd.h>
	#include <sys/select.h>
#else
	#error "Unknown System"
#endif

#if __linux__
	#include <linux/serial.h>
#elif __APPLE__
	#include <dirent.h>
#endif

#include <list>
#include <iostream>

#include "vn/xplat/thread.h"
#include "vn/xplat/criticalsection.h"
#include "vn/exceptions.h"
#include <vn/xplat/event.h>

using namespace std;*/

#if PYTHON
namespace bp = boost::python;
#endif

namespace vn {

#if OLD
void Event::operator+=(PyObject* handler)
{
}

void Event::add(PyObject* handler)
{
	_pycallables.push_back(handler);
	handler->ob_refcnt++;
}

//void Event::fire()
//{
//	for (auto h : _pycallables)
//		boost::python::call<void>(h);
//}

bp::tuple Event::fire(bp::tuple args, bp::dict kwargs)
{
	// TEMP
	for (auto h : _pycallables)
		boost::python::call<void>(h);

	return bp::make_tuple();
}

#endif

}
