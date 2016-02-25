// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#include "boost/python.hpp"
#include "boost/python/scope.hpp"

#include "vn/xplat/port.h"
#include "vn/xplat/serialport.h"
//#include "vn/event.h"
#include "vn/xplat/thread.h"
#include "vn/python/util.h"
#include "vn/xplat/time.h"

using namespace boost::python;
using namespace vn;
using namespace vn::xplat;

void writeWrapper(IPort& self, str out)
{
	std::string toWrite = extract<std::string>(out);
	self.write(toWrite.c_str(), toWrite.size());
}

list readWrapper(IPort& self, size_t numBytesToRead)
{
	char* buf = new char[numBytesToRead];
	list r;
	size_t numBytesActuallyRead = 0;

	self.read(buf, numBytesToRead, numBytesActuallyRead);

	for (size_t i = 0; i < numBytesActuallyRead; i++)
		r.append(buf[i]);

	delete [] buf;

	return r;
}

list getPortNames()
{
	std::vector<std::string> names = SerialPort::getPortNames();

	list pl;

	for (std::vector<std::string>::iterator it = names.begin(); it != names.end(); ++it)
		pl.append(*it);
	//for (std::string n : names)
	//	pl.append(n);

	return pl;
}

void threadSleepSecWrapper(uint32_t numOfSecsToSleep)
{
	python::ReleaseGIL scopedRelase;

	Thread::sleepSec(numOfSecsToSleep);
}

void threadSleepMsWrapper(uint32_t numOfMsToSleep)
{
	python::ReleaseGIL scopedRelase;

	Thread::sleepMs(numOfMsToSleep);
}

BOOST_PYTHON_MODULE(_xplat)
{
//	class_<Event, boost::noncopyable>("Event")
//		//.def(self += int())
//		.def(self += &PyObject())
//		.def("fire", &Event::fire)
//		;

	class_<TimeStamp>("TimeStamp", no_init)
		.def("get", &TimeStamp::get).staticmethod("get")
		.def_readonly("_sec", &TimeStamp::_sec)
		.def_readonly("_usec", &TimeStamp::_usec)
		;

	class_<IPort, boost::noncopyable>("Port", no_init)
		.def("open", &IPort::open)
		.def("write", &writeWrapper)
		.def("read", &readWrapper)
		;

	class_<Thread, boost::noncopyable>("Thread", no_init)
		//.def("sleep_sec", &Thread::sleepSec).staticmethod("sleep_sec")
		.def("sleep_sec", &threadSleepSecWrapper).staticmethod("sleep_sec")
		.def("sleep_ms", &threadSleepMsWrapper).staticmethod("sleep_ms")
		;

	scope serialPortScope = class_<SerialPort, bases<IPort>, boost::noncopyable>("SerialPort", "Represents a cross-platform serial port.", init<std::string, uint32_t>())
		.def("get_port_names", &getPortNames, "The list of available serial port names.")
		.add_property("stop_bits", &SerialPort::stopBits, &SerialPort::setStopBits)
		;
		
	enum_<SerialPort::StopBits>("StopBits")
		.value("ONE_STOP_BIT", SerialPort::ONE_STOP_BIT)
		.value("TWO_STOP_BITS", SerialPort::TWO_STOP_BITS)
		.export_values();

}
