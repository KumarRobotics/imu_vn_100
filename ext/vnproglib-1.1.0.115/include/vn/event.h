// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#ifndef _VN_EVENT_H_
#define _VN_EVENT_H_

/*#if _MSC_VER && _WIN32
	#pragma comment(lib, "setupapi.lib")
#endif*/

//#include <string>
//#include <vector>
#include <list>

#include "vn/util/export.h"
#include "vn/util/compiler.h"

#if PYTHON
	#include "vn/util/boostpython.h"
	#include "vn/python/util.h"
#endif

namespace vn {

#if PYTHON

template <typename A1, typename A2, typename A3>
class Event
{
public:

	Event& operator+=(PyObject* handler)
	{
		this->add(handler);

		return *this;
	}

	// TODO: Prefer the += operator overloaded version.
	void add(PyObject* handler)
	{
		_pycallables.push_back(handler);
		handler->ob_refcnt++;
	}

	void fire(A1 arg1, A2 arg2, A3 arg3)
	//void fire()
	{
		//packet, index, timestamp

		if (_pycallables.empty())
			return;

		python::AcquireGIL scopedLock;

		#if VN_HAS_RANGE_LOOP
		for (auto h : _pycallables)
			boost::python::call<void>(h, boost::ref(*this), boost::ref(arg1), arg2, arg3);
		#else
		for (std::list<PyObject*>::iterator it = _pycallables.begin(); it != _pycallables.end(); ++it)
			boost::python::call<void>(*it, boost::ref(*this), boost::ref(arg1), arg2, arg3);
		#endif
	}

	// TODO: Prefer the -= operator overloaded version.
	void remove(PyObject* handler)
	{
		_pycallables.remove(handler);
		handler->ob_refcnt--;
	}

private:

	#if defined(_MSC_VER)
		#pragma warning(push)

		// Warning about needing dll-interface for _pycallables to be used by clients.
		#pragma warning(disable:4251)
	#endif

	std::list<PyObject*> _pycallables;

	#if defined (_MSC_VER)
		#pragma warning(pop)
	#endif
};

#endif

#if OLD
/// \brief Provides event like hooks similiar to events in .NET.
///
/// \remarks This class is intended to provide event functionality in Python
///          while allowing to be accessed from C++.
//template <class F>
template <typename A1>
class vn_proglib_DLLEXPORT Event
{
	typedef void(Handler)(Event* caller, A1 arg1);
public:

	//#if PYTHON

	/// \brief Allows Python objects to register with this event.
	///
	/// \param[in] handler The Python callable to hook to this event.
//	void operator+=(PyObject* handler);

///	void add(PyObject* handler);

	//{
	//	_pycallables.push_back(handler);
	//	handler->ob_refcnt++;
	//}

	//void fire();
//	boost::python::tuple fire(boost::python::tuple args, boost::python::dict kwargs);

	void add(std::function<Handler> handler)
	{
		_handlers.push_back(handler);
	}

	void fire(A1 arg1)
	{
		for (auto h : _handlers)
			h(this, arg1);
	}



	//EventT& operator+=(int test)
	//void operator+=(int test)
	/*void operator+=(PyObject* handler)
	{
		_pycallables.push_back(handler);
		handler->ob_refcnt++;
		//_handlers.push_back(test);

		//return *this;
	}*/

	//#endif

	#if NOT_COMPILING_ON_LINUX

	void fire()
	{
		#if PYTHON
		for (auto h : _pycallables)
			boost::python::call<void>(h);
		#endif

		//vector<char> pRawData(readBuffer, readBuffer + numOfBytesRead);

		//python::AcquireGIL scopedLock;

		//boost::python::call<void>(pi->_rawDataReceivedHandlerPython, pRawData, pi->_dataRunningIndex);

		//for (auto h : _handlers)
//			std::cout << h << std::endl;
	}

	#endif

private:

	//std::list<F> _handlers;

	std::list<Handler> _handlers;


	#if PYTHON
	std::list<PyObject*> _pycallables;
	#endif
//	std::list<int> _handlers;
	/*def __init__(self) :
		self.__handlers = []

		def __iadd__(self, handler) :
		self.__handlers.append(handler)
		return self

		def __isub__(self, handler) :
		self.__handlers.remove(handler)
		return self

		def fire(self, *args, **keywargs) :
		for handler in self.__handlers :
			handler(*args, **keywargs)

			def clearObjectHandlers(self, inObject) :
			for theHandler in self.__handlers :
				if theHandler.im_self == inObject :
					self -= theHandler*/
};
#endif

#if NON_TEMPLATED_EVENT

/// \brief Provides event like hooks similiar to events in .NET.
///
/// \remarks This class is intended to provide event functionality in Python
///          while allowing to be accessed from C++.
class vn_proglib_DLLEXPORT Event
{
public:

	//#if PYTHON

	/// \brief Allows Python objects to register with this event.
	///
	/// \param[in] handler The Python callable to hook to this event.
	void operator+=(PyObject* handler);

	void add(PyObject* handler);

	//{
	//	_pycallables.push_back(handler);
	//	handler->ob_refcnt++;
	//}

	//void fire();
	boost::python::tuple fire(boost::python::tuple args, boost::python::dict kwargs);


	//EventT& operator+=(int test)
	//void operator+=(int test)
	/*void operator+=(PyObject* handler)
	{
		_pycallables.push_back(handler);
		handler->ob_refcnt++;
		//_handlers.push_back(test);

		//return *this;
	}*/

	//#endif

	#if NOT_COMPILING_ON_LINUX

	void fire()
	{
		#if PYTHON
		for (auto h : _pycallables)
			boost::python::call<void>(h);
		#endif

		//vector<char> pRawData(readBuffer, readBuffer + numOfBytesRead);

		//python::AcquireGIL scopedLock;

		//boost::python::call<void>(pi->_rawDataReceivedHandlerPython, pRawData, pi->_dataRunningIndex);

		//for (auto h : _handlers)
//			std::cout << h << std::endl;
	}

	#endif

private:
	#if PYTHON
	std::list<PyObject*> _pycallables;
	#endif
	std::list<int> _handlers;
	/*def __init__(self) :
		self.__handlers = []

		def __iadd__(self, handler) :
		self.__handlers.append(handler)
		return self

		def __isub__(self, handler) :
		self.__handlers.remove(handler)
		return self

		def fire(self, *args, **keywargs) :
		for handler in self.__handlers :
			handler(*args, **keywargs)

			def clearObjectHandlers(self, inObject) :
			for theHandler in self.__handlers :
				if theHandler.im_self == inObject :
					self -= theHandler*/
};

#endif

}

#endif
