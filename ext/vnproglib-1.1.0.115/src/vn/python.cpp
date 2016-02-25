// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#include "vn/util/boostpython.h"
#include "boost/python/raw_function.hpp"
//#include "boost/python/errors.hpp"
//#include "boost/python/base_type_traits.hpp"

#if _WIN32
	#include <Windows.h>
#endif

#include "vn/exceptions.h"
#include "vn/event.h"
#include "vn/utilities.h"

// TEMP
#include <iostream>
using namespace std;

namespace bp = boost::python;

#if _WIN32

void setDllLoadPaths(HINSTANCE histDll);
void unsetDllLoadPaths();

#if REVIEW
LONG WINAPI unhandledExceptionHandler(_In_ struct _EXCEPTION_POINTERS *ExceptionInfo)
{
	cout << "EXCEPTION################################" << endl;

	//DWORD temp = GetExceptionCode();

	if (ExceptionInfo == NULL)
		cout << "ITS NULL!" << flush << endl;
	else
		cout << "NOT NULL" << flush << endl;

	try
	{
		cout << "INFO: " << (long)ExceptionInfo << flush << endl;

		cout << "CODE: " << (int)(ExceptionInfo->ExceptionRecord->ExceptionCode) << flush << endl;

		cout << "Made Past" << flush << endl;
	}
	catch (...)
	{
		cout << "EXCEPTION HERE!" << flush << endl;
	}

	return 0;
}
#endif

BOOL WINAPI DllMain(HINSTANCE histDll, DWORD fdwReason, LPVOID)
{
	if (fdwReason == DLL_PROCESS_ATTACH)
	{
		setDllLoadPaths(histDll);
	}
	else if (fdwReason == DLL_PROCESS_DETACH)
	{
		unsetDllLoadPaths();
	}

	//SetUnhandledExceptionFilter(unhandledExceptionHandler);

	return true;
}

// Sets the DLL paths so that the other *.pyd files can load their dependancies.
void setDllLoadPaths(HINSTANCE histDll)
{
	// Get the folder containing _vnpy.pyd.
	char vnpyPydPath[MAX_PATH] = { 0 };
	GetModuleFileName(histDll, vnpyPydPath, _countof(vnpyPydPath));
	string path(vnpyPydPath);
	string dlldir = path.substr(0, path.find_last_of("\\/"));

	SetDllDirectory(dlldir.c_str());
}

void unsetDllLoadPaths()
{
	SetDllDirectory(NULL);
}

#endif

void timeoutTranslator(vn::timeout const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void invalidOperationTranslator(vn::invalid_operation const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void dimensionErrorTranslator(vn::dimension_error const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void unknownErrorTranslator(vn::unknown_error const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void notImplementedTranslator(vn::not_implemented const& e)
{
	PyErr_SetString(PyExc_NotImplementedError, e.what());
}

void nullPointerTranslator(vn::null_pointer const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void permissionDeniedTranslator(vn::permission_denied const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void notSupportedTranslator(vn::not_supported const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

void notFoundTranslator(vn::not_found const& e)
{
	PyErr_SetString(PyExc_FileNotFoundError, e.what());
}

void invalidFormatTranslator(vn::invalid_format const& e)
{
	PyErr_SetString(PyExc_RuntimeError, e.what());
}

bp::tuple test_fire(bp::tuple args, bp::dict kwargs)
{
	cout << "It's working" << flush << endl;
	return bp::make_tuple();
}

//class __declspec(dllexport) TestRaw
class TestRaw
{
	public:
		bp::object testing(bp::tuple args, bp::dict kwargs)
		{
			cout << "Made 1" << flush << endl;
			//return bp::make_tuple();
			return bp::object();
		}
};

bp::object test_this(TestRaw* raw, bp::tuple args, bp::dict kwargs)
{
	return raw->testing(args, kwargs);
}

//bp::object testing_global()

std::string checkDllValidity(std::string dll, std::string workingDir)
{
	std::string returnString;
	std::vector<std::string> missingDlls;

	if(!vn::checkDllValidity(dll, workingDir, missingDlls))
	{
		for (size_t index = 0; index < missingDlls.size(); ++index)
		{
			returnString += missingDlls[index];
			if (index < missingDlls.size() - 1)
			{
				returnString += "\n";
			}
		}
	}

	return returnString;
}

BOOST_PYTHON_MODULE(_vnpy)
{
	//PyEval_InitThreads();

	//docstring_options local_docstring_options(true, true, false);

	bp::def("check_dll_validity", &::checkDllValidity);

	bp::register_exception_translator<vn::timeout>(&timeoutTranslator);
	bp::register_exception_translator<vn::invalid_operation>(&invalidOperationTranslator);
	bp::register_exception_translator<vn::dimension_error>(&dimensionErrorTranslator);
	bp::register_exception_translator<vn::unknown_error>(&unknownErrorTranslator);
	bp::register_exception_translator<vn::not_implemented>(&notImplementedTranslator);
	bp::register_exception_translator<vn::null_pointer>(&nullPointerTranslator);
	bp::register_exception_translator<vn::permission_denied>(&permissionDeniedTranslator);
	bp::register_exception_translator<vn::not_supported>(&notSupportedTranslator);
	bp::register_exception_translator<vn::not_found>(&notFoundTranslator);
	bp::register_exception_translator<vn::invalid_format>(&invalidFormatTranslator);

	
//	bp::class_<vn::Event>("Event", "Allows registration for notification when an event occurs.")
		//.def("fire", static_cast<void (vn::Event::*)()>(&vn::Event::fire))
		//.def("fire", bp::raw_function(vn::Event::fire))
//		.def("fire", bp::raw_function(&vn::Event::fire))
		//.def("fire", raw_function(test_fire))
		//.def("fire", raw_function(static_cast<void (vn::Event::*)(tuple, dict)>(&vn::Event::fire), 1))
//		.def("add", &vn::Event::add)
//		;

//	bp::class_<TestRaw>("TestRaw")
//		.def("testing", bp::raw_function(&TestRaw::testing))
		//.def("testing", bp::raw_function<bp::object (TestRaw::*)(bp::tuple, bp::dict)>(&TestRaw::testing, 1))
		//.def("testing", bp::raw_function(&test_this, 1))
//		;

	//def __init__(self):
    //    self.__handlers = []

    //def __iadd__(self, handler):
    //    self.__handlers.append(handler)
    //    return self

    //def __isub__(self, handler):
    //    self.__handlers.remove(handler)
    //    return self

    //def fire(self, *args, **keywargs):
    //    for handler in self.__handlers:
    //        handler(*args, **keywargs)

    //def clearObjectHandlers(self, inObject):
    //    for theHandler in self.__handlers:
    //        if theHandler.im_self == inObject:
    //            self -= theHandler
}
