// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#pragma once

#include "Python.h"

namespace vn {
namespace python {

/// \brief Used for releasing the Python Global Interpretor Lock.
///
/// <code>
/// void run()
/// {
///     // Constructor does necessary unlocking.
///     ReleaseGIL gilUnlocked = ReleaseGIL();
///
///     // ... do your stuff here ...
///
///     // Once function returns, destructor takes care for reaquiring the lock.
/// }
/// </code>
///
/// NOTE: Need to ensure that PyEval_InitThreads() is called early in the program
/// before use this class.
class ReleaseGIL
{
public:
	inline ReleaseGIL()
	{
		// Python threads must have already been initialized globally via PyEval_InitThreads().
		//assert(PyEval_ThreadsInitialize());

		_saveState = PyEval_SaveThread();
	}

	inline ~ReleaseGIL()
	{
		PyEval_RestoreThread(_saveState);
	}

private:
	PyThreadState* _saveState;
};

/// \brief Used for acquiring and then releasing the Python Global Interpretor Lock.
///
/// <code>
/// void run()
/// {
///     // Constructor does necessary locking.
///     AcquireGIL gilLocked;
///
///     // ... do your stuff here such as calling Python code ...
///
///     // Once function returns, destructor takes care for releasing the lock.
/// }
/// </code>
///
/// NOTE: Need to ensure that PyEval_InitThreads() is called early in the program
/// before use this class.
class AcquireGIL
{
public:
	inline AcquireGIL()
	{
		// Python threads must have already been initialized globally via PyEval_InitThreads().
		//assert(PyEval_ThreadsInitialize());

		_saveState = PyGILState_Ensure();
	}

	inline ~AcquireGIL()
	{
		PyGILState_Release(_saveState);
	}

private:
	PyGILState_STATE _saveState;
};

}
}
