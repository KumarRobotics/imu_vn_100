// VectorNav Programming Library v1.1.0.115
// Copyright (c) 2016 VectorNav Technologies, LLC
#ifndef _VN_UTIL_EXPORT_H
#define _VN_UTIL_EXPORT_H

#if defined _WINDOWS && defined _BUILD_DLL
	#if defined proglib_cpp_EXPORTS
		#define vn_proglib_DLLEXPORT __declspec(dllexport)
	#else
		#define vn_proglib_DLLEXPORT __declspec(dllimport)
	#endif
#else
	#define vn_proglib_DLLEXPORT
#endif

#if defined _WINDOWS && defined _BUILD_DLL
	#if defined proglib_cpp_graphics_EXPORTS
		#define vn_proglib_graphics_DLLEXPORT __declspec(dllexport)
	#else
		#define vn_proglib_graphics_DLLEXPORT __declspec(dllimport)
	#endif
#else
	#define vn_proglib_graphics_DLLEXPORT
#endif

#endif
