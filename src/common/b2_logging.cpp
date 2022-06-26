/*
Copyright (c) 2013 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "box2d_collision/b2_logging.h"

#include <stdio.h>
#include <stdarg.h>

#ifdef _WIN32
#include <windows.h>
#endif  //_WIN32

void b2PrintfFuncDefault(const char* msg)
{
#ifdef _WIN32
	OutputDebugStringA(msg);
#endif
	printf("%s", msg);
	//is this portable?
	fflush(stdout);
}

void b2WarningMessageFuncDefault(const char* msg)
{
#ifdef _WIN32
	OutputDebugStringA(msg);
#endif
	printf("%s", msg);
	//is this portable?
	fflush(stdout);
}

void b2ErrorMessageFuncDefault(const char* msg)
{
#ifdef _WIN32
	OutputDebugStringA(msg);
#endif
	printf("%s", msg);

	//is this portable?
	fflush(stdout);
}

static b2PrintfFunc* b2s_printfFunc = b2PrintfFuncDefault;
static b2WarningMessageFunc* b2s_warningMessageFunc = b2WarningMessageFuncDefault;
static b2ErrorMessageFunc* b2s_errorMessageFunc = b2ErrorMessageFuncDefault;

///The developer can route b2Printf output using their own implementation
void b2SetCustomPrintfFunc(b2PrintfFunc* printfFunc)
{
	b2s_printfFunc = printfFunc;
}
void b2SetCustomWarningMessageFunc(b2PrintfFunc* warningMessageFunc)
{
	b2s_warningMessageFunc = warningMessageFunc;
}
void b2SetCustomErrorMessageFunc(b2PrintfFunc* errorMessageFunc)
{
	b2s_errorMessageFunc = errorMessageFunc;
}

//#define B2_MAX_DEBUG_STRING_LENGTH 2048
#define B2_MAX_DEBUG_STRING_LENGTH 32768

void b2OutputPrintfVarArgsInternal(const char* str, ...)
{
	char strDebug[B2_MAX_DEBUG_STRING_LENGTH] = {0};
	va_list argList;
	va_start(argList, str);
#ifdef _MSC_VER
	vsprintf_s(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
	vsnprintf(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
	(b2s_printfFunc)(strDebug);
	va_end(argList);
}
void b2OutputWarningMessageVarArgsInternal(const char* str, ...)
{
	char strDebug[B2_MAX_DEBUG_STRING_LENGTH] = {0};
	va_list argList;
	va_start(argList, str);
#ifdef _MSC_VER
	vsprintf_s(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
	vsnprintf(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
	(b2s_warningMessageFunc)(strDebug);
	va_end(argList);
}
void b2OutputErrorMessageVarArgsInternal(const char* str, ...)
{
	char strDebug[B2_MAX_DEBUG_STRING_LENGTH] = {0};
	va_list argList;
	va_start(argList, str);
#ifdef _MSC_VER
	vsprintf_s(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
	vsnprintf(strDebug, B2_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
	(b2s_errorMessageFunc)(strDebug);
	va_end(argList);
}

void b2EnterProfileZoneDefault(const char* name)
{
}
void b2LeaveProfileZoneDefault()
{
}
static b2EnterProfileZoneFunc* b2s_enterFunc = b2EnterProfileZoneDefault;
static b2LeaveProfileZoneFunc* b2s_leaveFunc = b2LeaveProfileZoneDefault;
void b2EnterProfileZone(const char* name)
{
	(b2s_enterFunc)(name);
}
void b2LeaveProfileZone()
{
	(b2s_leaveFunc)();
}

void b2SetCustomEnterProfileZoneFunc(b2EnterProfileZoneFunc* enterFunc)
{
	b2s_enterFunc = enterFunc;
}
void b2SetCustomLeaveProfileZoneFunc(b2LeaveProfileZoneFunc* leaveFunc)
{
	b2s_leaveFunc = leaveFunc;
}

#ifndef _MSC_VER
#undef vsprintf_s
#endif
