// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_LOGGING_H
#define B2_LOGGING_H

#ifdef __cplusplus
extern "C" {
#endif

/// We add the do/while so that the statement "if (condition) b2Printf("test"); else {...}" would fail
/// You can also customize the message by uncommenting out a different line below
#define b2Printf(...) b2OutputPrintfVarArgsInternal(__VA_ARGS__)
//#define b2Printf(...) do {b2OutputPrintfVarArgsInternal("b2Printf[%s,%d]:",__FILE__,__LINE__);b2OutputPrintfVarArgsInternal(__VA_ARGS__); } while(0)
//#define b2Printf b2OutputPrintfVarArgsInternal
//#define b2Printf(...) printf(__VA_ARGS__)
//#define b2Printf(...)
#define b2Warning(...)                                                                \
  do {                                                                                \
    b2OutputWarningMessageVarArgsInternal("b2Warning[%s,%d]:\n", __FILE__, __LINE__); \
    b2OutputWarningMessageVarArgsInternal(__VA_ARGS__);                               \
  } while (0)
#define b2Error(...)                                                              \
  do {                                                                            \
    b2OutputErrorMessageVarArgsInternal("b2Error[%s,%d]:\n", __FILE__, __LINE__); \
    b2OutputErrorMessageVarArgsInternal(__VA_ARGS__);                             \
  } while (0)
#ifndef B2_NO_PROFILE

void b2EnterProfileZone(const char * name);
void b2LeaveProfileZone();
#ifdef __cplusplus

class b2ProfileZone
{
public:
  b2ProfileZone(const char * name)
  {
    b2EnterProfileZone(name);
  }

  ~b2ProfileZone()
  {
    b2LeaveProfileZone();
  }
};

#define B2_PROFILE(name) b2ProfileZone __profile(name)
#endif

#else  // B2_NO_PROFILE

#define B2_PROFILE(name)
#define b2StartProfile(a)
#define b2StopProfile

#endif  //#ifndef B2_NO_PROFILE

typedef void(b2PrintfFunc)(const char * msg);
typedef void(b2WarningMessageFunc)(const char * msg);
typedef void(b2ErrorMessageFunc)(const char * msg);
typedef void(b2EnterProfileZoneFunc)(const char * msg);
typedef void(b2LeaveProfileZoneFunc)();

/// The developer can route b2Printf output using their own implementation
void b2SetCustomPrintfFunc(b2PrintfFunc * printfFunc);
void b2SetCustomWarningMessageFunc(b2WarningMessageFunc * warningMsgFunc);
void b2SetCustomErrorMessageFunc(b2ErrorMessageFunc * errorMsgFunc);

/// Set custom profile zone functions (zones can be nested)
void b2SetCustomEnterProfileZoneFunc(b2EnterProfileZoneFunc * enterFunc);
void b2SetCustomLeaveProfileZoneFunc(b2LeaveProfileZoneFunc * leaveFunc);

/// Don't use those internal functions directly, use the b2Printf or b2SetCustomPrintfFunc instead (or warning/error version)
void b2OutputPrintfVarArgsInternal(const char * str, ...);
void b2OutputWarningMessageVarArgsInternal(const char * str, ...);
void b2OutputErrorMessageVarArgsInternal(const char * str, ...);

#ifdef __cplusplus
}
#endif

#endif  // B2_LOGGING_H
