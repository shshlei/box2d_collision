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

#ifndef B2_COMMON_H
#define B2_COMMON_H

#include "b2_scalar.h"
#include "b2_settings.h"

#include <stddef.h>
#include <assert.h>
#include <float.h>

#ifdef B2_DEBUG
	#define b2DEBUG
#endif

#define B2_NOT_USED(x) ((void)(x))

#define	b2_maxFloat		B2_INFINITY
#define	b2_epsilon		B2_EPSILON
#define b2_pi			B2_PI

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
#define b2_maxManifoldPoints	2

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
#define b2_aabbExtensionRatio	b2Scalar(0.1)

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
#define b2_aabbMultiplier		b2Scalar(4.0)

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
#define b2_maxTOIContacts			32

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
#define b2_maxAngularCorrection		(b2Scalar(8.0) / b2Scalar(180.0) * b2_pi)

/// The maximum linear translation of a body per step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this. Meters.
#define b2_maxTranslationSquared	(b2_maxTranslation * b2_maxTranslation)

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
#define b2_maxRotation				(b2Scalar(0.5) * b2_pi)
#define b2_maxRotationSquared		(b2_maxRotation * b2_maxRotation)

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
#define b2_baumgarte				b2Scalar(0.2)
#define b2_toiBaumgarte				b2Scalar(0.75)


/// Dump to a file. Only one dump file allowed at a time.
void b2OpenDump(const char* fileName);
void b2Dump(const char* string, ...);
void b2CloseDump();

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
};

/// Current version.
extern B2_API b2Version b2_version;

#endif
