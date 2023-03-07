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

#include "b2_api.h"
#include "b2_scalar.h"

#include <stdarg.h>
#include <stdint.h>

#ifdef B2_DEBUG
    #define b2DEBUG
#endif

#define B2_NOT_USED(x) ((void)(x))

#define	b2_maxFloat		B2_INFINITY
#define	b2_epsilon		B2_EPSILON
#define	b2_epsilon2		B2_EPSILON * B2_EPSILON
#define b2_pi			B2_PI

#define b2_maxPolygonVertices	8

// User data

/// You can define this to inject whatever data you want in b2Body
struct B2_API b2BodyUserData
{
    b2BodyUserData()
    {
        pointer = 0;
    }

    /// For legacy compatibility
    uintptr_t pointer;
};

/// You can define this to inject whatever data you want in b2Fixture
struct B2_API b2FixtureUserData
{
    b2FixtureUserData()
    {
        pointer = 0;
    }

    /// For legacy compatibility
    uintptr_t pointer;
};

// Memory Allocation

/// Default allocation functions
B2_API B2_FORCE_INLINE void* b2Alloc_Default(int size)
{
    return malloc(size);
}

B2_API B2_FORCE_INLINE void b2Free_Default(void* mem)
{
    free(mem);
}

/// Implement this function to use your own memory allocator.
B2_FORCE_INLINE void* b2Alloc(int size)
{
    return b2Alloc_Default(size);
}

/// If you implement b2Alloc, you should also implement this function.
B2_FORCE_INLINE void b2Free(void* mem)
{
    b2Free_Default(mem);
}

#endif
