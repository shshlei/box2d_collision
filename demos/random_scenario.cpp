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

/* Author: Shi Shenglei */

#include "box2d_collision/b2_bvh_manager.h"
#include <stdio.h>

int main()
{
    b2BVHManager manager;

    // Define the ground box shape.
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0, 10.0);
    manager.AddBody("box1", &groundBox, false);

    // Define another box shape for our dynamic body.
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0, 1.0);
    manager.AddBody("box2", &dynamicBox, true);

    b2Transform boxxf(b2Vec2(0.0, -11.1), 0.0);
    manager.SetBodyTransform("box1", boxxf);

    bool collision = manager.ContactTest();
    printf("Collision: %d\n", collision);

    return 0;
}
