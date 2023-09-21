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

#include "box2d_collision/b2_bvh_manager.h"
#include "box2d_collision/b2_distance.h"
#include <stdio.h>
#include <iostream>

int main()
{
    b2BVHManager manager;

    // Define the ground box shape.
    b2RectangleShape groundBox(50.0, 10.0);
    manager.AddBody("box1", &groundBox);

    // Define another box shape for our dynamic body.
    b2RectangleShape dynamicBox(1.0, 1.0);
    manager.AddBody("box2", &dynamicBox);

    b2Transform boxxf(b2Vec2(0.0, -10.0), 0.0);
    manager.SetBodyTransform("box2", boxxf);

    bool collision = manager.ContactTest();
    printf("Collision: %d\n", collision);

    b2CircleShape circle1(b2Scalar(1.0));
    b2CircleShape circle2(b2Scalar(1.0));

    std::cout << "Signed Distance Test!" << std::endl;
    b2Scalar d;
    b2Vec2 p1, p2; 
    b2ShapeDistance dist;
    dist.SignedDistance(&circle1, b2Transform(), &circle2, b2Transform(0.0, 1.0, 0.0), &d, &p1, &p2);
    std::cout << "d = " << d << " p1 = (" << p1.x << ", " << p1.y << ") p2 = (" << p2.x << ", " << p2.y << ")" << std::endl;

    dist.SignedDistance(&circle1, b2Transform(), &circle2, b2Transform(0.0, -1.0, 0.0), &d, &p1, &p2);
    std::cout << "d = " << d << " p1 = (" << p1.x << ", " << p1.y << ") p2 = (" << p2.x << ", " << p2.y << ")" << std::endl;

    dist.SignedDistance(&circle1, b2Transform(), &circle2, b2Transform(1.0, 0.0, 0.0), &d, &p1, &p2);
    std::cout << "d = " << d << " p1 = (" << p1.x << ", " << p1.y << ") p2 = (" << p2.x << ", " << p2.y << ")" << std::endl;

    dist.SignedDistance(&circle1, b2Transform(), &circle2, b2Transform(-1.0, 0.0, 0.0), &d, &p1, &p2);
    std::cout << "d = " << d << " p1 = (" << p1.x << ", " << p1.y << ") p2 = (" << p2.x << ", " << p2.y << ")" << std::endl;

    return 0;
}
