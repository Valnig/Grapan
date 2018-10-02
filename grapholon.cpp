//Copyright (c) 2018 Valentin NIGOLIAN
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.
//
//

// grapholon.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include "VoxelSkeleton.hpp"

using namespace std;
using namespace grapholon;



int main()
{
	GRuint w(10), h(10), s(10);

	VoxelSkeleton skeleton(w,h,s);

	for (GRuint i(0); i < w; i++) {
		cout << " width " << i << " : " << endl;
		for (GRuint j(0); j < h; j++) {
			for (GRuint k(0); k < s; k++) {
				std::cout << "voxel " << i << "," << j << "," << k << " : " << skeleton.voxel(i,j,k).value_<< std::endl;
			}
			cout << endl;
		}
		cout << endl << endl;
	}


	while (true);
    return 0;
}

