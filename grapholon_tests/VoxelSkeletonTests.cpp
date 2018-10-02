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
#include "stdafx.h"
#include "CppUnitTest.h"

#include <stdlib.h>     /* srand, rand */

#include "VoxelSkeleton.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace grapholon;

#define MAX_RAND 100

namespace grapholon_tests
{		
	TEST_CLASS(VoxelSkeletonTest)
	{
	public:
		TEST_CLASS_INITIALIZE(InitRandom)
		{
			GRuint seed(1234567890);
			srand(seed);
		}

		GRuint next_uint() {
			return rand() % (MAX_RAND + 1);
		}
		
		TEST_METHOD(canCreateSkeletonAndAccessAllVoxels)
		{
			GRuint w(next_uint()), h(next_uint()), s(next_uint());

			VoxelSkeleton skeleton(w, h, s);

			for (GRuint i(0); i < w; i++) {
				for (GRuint j(0); j < h; j++) {
					for (GRuint k(0); k < s; k++) {
						Assert::IsFalse(skeleton.voxel(i, j, k).value_);
					}
				}
			}
		}


		TEST_METHOD(idToCoordinatesToIdIsSameId)
		{
			GRuint w(10), h(20), s(next_uint());

			VoxelSkeleton skeleton(w, h, s);

			for (GRuint i(0); i < w*h*s; i++) {
				GRuint x, y, z;
				skeleton.voxel_id_to_coordinates(i, x, y, z);
				Assert::AreEqual(i, skeleton.voxel_coordinates_to_id(x,y,z));
			}
		}

	};
}