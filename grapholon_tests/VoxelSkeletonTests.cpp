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

		TEST_METHOD(adjencyTests) {
			VoxelSkeleton skeleton(3, 3, 3);
			for (GRuint i(0); i < 9; i++) {
				skeleton.set_voxel(i);
			}

			//checking the 6-neighborhood
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 1, 1, 0));
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 1, 0, 1));
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 0, 1, 1));
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 2, 1, 1));
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 1, 2, 1));
			Assert::IsTrue(skeleton.are_2adjacent(1, 1, 1, 1, 1, 2));

			//checking the 18-neighborhood
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 1, 0, 0));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 0, 1, 0));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 2, 1, 0));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 1, 2, 0));

			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 0, 0, 1));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 2, 0, 1));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 0, 2, 1));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 2, 2, 1));

			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 1, 0, 2));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 0, 1, 2));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 2, 1, 2));
			Assert::IsTrue(skeleton.are_1adjacent(1, 1, 1, 1, 2, 2));
			
			//checking the 26-neighborhood
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 0, 0, 0));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 0, 2, 0));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 2, 0, 0));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 2, 2, 0));

			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 0, 0, 2));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 0, 2, 2));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 2, 0, 2));
			Assert::IsTrue(skeleton.are_0adjacent(1, 1, 1, 2, 2, 2));
			
			//double check :
			for (GRuint i(0); i < 26; i++) {
				if (i != 13) {//id of voxel (1,1,1)
					Assert::IsTrue(skeleton.are_0adjacent(13, i));
				}
			}
		}

		TEST_METHOD(zeroConnectednessTest) {
			VoxelSkeleton skeleton(5, 5, 5);

			skeleton.set_voxel(0, 0, 0);
			skeleton.set_voxel(0, 0, 1);
			skeleton.set_voxel(0, 0, 2);
			skeleton.set_voxel(0, 0, 3);
			skeleton.set_voxel(0, 1, 4);
			skeleton.set_voxel(0, 2, 5);

			Assert::IsTrue(skeleton.is_0connected(skeleton.true_voxels_));
		}

		TEST_METHOD(zeroNotConnectednessTest) {
			VoxelSkeleton skeleton(5, 5, 5);

			skeleton.set_voxel(0, 0, 0);
			skeleton.set_voxel(0, 0, 1);
			skeleton.set_voxel(0, 0, 2);
			skeleton.set_voxel(0, 1, 4);
			skeleton.set_voxel(0, 2, 5);

			Assert::IsFalse(skeleton.is_0connected(skeleton.true_voxels_));
		}

		TEST_METHOD(CriticalCliquesInBertrandStructure)
		{
			VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

			//critical 2-cliques
			Assert::IsTrue(skeleton->is_critical_2clique(2, 1, 1, 2, 2, 1));
			Assert::IsTrue(skeleton->is_critical_2clique(2, 2, 1, 2, 1, 1));
			Assert::IsTrue(skeleton->is_critical_2clique(3, 1, 1, 3, 2, 1));
			Assert::IsTrue(skeleton->is_critical_2clique(3, 2, 1, 3, 2, 2));

			//non-critical 2-cliques
			Assert::IsFalse(skeleton->is_critical_2clique(1, 0, 0, 1, 1, 0));
			Assert::IsFalse(skeleton->is_critical_2clique(0, 1, 2, 0, 2, 2));
			Assert::IsFalse(skeleton->is_critical_2clique(0, 2, 2, 1, 2, 2));
			Assert::IsFalse(skeleton->is_critical_2clique(2, 1, 1, 3, 1, 1));

			delete skeleton;
		}

		//TODO check symetry {A,B} - {B,A} with random voxel sets and cliques
	};
}