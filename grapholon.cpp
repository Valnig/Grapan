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
#include <bitset>

#include "VoxelSkeleton.hpp"

using namespace std;
using namespace grapholon;


void sand_box() {
	GRuint w(100), h(100), s(100);

	VoxelSkeleton skeleton(w, h, s);

	skeleton.generate_random_skeleton_like(5000, 15164);

	skeleton.compute_voxel_attributes();

	std::cout << "total number of voxels : " << skeleton.true_voxels().size() << std::endl;

	//voxel list
	/*for (GRuint i(0); i< skeleton.true_voxels_.size(); i++){
	GRuint voxel_id = skeleton.true_voxels_[i];
	GRuint x, y, z;
	skeleton.voxel_id_to_coordinates(voxel_id,x,y,z);
	if (true || skeleton.voxels_[voxel_id].topological_class_ == INTERIOR_POINT) {
	std::cout << " voxel " << voxel_id << " at : " << x << ", " << y << ", " << z << " : " << skeleton.voxels_[voxel_id].topological_class_ << std::endl;
	}
	}*/

	//ids list
	/*for (GRuint i(0); i < skeleton.true_voxels_.size(); i++) {
	std::cout << " " << skeleton.true_voxels_[i];
	}
	std::cout << endl;
	*/

	//inside count
	GRuint inside_count(0);
	for (GRuint i(0); i < skeleton.true_voxels().size(); i++) {
		inside_count += (skeleton.voxel(skeleton.true_voxels()[i]).topological_class_ == BORDER_POINT);
	}
	std::cout << "number of inside points : " << inside_count << std::endl;
}


void BertandStructureTests(){

	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();
	
	std::cout << "Bertrand's structure has a critical 2-clique at 211-221: "
		<< skeleton->is_critical_2_clique(2, 1, 1, Y_AXIS) << std::endl;

	
	std::cout << "Bertrand's structure has a critical 2-clique at 311-321: "
		<< skeleton->is_critical_2_clique(3, 1, 1, Y_AXIS) << std::endl;

	std::cout << "Bertrand's structure has a critical 2-clique at 321-322: "
		<< skeleton->is_critical_2_clique(3, 2, 1, Z_AXIS) << std::endl;

	std::cout << "Bertrand's structure has a critical 2-clique at 100-110: "
		<< skeleton->is_critical_2_clique(1, 0, 0, Y_AXIS)<<std::endl;
	std::cout << "Bertrand's structure has a critical 2-clique at 012-022: "
		<< skeleton->is_critical_2_clique(0, 1, 2, Y_AXIS) << std::endl;
	std::cout << "Bertrand's structure has a critical 2-clique at 022-122: "
		<< skeleton->is_critical_2_clique(0, 2, 2, X_AXIS) << std::endl;
	std::cout << "Bertrand's structure has a critical 2-clique at 211-311: "
		<< skeleton->is_critical_2_clique(2, 1, 1, X_AXIS) << std::endl;


	delete skeleton;

}


void TableLookupVSOnTheFlyCliqueCheck(){
	std::bitset<K2Y_CONFIGURATIONS> bitset_masks(0);

	VoxelSkeleton::precompute_K2_masks(bitset_masks);

	std::cout << std::endl << std::endl << " lookup table : " << std::endl;
	for (GRuint i(0); i < 100; i++) {
		std::cout << " " << bitset_masks[i + 1024 + 128];
	}

	std::cout << "lookup table size : " << sizeof(bitset_masks) << std::endl;

}

void SimpleVoxelTests() {
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

	for (GRuint i(0); i < skeleton->true_voxels().size(); i++) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(skeleton->true_voxels()[i], x, y, z);
		std::cout << " voxel " << x << " " << y << " " << z << " is simple : " << skeleton->is_simple(x, y, z) << std::endl;
	}

	//std::cout << " voxel  122 is simple : " << skeleton->is_simple(1, 2, 2) << std::endl;
	//not working yet

	delete skeleton;
}


void K1Tests() {
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();


	bool matches = skeleton->clique_matches_K1_mask(
		0, 0, 0, 
		0, 1, 0,
		0, 0, 1,
		0, 1, 1,
		 X_AXIS);
	std::cout << " nodes starting at 0 0 0 match the K1 mask : " << matches << std::endl;

	matches = skeleton->clique_matches_K1_mask(
		2, 1, 1,
		3, 1, 1,
		2, 1, 2,
		3, 1, 2,
		Y_AXIS);
	std::cout << " nodes starting at 2 1 1 match the K1 mask : " << matches << std::endl;

	matches = skeleton->clique_matches_K1_mask(
		2, 1, 1,
		3, 1, 1,
		2, 2, 1,
		3, 2, 1,
		Z_AXIS);
	std::cout << " nodes starting at 2 1 1 match the K1 mask : " << matches << std::endl;

	matches = skeleton->clique_matches_K1_mask(
		0, 1, 2,
		1, 1, 2,
		0, 2, 2,
		1, 2, 2,
		Z_AXIS);
	std::cout << " nodes starting at 0 1 2 match the K1 mask : " << matches << std::endl;


	delete skeleton;
}

void interiorBlockTest() {
	VoxelSkeleton skeleton(8, 8, 8);

	for (GRuint i(0); i < 4; i++) {
		for (GRuint j(0); j < 3; j++) {
			for (GRuint k(0); k < 3; k++) {
				skeleton.set_voxel(i, j, k);
			}
		}
	}

	std::vector<GRuint> critical_3_cliques;
	std::vector<std::vector<GRuint>> critical_2_cliques;
	std::vector<std::vector<GRuint>> critical_1_cliques;
	std::vector<std::vector<GRuint>> critical_0_cliques;

	for (GRuint i(0); i < skeleton.true_voxels().size(); i++) {
		GRuint voxel_id(skeleton.true_voxels()[i]);
		GRuint x, y, z;
		skeleton.voxel_id_to_coordinates(voxel_id, x, y, z);
		
		//first detect 3-cliques
		if (skeleton.is_critical_3_clique(x, y, z)) {
			critical_3_cliques.push_back(voxel_id);
		}

		//then detect 2-cliques
		for (GRuint axis(X_AXIS); axis <= Z_AXIS; axis++) {
			if (skeleton.is_critical_2_clique(x, y, z, X_AXIS)) {
				GRuint voxel_B_id(skeleton.voxel_coordinates_to_id(x + (axis==X_AXIS), y + (axis==Y_AXIS), z + (axis==Z_AXIS)));
				critical_2_cliques.push_back({ voxel_id, voxel_B_id });
			}
		}



	}

}


void FullClassificationBertrandStructureTest() {
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

	std::vector<GRuint> critical_3_cliques;
	std::vector<std::vector<GRuint>> critical_2_cliques;
	std::vector<std::vector<GRuint>> critical_1_cliques;
	std::vector<std::vector<GRuint>> critical_0_cliques;

	for (GRuint i(0); i < skeleton->true_voxels().size(); i++) {


		GRuint voxel_id(skeleton->true_voxels()[i]);
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);

		//std::cout << "classifying voxel : ( " << x << ", " << y << ", " << z << " )" << std::endl;

		//first detect 3-cliques
		if (skeleton->is_critical_3_clique(x, y, z)) {
			critical_3_cliques.push_back(voxel_id);
			//std::cout << "	it is a 3-clique " << std::endl;
		}

		//then detect 2-cliques
		for (GRuint axis(X_AXIS); axis <= Z_AXIS; axis++) {
		//	std::cout << "checkin 1-clique on axis " << axis << std::endl;
			if (skeleton->is_critical_2_clique(x, y, z, (Axis)axis)) {
				GRuint voxel_B_id(skeleton->voxel_coordinates_to_id(x + (axis == X_AXIS), y + (axis == Y_AXIS), z + (axis == Z_AXIS)));
				critical_2_cliques.push_back({ voxel_id, voxel_B_id });
				//std::cout << "		there is a 2-clique on axis " << axis << std::endl;
			}
		}

		//std::cout << "	checked 2-cliques" << std::endl << std::endl;


		//then detect 1-cliques
		for (GRuint axis(X_AXIS); axis <= Z_AXIS; axis++) {
			//std::cout << "		checking 1-clique on axis " << axis << std::endl;
			GRuint x_B(x + (axis == Y_AXIS));
			GRuint y_B(y + (axis != Y_AXIS));
			GRuint z_B(z);

			GRuint x_C(x + (axis == Z_AXIS));
			GRuint y_C(y);
			GRuint z_C(z + (axis != Z_AXIS));

			GRuint x_D(x_B + x_C - x);
			GRuint y_D(y_B + y_C - y);
			GRuint z_D(z_B + z_C - z);
			/*std::cout << "		voxels are " << std::endl;
			std::cout << "		( " << x << " " << y << " " << z << " )" << std::endl;
			std::cout << "		( " << x_B << " " << y_B << " " << z_B << " )" << std::endl;
			std::cout << "		( " << x_C << " " << y_C << " " << z_C << " )" << std::endl;
			std::cout << "		( " << x_D << " " << y_D << " " << z_D << " )" << std::endl;
			*/

			if (skeleton->is_critical_1_clique(
				x, y, z,
				x_B, y_B, z_B,
				x_C, y_C, z_C,
				x_D, y_D, z_D,
				(Axis)axis)) {

				//std::cout << "		there is a 1-clique on axis " << axis << std::endl;

				GRuint voxel_B_id(skeleton->voxel_coordinates_to_id(x_B, y_B, z_B));
				GRuint voxel_C_id(skeleton->voxel_coordinates_to_id(x_C, y_C, z_C));
				GRuint voxel_D_id(skeleton->voxel_coordinates_to_id(x_D, y_D, z_D));
				critical_1_cliques.push_back(std::vector<GRuint>());
				
				if (skeleton->voxel(voxel_id).value_) {
					critical_1_cliques.back().push_back(voxel_id);
				}
				if (skeleton->voxel(voxel_B_id).value_) {
					critical_1_cliques.back().push_back(voxel_B_id);
				}
				if (skeleton->voxel(voxel_C_id).value_) {
					critical_1_cliques.back().push_back(voxel_C_id);
				}
				if (skeleton->voxel(voxel_D_id).value_) {
					critical_1_cliques.back().push_back(voxel_D_id);
				}
			}
		}

		//std::cout << "	checked 1-cliques" << std::endl << std::endl;


		//and finally 0-cliques
		for (GRuint j(0); j < 2; j++) {
			for (GRuint k(0); k < 2; k++) {

				GRuint x_A(x - j);
				GRuint y_A(y - 1);
				GRuint z_A(z - k);
				
				GRuint x_B(x_A);
				GRuint y_B(y_A + 1);
				GRuint z_B(z_A);

				GRuint x_C(x_A);
				GRuint y_C(y_A);
				GRuint z_C(z_A + 1);

				GRuint x_D(x_A);
				GRuint y_D(y_A + 1);
				GRuint z_D(z_A + 1);

				GRuint x_E(x_A + 1);
				GRuint y_E(y_A);
				GRuint z_E(z_A);

				GRuint x_F(x_A + 1);
				GRuint y_F(y_A + 1);
				GRuint z_F(z_A);

				GRuint x_G(x_A + 1);
				GRuint y_G(y_A);
				GRuint z_G(z_A + 1);

				GRuint x_H(x_A + 1);
				GRuint y_H(y_A + 1);
				GRuint z_H(z_A + 1);

				/*std::cout << "		voxels are " << std::endl;
				std::cout << "		( " << x_A << " " << y_A << " " << z_A << " )" << std::endl;
				std::cout << "		( " << x_B << " " << y_B << " " << z_B << " )" << std::endl;
				std::cout << "		( " << x_C << " " << y_C << " " << z_C << " )" << std::endl;
				std::cout << "		( " << x_D << " " << y_D << " " << z_D << " )" << std::endl;
				std::cout << "		( " << x_E << " " << y_E << " " << z_E << " )" << std::endl;
				std::cout << "		( " << x_F << " " << y_F << " " << z_F << " )" << std::endl;
				std::cout << "		( " << x_G << " " << y_G << " " << z_G << " )" << std::endl;
				std::cout << "		( " << x_H << " " << y_H << " " << z_H << " )" << std::endl;
				*/

				if (skeleton->is_critical_0_clique(
					x_A, y_A, z_A,
					x_B, y_B, z_B,
					x_C, y_C, z_C,
					x_D, y_D, z_D,
					x_E, y_E, z_E,
					x_F, y_F, z_F,
					x_G, y_G, z_G,
					x_H, y_H, z_H)) {

					GRuint voxel_A_id(skeleton->voxel_coordinates_to_id(x_A, y_A, z_A));
					GRuint voxel_B_id(skeleton->voxel_coordinates_to_id(x_B, y_B, z_B));
					GRuint voxel_C_id(skeleton->voxel_coordinates_to_id(x_C, y_C, z_C));
					GRuint voxel_D_id(skeleton->voxel_coordinates_to_id(x_D, y_D, z_D));
					GRuint voxel_E_id(skeleton->voxel_coordinates_to_id(x_E, y_E, z_E));
					GRuint voxel_F_id(skeleton->voxel_coordinates_to_id(x_F, y_F, z_F));
					GRuint voxel_G_id(skeleton->voxel_coordinates_to_id(x_G, y_G, z_G));
					GRuint voxel_H_id(skeleton->voxel_coordinates_to_id(x_H, y_H, z_H));
					critical_0_cliques.push_back(std::vector<GRuint>());

					if (skeleton->voxel(voxel_A_id).value_) {
						critical_0_cliques.back().push_back(voxel_A_id);
					}
					if (skeleton->voxel(voxel_B_id).value_) {
						critical_0_cliques.back().push_back(voxel_B_id);
					}
					if (skeleton->voxel(voxel_C_id).value_) {
						critical_0_cliques.back().push_back(voxel_C_id);
					}
					if (skeleton->voxel(voxel_D_id).value_) {
						critical_0_cliques.back().push_back(voxel_D_id);
					}
					if (skeleton->voxel(voxel_E_id).value_) {
						critical_0_cliques.back().push_back(voxel_E_id);
					}
					if (skeleton->voxel(voxel_F_id).value_) {
						critical_0_cliques.back().push_back(voxel_F_id);
					}
					if (skeleton->voxel(voxel_G_id).value_) {
						critical_0_cliques.back().push_back(voxel_G_id);
					}
					if (skeleton->voxel(voxel_H_id).value_) {
						critical_0_cliques.back().push_back(voxel_H_id);
					}
				}
			
			}
		}

	}

	GRuint x, y, z;

	std::cout << "critical cliques found in Bertrand structure : " << std::endl << endl;

	std::cout << " critical 3-cliques : " << critical_3_cliques.size() << std::endl;
	for (GRuint i(0); i < critical_3_cliques.size(); i++) {
		skeleton->voxel_id_to_coordinates(critical_3_cliques[i], x, y, z);
		std::cout << " clique "<<i<<" : ( " << x << " " << y << " " << z << " )" << endl;
	}
	cout << endl;

	std::cout << " critical 2-cliques : " << critical_2_cliques.size() << std::endl;
	for (GRuint i(0); i < critical_2_cliques.size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_2_cliques[i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_2_cliques[i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 1-cliques : " << critical_1_cliques.size() << std::endl;
	for (GRuint i(0); i < critical_1_cliques.size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_1_cliques[i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_1_cliques[i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 0-cliques : " <<critical_0_cliques.size()<< std::endl;
	for (GRuint i(0); i < critical_0_cliques.size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_0_cliques[i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_0_cliques[i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;



	delete skeleton;
}

int main()
{

	FullClassificationBertrandStructureTest();
	
	/*
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

	//critical 1-cliques
	bool clique1(skeleton->is_critical_1_clique(
		0, 1, 2,
		1, 1, 2,
		0, 2, 2,
		1, 2, 2,
		Z_AXIS));
	std::cout << "clique 1  : " << clique1 << std::endl<<endl;

	bool clique2(skeleton->is_critical_1_clique(
		1, 1, 0,
		2, 1, 0,
		1, 1, 1,
		2, 1, 1,
		Y_AXIS));
	cout << "clique 2 : " << clique2<< endl;
	
	delete skeleton;
	*/
	while (true);
    return 0;
}

