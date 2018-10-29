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

//#include "stdafx.h"
#include "pch.h"
#include <iostream>
#include <bitset>
#include <ctime>

#include "Curve.hpp"
#include "VoxelSkeleton.hpp"

using namespace std;
using namespace grapholon;


void sand_box() {
	GRuint w(100), h(100), s(100);

	VoxelSkeleton skeleton(w, h, s);

	skeleton.generate_random_skeleton_like(5000, 15164);

	skeleton.compute_voxel_attributes();

	std::cout << "total number of voxels : " << skeleton.true_voxels().size() << std::endl;


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



void CliquesInBertrandStructureTest() {
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

	std::vector<std::vector<std::vector<GRuint>>> critical_cliques;
	skeleton->extract_all_cliques(critical_cliques);


	GRuint x, y, z;

	std::cout << "critical cliques found in Bertrand structure : " << std::endl << endl;

	std::cout << " critical 3-cliques : " << critical_cliques[3].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[3].size(); i++) {
		skeleton->voxel_id_to_coordinates(critical_cliques[3][i][0], x, y, z);
		std::cout << " clique " << i << " : ( " << x << " " << y << " " << z << " )" << endl;
	}
	cout << endl;

	std::cout << " critical 2-cliques : " << critical_cliques[2].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[2].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[2][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[2][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 1-cliques : " << critical_cliques[1].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[1].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[1][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[1][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 0-cliques : " << critical_cliques[0].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[0].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[0][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[0][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;



	delete skeleton;
}



void interiorBlockTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(8, 8, 8);

	for (GRuint i(0); i < 4; i++) {
		for (GRuint j(0); j < 3; j++) {
			for (GRuint k(0); k < 3; k++) {
				skeleton->set_voxel(i, j, k);
			}
		}
	}

	std::vector<std::vector<std::vector<GRuint>>> critical_cliques;
	skeleton->extract_all_cliques(critical_cliques);


	GRuint x, y, z;

	std::cout << "critical cliques found in block structure : " << std::endl << endl;

	std::cout << " critical 3-cliques : " << critical_cliques[3].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[3].size(); i++) {
		skeleton->voxel_id_to_coordinates(critical_cliques[3][i][0], x, y, z);
		std::cout << " clique " << i << " : ( " << x << " " << y << " " << z << " )" << endl;
	}
	cout << endl;

	std::cout << " critical 2-cliques : " << critical_cliques[2].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[2].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[2][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[2][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 1-cliques : " << critical_cliques[1].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[1].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[1][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[1][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;

	std::cout << " critical 0-cliques : " << critical_cliques[0].size() << std::endl;
	for (GRuint i(0); i < critical_cliques[0].size(); i++) {
		cout << "clique " << i << " : " << endl;
		for (GRuint j(0); j < critical_cliques[0][i].size(); j++) {
			skeleton->voxel_id_to_coordinates(critical_cliques[0][i][j], x, y, z);
			std::cout << "( " << x << " " << y << " " << z << " )" << endl;
		}
		cout << endl;
	}
	cout << endl;



	delete skeleton;
}



void blockCritical2Clique() {
		VoxelSkeleton* skeleton = new VoxelSkeleton(8, 8, 8);

		for (GRuint i(0); i < 4; i++) {
			for (GRuint j(0); j < 3; j++) {
				for (GRuint k(0); k < 3; k++) {
					skeleton->set_voxel(i, j, k);
				}
			}
		}

		std::vector<std::vector<std::vector<GRuint>>> critical_cliques;

		skeleton->extract_all_cliques(critical_cliques);


		GRuint x, y, z;

		std::cout << " critical 2-cliques : " << critical_cliques[2].size() << std::endl;
		for (GRuint i(0); i < critical_cliques[2].size(); i++) {
			cout << "clique " << i << " : " << endl;
			for (GRuint j(0); j < critical_cliques[2][i].size(); j++) {
				skeleton->voxel_id_to_coordinates(critical_cliques[2][i][j], x, y, z);
				std::cout << "( " << x << " " << y << " " << z << " )" << endl;
			}
			cout << endl;
		}
		cout << endl;

	delete skeleton;
}

void BertrandStructureThinningTest() {
	VoxelSkeleton* skeleton = VoxelSkeleton::BertrandStructure();

	cout << "voxels before thinning : " <<skeleton->true_voxels().size()<< std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}


	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::ManualTipSkel);

	cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	delete skeleton;
}


void InteriorBlockThinningTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(8, 8, 8);

	for (GRuint i(0); i < 4; i++) {
		for (GRuint j(0); j < 3; j++) {
			for (GRuint k(0); k < 3; k++) {
				skeleton->set_voxel(i, j, k);
			}
		}
	}

	cout << "voxels before thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}


	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::AlwaysFalseSkel);

	cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	delete skeleton;
}


void SkeletonLikeThinningTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(100, 100, 100);

	skeleton->generate_random_skeleton_like(1000);

	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::AlwaysFalseSkel);

	cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	delete skeleton;
}



void predefSkeletonTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(98, 98, 98);

	skeleton->generate_random(10, 102);

	cout << "voxels before thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::AlwaysFalseSkel);

	cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	delete skeleton;
}



void nonCliqueConfigTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(98, 98, 98);

	skeleton->set_voxel(5, 5, 5);

	skeleton->set_voxel(6, 5, 5);
	skeleton->set_voxel(7, 5, 5);
	skeleton->set_voxel(8, 5, 5);


	skeleton->set_voxel(5, 6, 5);
	skeleton->set_voxel(5, 7, 5);
	skeleton->set_voxel(5, 8, 5);

	skeleton->set_voxel(5, 5, 6);
	skeleton->set_voxel(5, 5, 7);
	skeleton->set_voxel(5, 5, 8);

	cout << "voxels before thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
		if (skeleton->is_1_isthmus(x, y, z)) {
			std::cout << "	is a 1-isthmus " << std::endl;
		}
	}


	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);

	cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}

	delete skeleton;
}


void find_wrong_skeletonization() {
	GRuint wrong_count(0);
	GRuint nb_trials(100);

	for (GRuint i(0); i < nb_trials; i++) {
		VoxelSkeleton* skeleton = new VoxelSkeleton(98, 98, 98);

		GRuint seed(i);
		skeleton->generate_random(1000, seed);

		std::cout << "checking seed : " << seed << std::endl;

		skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);

		std::cout <<  " size : " << skeleton->true_voxels().size() << std::endl;
		if (! skeleton->is_k_connected(skeleton->true_voxels(),0u)) {
			cout << endl;
			cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;
			for (GRuint j(0); j < skeleton->true_voxels().size(); j++) {
				GRuint x, y, z;
				skeleton->voxel_id_to_coordinates(skeleton->true_voxels()[j], x, y, z);
				cout << "( " << x << " " << y << " " << z << ")" << endl;
			}
			std::cout << " seed : " << seed << std::endl << std::endl;;
			wrong_count++;
		}

		/*for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
		}*/

		delete skeleton;
	}
	std::cout << "done. found " << wrong_count << " wrong skeletons among " << nb_trials << std::endl;

}


void DoubleThinningTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(98, 98, 98);

	skeleton->generate_random(400, 1234);
	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel, true);

	std::cout << " size after first thinning : " << skeleton->true_voxels().size() << std::endl;
	std::cout << "	connectedness : " << skeleton->is_k_connected(skeleton->true_voxels(), 0u) << std::endl;;

	for (GRuint j(0); j < skeleton->true_voxels().size(); j++) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(skeleton->true_voxels()[j], x, y, z);
		//cout << "skeleton->set_voxel( " << x << "," << y << "," << z << ");" << endl;
	}

	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel, true);



	std::cout << " size after second thinning : " << skeleton->true_voxels().size() << std::endl;
	std::cout << "	connectedness : " << skeleton->is_k_connected(skeleton->true_voxels(), 0u) << std::endl;;

	delete skeleton;
}


void SubdivisionTest() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(100, 100, 100);

	//skeleton->generate_random(4);

	skeleton->set_voxel(1, 1, 1);
	

	VoxelSkeleton* subdivided_skeleton1 = skeleton->subdivide(1);
	VoxelSkeleton* subdivided_skeleton2 = skeleton->subdivide(2);
	VoxelSkeleton* subdivided_skeleton3 = skeleton->subdivide(3);

	std::cout << " skeleton 1 : " << std::endl;
	for (GRuint j(0); j < subdivided_skeleton1->true_voxels().size(); j++) {
		GRuint x, y, z;
		subdivided_skeleton1->voxel_id_to_coordinates(subdivided_skeleton1->true_voxels()[j], x, y, z);
		
		cout << "	( " << x << "," << y << "," << z << ");" << endl;
	}

	std::cout << " skeleton 2 : " << std::endl;
	for (GRuint j(0); j < subdivided_skeleton2->true_voxels().size(); j++) {
		GRuint x, y, z;
		subdivided_skeleton2->voxel_id_to_coordinates(subdivided_skeleton2->true_voxels()[j], x, y, z);

		cout << "	( " << x << "," << y << "," << z << ");" << endl;
	}

	std::cout << " skeleton 1 : " << std::endl;
	for (GRuint j(0); j < subdivided_skeleton3->true_voxels().size(); j++) {
		GRuint x, y, z;
		subdivided_skeleton3->voxel_id_to_coordinates(subdivided_skeleton3->true_voxels()[j], x, y, z);

		cout << "	( " << x << "," << y << "," << z << ");" << endl;
	}


	delete skeleton;
	delete subdivided_skeleton1;
	delete subdivided_skeleton2;
	delete subdivided_skeleton3;

}

void CreateSkeletalGraphAndAddStuff(){

	SkeletalGraph graph(0);

	std::vector<VertexDescriptor> vertex_descriptors;
	std::vector<EdgeDescriptor> edge_descriptors;

	GRuint vertex_count(10);

	for (GRuint i(0); i < vertex_count; i++) {
		vertex_descriptors.push_back(graph.add_vertex({ {(GRfloat)i, -(GRfloat)i, (GRfloat)i * 2} }));
	}

	for (GRuint i(0); i < vertex_count; i++) {
		edge_descriptors.push_back(graph.add_edge(vertex_descriptors[rand() % vertex_count], vertex_descriptors[rand() % vertex_count], {}).first);
	}


	std::cout<<graph.to_string();
}


void ExtractGraphFromTestStructure2() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(10, 10, 10);

	skeleton->set_voxel(1, 1, 0);

	skeleton->set_voxel(2, 3, 0);
	skeleton->set_voxel(2, 4, 0);

	skeleton->set_voxel(3, 2, 0);
	skeleton->set_voxel(3, 5, 0);
	skeleton->set_voxel(3, 9, 0);

	skeleton->set_voxel(4, 2, 0);
	skeleton->set_voxel(4, 4, 0);
	skeleton->set_voxel(4, 7, 0);
	skeleton->set_voxel(4, 8, 0);

	skeleton->set_voxel(5, 3, 0);

	skeleton->set_voxel(6, 2, 0);
	skeleton->set_voxel(6, 4, 0);
	skeleton->set_voxel(6, 5, 0);
	skeleton->set_voxel(6, 6, 0);

	skeleton->set_voxel(7, 2, 0);
	skeleton->set_voxel(7, 7, 0);

	skeleton->set_voxel(8, 1, 0);
	skeleton->set_voxel(8, 3, 0);

	SkeletalGraph* graph = skeleton->extract_skeletal_graph();

	std::cout<<graph->to_string();

	delete graph;
	delete skeleton;
}

void ExtractGraphFromTestStructure3() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(10, 10, 10);

	skeleton->set_voxel(0, 7, 0);

	skeleton->set_voxel(1, 6, 0);

	skeleton->set_voxel(2, 2, 0);
	skeleton->set_voxel(2, 3, 0);
	skeleton->set_voxel(2, 5, 0);
	skeleton->set_voxel(2, 7, 0);

	skeleton->set_voxel(3, 1, 0);
	skeleton->set_voxel(3, 4, 0);
	skeleton->set_voxel(3, 7, 0);

	skeleton->set_voxel(4, 1, 0);
	skeleton->set_voxel(4, 3, 0);
	skeleton->set_voxel(4, 5, 0);
	skeleton->set_voxel(4, 7, 0);

	skeleton->set_voxel(5, 1, 0);
	skeleton->set_voxel(5, 3, 0);
	skeleton->set_voxel(5, 6, 0);
	skeleton->set_voxel(5, 8, 0);

	skeleton->set_voxel(6, 1, 0);
	skeleton->set_voxel(6, 3, 0);
	skeleton->set_voxel(6, 5, 0);
	skeleton->set_voxel(6, 8, 0);

	skeleton->set_voxel(7, 2, 0);
	skeleton->set_voxel(7, 4, 0);
	skeleton->set_voxel(7, 6, 0);
	skeleton->set_voxel(7, 8, 0);

	skeleton->set_voxel(8, 4, 0);
	skeleton->set_voxel(8, 7, 0);

	skeleton->set_voxel(9, 5, 0);
	skeleton->set_voxel(9, 6, 0);

	SkeletalGraph* graph = skeleton->extract_skeletal_graph();

	std::cout << graph->to_string();

	delete graph;
	delete skeleton;
}

void ExtractGraphFromLoop() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(10, 10, 10);

	skeleton->set_voxel(0, 2, 0);

	skeleton->set_voxel(1, 1, 0);
	skeleton->set_voxel(1, 3, 0);

	skeleton->set_voxel(2, 0, 0);
	skeleton->set_voxel(2, 4, 0);

	skeleton->set_voxel(3, 1, 0);
	skeleton->set_voxel(3, 3, 0);

	skeleton->set_voxel(4, 2, 0);

	SkeletalGraph* graph = skeleton->extract_skeletal_graph();

	std::cout << graph->to_string();

	delete graph;
	delete skeleton;
}

void ExtractGraphFromDoubleLoop() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(10, 10, 10);

	skeleton->set_voxel(0, 2, 0);

	skeleton->set_voxel(1, 1, 0);
	skeleton->set_voxel(1, 3, 0);

	skeleton->set_voxel(2, 0, 0);
	skeleton->set_voxel(2, 4, 0);

	skeleton->set_voxel(3, 1, 0);
	skeleton->set_voxel(3, 3, 0);

	skeleton->set_voxel(4, 2, 0);

	skeleton->set_voxel(5, 1, 0);
	skeleton->set_voxel(5, 3, 0);

	skeleton->set_voxel(6, 0, 0);
	skeleton->set_voxel(6, 4, 0);

	skeleton->set_voxel(7, 1, 0);
	skeleton->set_voxel(7, 3, 0);

	skeleton->set_voxel(8, 2, 0);

	SkeletalGraph* graph = skeleton->extract_skeletal_graph();

	std::cout << graph->to_string();

	delete graph;
	delete skeleton;
}

void ExtractGraphFromRandomSructure() {
	VoxelSkeleton* skeleton = new VoxelSkeleton(100, 100, 100);
	skeleton->generate_random(400,1234);
	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel, false);

	std::cout << " voxel count after thinning : " << skeleton->set_voxel_count() << std::endl;

	SkeletalGraph* graph = skeleton->extract_skeletal_graph();


	std::cout << graph->to_string();

	delete graph;
	delete skeleton;
}


void MovingAverageDiscreteCurve() {

	DiscreteCurve discrete_curve;
	DiscreteCurve averaged_curve;
	
	GRuint length(10);
	GRuint nb_samples(10);
	GRfloat increment((GRfloat)length / (GRfloat)nb_samples);

	for (GRuint i(0); i < nb_samples; i++) {
		GRfloat x = increment * i;
		GRfloat y = (std::sin(increment*i) + 1) * 0.5f * length;
		discrete_curve.push_back(Vector3f(x, y, 0));
	}

	averaged_curve = discrete_curve;

	averaged_curve.smooth_moving_average(5);

	std::cout << "base    : " << std::endl << discrete_curve.to_string() << std::endl;
	std::cout << "average : " << std::endl << averaged_curve.to_string() << std::endl;

}

void MovingAverageDiscreteCurve2() {
	DiscreteCurve discrete_curve;
	DiscreteCurve averaged_curve3;
	DiscreteCurve averaged_curve5;

	GRuint length(10);

	for (GRuint i(0); i < length; i++) {
		discrete_curve.push_back(Vector3f((GRfloat)i, 0.f, 0.f));
	}
	discrete_curve[length / 2] = Vector3f((GRfloat)length / 2, 1.f, 0.f);

	averaged_curve3 = discrete_curve;
	averaged_curve5 = discrete_curve;

	averaged_curve3.smooth_moving_average(3);
	averaged_curve5.smooth_moving_average(5);

	std::cout << "base    : " << std::endl << discrete_curve.to_string() << std::endl;
	std::cout << "average 3 : " << std::endl << averaged_curve3.to_string() << std::endl;
	std::cout << "average 5 : " << std::endl << averaged_curve5.to_string() << std::endl;
}

void CurveFittinSpline() {
	DiscreteCurve discrete_curve;

	GRuint nb_points(100);

	for (GRuint i(0); i < nb_points; i++) {
		discrete_curve.push_back(Vector3f((GRfloat)i, sin(3*i /(GRfloat)nb_points), 0));
	}

	SplineCurve* spline = discrete_curve.to_spline_curve(DiscreteCurve::CURVE_FITTING);

	std::cout << "base    : " << std::endl << discrete_curve.to_string() << std::endl;
	std::cout << "spline  : " << std::endl << spline->to_string() << std::endl;

}


void movingAverageSmoothTest() {

	VoxelSkeleton skeleton(100, 100, 100);

	skeleton.generate_random(10000, 1234);

	cout << "count before : " << skeleton.set_voxel_count() << endl;

	VoxelSkeleton* smoothed1 = skeleton.smooth_moving_average(0, 0.01f);

	cout << "count after smoothing with_window width = 1 : " << smoothed1->set_voxel_count() << endl;

	VoxelSkeleton* smoothed2 = skeleton.smooth_moving_average(1, 0.5f);

	cout << "count after smoothing with_window width = 1, thresh = 0.5 : " << smoothed2->set_voxel_count() << endl;

	VoxelSkeleton* smoothed3 = skeleton.smooth_moving_average(3, 0.1f);

	cout << "count after smoothing with_window width = 5, thresh = 0.5 : " << smoothed3->set_voxel_count() << endl;
}

int main()
{
	
	movingAverageSmoothTest();

	while (true);
    return 0;
}

