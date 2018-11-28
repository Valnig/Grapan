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
	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);

	std::cout << " size after first thinning : " << skeleton->true_voxels().size() << std::endl;
	std::cout << "	connectedness : " << skeleton->is_k_connected(skeleton->true_voxels(), 0u) << std::endl;;

	for (GRuint j(0); j < skeleton->true_voxels().size(); j++) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(skeleton->true_voxels()[j], x, y, z);
		//cout << "skeleton->set_voxel( " << x << "," << y << "," << z << ");" << endl;
	}

	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);



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
	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);

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






void movingAverageSmoothSkeletonTest() {

	VoxelSkeleton skeleton(100, 100, 100);

	skeleton.generate_random_skeleton_like(1000, 1234);

	VoxelSkeleton* subdivided1 = skeleton.subdivide(2);
	VoxelSkeleton* subdivided2 = subdivided1->subdivide(2);
	cout << "count before : " << subdivided2->set_voxel_count() << endl;

	VoxelSkeleton* smoothed = subdivided2->smooth_moving_average(0, 0.01f);
	cout << "count after smoothing with distance = 0 : " << smoothed->set_voxel_count() << endl;
	
	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(1, 0.8f);
	cout << "count after smoothing  with distance = 1, thresh = 0.8 : " << smoothed->set_voxel_count() << endl;

	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(1, 0.5f);
	cout << "count after smoothing  with distance = 1, thresh = 0.5 : " << smoothed->set_voxel_count() << endl;

	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(1, 0.1f);
	cout << "count after smoothing  with distance = 1, thresh = 0.1 : " << smoothed->set_voxel_count() << endl;

	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(2, 0.8f);
	cout << "count after smoothing  with distance = 2, thresh = 0.8 : " << smoothed->set_voxel_count() << endl;

	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(2, 0.5f);
	cout << "count after smoothing  with distance = 2, thresh = 0.5 : " << smoothed->set_voxel_count() << endl;

	delete smoothed;
	smoothed = subdivided2->smooth_moving_average(2, 0.1f);
	cout << "count after smoothing  with distance = 2, thresh = 0.1 : " << smoothed->set_voxel_count() << endl;

	delete subdivided1;
	delete subdivided2;
	delete smoothed;
}


void graphExtractionOnSkeletonLikeAndCurveFitting() {
	VoxelSkeleton* base_skeleton = new VoxelSkeleton(100, 100, 100);
	base_skeleton->generate_random_skeleton_like(1000, 12134);
	std::cout << "generated voxel count : " << base_skeleton->set_voxel_count() << std::endl;

	VoxelSkeleton* fit_skeleton = base_skeleton->fit_to_min_max();
	std::cout << "fit skeleton dimensions : " << fit_skeleton->width() << " " << fit_skeleton->height() << " " << fit_skeleton->slice() << std::endl;

	VoxelSkeleton* subdivided = fit_skeleton->subdivide(2);
	cout << "subdivided voxel count : " << subdivided->set_voxel_count() << std::endl;

	VoxelSkeleton* smoothed = subdivided->smooth_moving_average(1, 0.5f);
	cout << "smoothed voxel count : " << smoothed->set_voxel_count() << endl;

	smoothed->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::OneIsthmusSkel);
	cout << "skeleton voxel count : " << smoothed->set_voxel_count() << endl;

	SkeletalGraph* graph = smoothed->extract_skeletal_graph(nullptr, DiscreteCurve::CURVE_FITTING, 5, 1.f);
	cout << "resulting graph : " << graph->vertex_count()<<" vertices, "<<graph->edge_count()<<" edges and "<<graph->edge_spline_count()<<" splines" << endl;
	cout << graph->to_string() << endl;

	delete base_skeleton;
	delete fit_skeleton;
	delete subdivided;
	delete smoothed;
	delete graph;
}




void CurveFittinSpline() {
	DiscreteCurve discrete_curve;

	GRuint nb_points(100);

	for (GRuint i(0); i < nb_points; i++) {
		discrete_curve.push_back(Vector3f((GRfloat)i, sin(3 * i / (GRfloat)nb_points), 0));
	}

	GRfloat error(0.1f);

	SplineCurve* spline = discrete_curve.to_spline_curve(DiscreteCurve::FULL_CURVE);

	std::cout << "base    : " << std::endl << discrete_curve.to_string() << std::endl;
	std::cout << "size : " << discrete_curve.size() << std::endl;
	std::cout << "spline  : " << std::endl << spline->to_string() << std::endl;
	std::cout << "size : " << spline->size() << std::endl;
}


void modifiyGraphVertexPositions() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u1 = Vector3f(0, 0, 0);
	Vector3f u2 = Vector3f(6, 0, 0);
	Vector3f u3 = Vector3f(0, 6, 0);
	Vector3f u4 = Vector3f(6, 6, 0);

	Vector3f center = Vector3f(3, 3, 0);

	VertexDescriptor v1 = graph.add_vertex({ u1 });
	DiscreteCurve c1({ u1, Vector3f(1,1,0), Vector3f(2,2,0), center });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	VertexDescriptor v2 = graph.add_vertex({ u2 });
	DiscreteCurve c2({ center, Vector3f(4,2,0), Vector3f(5,1,0), u2 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	VertexDescriptor v3 = graph.add_vertex({ u3 });
	DiscreteCurve c3({ u3, Vector3f(1,5,0), Vector3f(2,4,0), center });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	VertexDescriptor v4 = graph.add_vertex({ u4 });
	DiscreteCurve c4({ center, Vector3f(4,4,0), Vector3f(5,5,0), u4 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	//center
	VertexDescriptor v_center = graph.add_vertex({ center });

	graph.add_edge(v1, v_center, e1);
	graph.add_edge(v_center, v2, e2);
	graph.add_edge(v3, v_center, e3);
	graph.add_edge(v_center, v4, e4);

	std::cout << "graph at first : " << graph.to_string() << endl;


	graph.move_and_scale(Vector3f(-1,0,0), 4.f);

	std::cout << "graph after moving and scaling : " << graph.to_string() << endl;


}


void collapseSingleEdge() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(0, 4, 0);
	Vector3f u2 = Vector3f(2, 2, 0);
	Vector3f u3 = Vector3f(4, 2, 0);
	Vector3f u4 = Vector3f(6, 0, 0);
	Vector3f u5 = Vector3f(6, 4, 0);

	Vector3f u6 = Vector3f(1, 1, 0);
	Vector3f u7 = Vector3f(1, 3, 0);
	Vector3f u8 = Vector3f(5, 1, 0);
	Vector3f u9 = Vector3f(5, 3, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });
	VertexDescriptor v5 = graph.add_vertex({ u5 });


	DiscreteCurve c0({ u0, u6, u2 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v0, v2, e0);

	DiscreteCurve c1({ u2, u7, u1 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v2, v1, e1);

	DiscreteCurve c2({ u2, u3 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse = graph.add_edge(v2, v3, e2).first;

	DiscreteCurve c3({ u3, u8, u4 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v3, v4, e3);

	DiscreteCurve c4({ u0, u6, u2 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v0, v2, e4);

	DiscreteCurve c5({ u5, u9, u3 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v5, v3, e5);

	std::cout << "graph at first : " << graph.to_string() << endl;


	graph.collapse_edge(to_collapse, SkeletalGraph::TARGET);

	std::cout << "graph after moving and scaling : " << graph.to_string() << endl;


}


void RemoveVerticesWithDegree() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(1, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 0, 0);

	Vector3f u3 = Vector3f(0, 0, 0);
	Vector3f u4 = Vector3f(0, 1, 0);
	Vector3f u5 = Vector3f(3, 0, 0);

	Vector3f u6 = Vector3f(0.5f, 0, 0);
	Vector3f u7 = Vector3f(0.5, 1, 0);
	Vector3f u8 = Vector3f(3.5, 0, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });
	VertexDescriptor v5 = graph.add_vertex({ u5 });


	DiscreteCurve c0({ u0, u1 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v1, e0).first;

	DiscreteCurve c1({ u1, u2 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v1, v2, e1).first;

	DiscreteCurve c2({ u2, u0 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v2, v0, e2).first;

	DiscreteCurve c3({ u3, u6, u0 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v3, v0, e3);

	DiscreteCurve c4({ u4, u7, u1 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v4, v1, e4);

	DiscreteCurve c5({ u2, u8, u5 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	graph.add_edge(v2, v5, e5);

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.remove_vertices_of_degree(3);

	std::cout << "graph after removing vertices of degree 1 : " << graph.to_string() << endl;
}


void mergeEdgesWhereDegree2() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 1, 0);
	Vector3f u3 = Vector3f(3, 1, 0);
	Vector3f u4 = Vector3f(4, 0, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });

	DiscreteCurve c0({ u2, u1, u0 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v2, v0, e0).first;

	DiscreteCurve c1({ u2, u3, u4 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v2, v4, e1).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.remove_vertices_of_degree_2_and_merge_edges();

	std::cout << "graph after removing vertices of degree 2 : " << graph.to_string() << endl;
}


void splitEdge() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 1, 0);
	Vector3f u3 = Vector3f(3, 1, 0);
	Vector3f u4 = Vector3f(4, 0, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });

	DiscreteCurve c0({ u0, u1, u2, u3, u4 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v0, v4, e0).first;


	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.split_edge_at(to_split,2, Vector3f(2,2,0));

	std::cout << "graph after removing vertices of degree 2 : " << graph.to_string() << endl;
}


void extrudeDiagonal() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 2, 0);
	Vector3f u3 = Vector3f(3, 3, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });

	DiscreteCurve c0({ u0, u1, u2, u3 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v0, v3, e0).first;


	std::cout << "graph at first : " << graph.to_string() << endl;

	for (GRuint i(0); i < 30; i++) {
		graph.extrude_tip_vertex(v3, Vector3f(2.f, 2.f, 0.f) + Vector3f(1.f, 1.f, 0.f)*0.1f*i, 1.f);
	}
	std::cout << "graph after removing vertices of degree 2 : " << graph.to_string() << endl;

}

void cutEdge() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 1, 0);
	Vector3f u3 = Vector3f(3, 1, 0);
	Vector3f u4 = Vector3f(4, 0, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });

	DiscreteCurve c0({ u0, u1, u2, u3, u4 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v0, v4, e0).first;


	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.cut_edge_at(to_split, 2, Vector3f(2, 2, 0));

	std::cout << "graph after removing vertices of degree 2 : " << graph.to_string() << endl;
}

void move_merge_and_move_again() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 1, 0);
	Vector3f u3 = Vector3f(3, 1, 0);
	Vector3f u4 = Vector3f(4, 0, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v4 = graph.add_vertex({ u4 });

	DiscreteCurve c0({ u0, u1, u2 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v2, e0).first;

	DiscreteCurve c1({ u2, u3, u4 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v2, v4, e1).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.update_vertex_position(v2, Vector3f(2, 2, 0));
	std::cout << "graph after moving middle vertex : " << graph.to_string() << endl;

	try {
		graph.remove_degree_2_vertex_and_merge_edges(v2);
	}catch(std::invalid_argument e){
		std::cout << std::endl << e.what() << std::endl << std::endl;
	}

	std::cout << "graph after removing middle vertex : " << graph.to_string() << endl;

	graph.update_vertex_position(v4, Vector3f(4, 2, 0));
	std::cout << "graph after moving end vertex : " << graph.to_string() << endl;
}


void move_vertex_merge_and_move_again() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(2, 0, 0);
	Vector3f u2 = Vector3f(1, 1, 0);
	Vector3f u3 = Vector3f(0, 2, 0);
	Vector3f u4 = Vector3f(1, 3, 0);
	Vector3f u5 = Vector3f(2, 2, 0);


	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v5 = graph.add_vertex({ u5 });

	DiscreteCurve c0({ u0, u2});
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v2, e0).first;

	DiscreteCurve c1({ u2, u1 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v2, v1, e1).first;

	DiscreteCurve c2({ u2, u3, u4, u5 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v2, v5, e2).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.update_vertex_position(v2, Vector3f(2, 2, 0));
	std::cout << "graph after moving v2: " << graph.to_string() << endl;

	VertexDescriptor merged = graph.merge_vertices(v2, v5, SkeletalGraph::SOURCE).first.first;
	if (merged == InternalBoostGraph::null_vertex()) {
		std::cout << std::endl << " nope that didn't work ..." << std::endl << std::endl;
		return;
	}
	if (merged == v2) {
		std::cout << "merged at v2" << std::endl;
	}
	else if (merged == v5) {
		std::cout << "merged at v5" << std::endl;
	}

	std::cout << "graph after merging v2 and v5 : " << graph.to_string() << endl;


	graph.update_vertex_position(v2, Vector3f(1, 2, 0));
	graph.update_vertex_position(v0, Vector3f(0, 2, 0));
	graph.update_vertex_position(v1, Vector3f(2, 2, 0));
	std::cout << "graph after moving v0, v1 and v2 : " << graph.to_string() << endl;


}




void collapseEdgesShorterThan() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(2, 0, 0);
	Vector3f u2 = Vector3f(3, 0, 0);
	Vector3f u3 = Vector3f(5, 0, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });


	DiscreteCurve c0({ u0, u1 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v1, e0).first;

	DiscreteCurve c1({ u1, u2 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v1, v2, e1).first;

	DiscreteCurve c2({ u2, u3 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v2, v3, e2).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.collapse_edges_shorter_than(1.2f);

	std::cout << "graph after removal of degree 2 vertices : " << graph.to_string() << endl;
}


void modifiyGraphVertexPositions3Points() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u1 = Vector3f(0, 0, 0);
	Vector3f u2 = Vector3f(1, 1, 0);
	Vector3f u3 = Vector3f(2, 2, 0);
	Vector3f u4 = Vector3f(3, 1, 0);
	Vector3f u5 = Vector3f(4, 0, 0);

	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v5 = graph.add_vertex({ u5 });

	DiscreteCurve c1({ u1, u2, u3, u4, u5 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });


	graph.add_edge(v1, v5, e1);


	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.update_vertex_position(v5, { Vector3f(4,2,0) });

	std::cout << "graph after moving center : " << graph.to_string() << endl;
}



void findCycle() {

	VoxelSkeleton skeleton(100, 100, 100);
	skeleton.generate_artificial_simple_kissing(10);

	SkeletalGraph* graph = skeleton.extract_skeletal_graph(nullptr,DiscreteCurve::START_AND_END, 1, 0);

	std::cout << "graph at first : " << graph->to_string() << endl;

	graph->find_cycles();

	graph->print_cycles();

	delete graph;
}

void findCycles() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 1, 0);
	Vector3f u3 = Vector3f(3, 0, 0);
	Vector3f u4 = Vector3f(4, 1, 0);
	Vector3f u5 = Vector3f(1, 2, 0);
	Vector3f u6 = Vector3f(2, 2, 0);
	Vector3f u7 = Vector3f(4, 2, 0);
	Vector3f u8 = Vector3f(3, 3, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v6 = graph.add_vertex({ u6 });
	VertexDescriptor v8 = graph.add_vertex({ u8 });


	DiscreteCurve c0({ u0, u1 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v1, e0).first;

	DiscreteCurve c1({ u1, u5, u6 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v1, v6, e1).first;

	DiscreteCurve c2({ u2, u1 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v2, v1, e2).first;


	DiscreteCurve c3({ u2, u6 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse3 = graph.add_edge(v2, v6, e3).first;

	DiscreteCurve c4({ u2, u3, u4, u7, u8 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse4 = graph.add_edge(v2, v8, e4).first;


	DiscreteCurve c5({ u2, u8 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	DiscreteCurve c6({ u6, u8 });
	EdgeProperties e6({ *c6.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	//EdgeDescriptor to_collapse6 = graph.add_edge(v2, v1, e6).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	graph.find_cycles();
	graph.print_cycles();

	graph.remove_edge(to_collapse3);

	EdgeDescriptor to_collapse5 = graph.add_edge(v2, v8, e5).first;


	graph.find_cycles();
	graph.print_cycles();
}



void findCycleInSinusoidal() {
	VoxelSkeleton skeleton(100, 100, 100);
	skeleton.generate_sinusoidal_skeleton();

	SkeletalGraph* graph = skeleton.extract_skeletal_graph(nullptr,DiscreteCurve::START_AND_END, 1, 0);

	std::cout << "graph at first : " << graph->to_string() << endl;

	graph->find_cycles();

	graph->print_cycles();

	delete graph;

}


void CutSimpleEdge() {

	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u1 = Vector3f(0, 0, 0);
	Vector3f u2 = Vector3f(1, 1, 0);
	Vector3f u3 = Vector3f(2, 2, 0);

	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });

	std::cout << "v1 : " << v1 << std::endl;
	std::cout << "v3 : " << v3 << std::endl;

	DiscreteCurve c1({ u1, u2, u3 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });

	EdgeDescriptor edge_to_cut = graph.add_edge(v1, v3, e1).first;
	std::cout << "edge to cut : " << edge_to_cut << std::endl;

	std::cout << "graph at first : " << graph.to_string() << endl;

	std::pair<VertexPair, EdgePair> result = graph.cut_edge_at(edge_to_cut, 1, Vector3f(1.5f, 1.5f, 0.f));
	VertexDescriptor new_vertex_left = result.first.first;
	VertexDescriptor new_vertex_right = result.first.second;

	EdgeDescriptor new_edge_left = result.second.first;
	EdgeDescriptor new_edge_right = result.second.second;


	//graph.split_edge_at(edge_to_cut, 0, Vector3f())

	std::cout << "graph after moving center : " << graph.to_string() << endl;

	std::cout << "vertex one is " << v1 << std::endl;
    std::cout << "vertex two is " << v3 << std::endl;
	std::cout << "new vertex left is : " << new_vertex_left << std::endl;
	std::cout << "new vertex right is : " << new_vertex_right << std::endl;

	std::cout << "new edge left is : " << new_edge_left << std::endl;
	std::cout << "new edge right is : " << new_edge_right << std::endl;
}


void SkeletonLikeThinningAndGraphExtraction() {
	VoxelSkeleton* original_skeleton = new VoxelSkeleton(100, 100, 100);
	original_skeleton->generate_random(200);

	VoxelSkeleton* skeleton = original_skeleton->copy();

	skeleton->AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::AlwaysFalseSkel);

	cout << "voxel count before thinning : " << original_skeleton->set_voxel_count() << endl;
	cout << "voxel count after thinning  : " << skeleton->set_voxel_count() << endl;

	/*cout << "voxels after thinning : " << skeleton->true_voxels().size() << std::endl;

	for (auto voxel_id : skeleton->true_voxels()) {
		GRuint x, y, z;
		skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);
		std::cout << "	( " << x << " " << y << " " << z << " )" << std::endl;
	}*/

	SkeletalGraph* graph = skeleton->extract_skeletal_graph(original_skeleton);

	cout << "resulting graph : " << graph->to_string() << std::endl;


	delete original_skeleton;
	delete skeleton;
	delete graph;
}



void splitEdgeAlongCurve() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 2, 0);
	Vector3f u3 = Vector3f(0, 4, 0);
	Vector3f u4 = Vector3f(1, 3, 0);
	Vector3f u5 = Vector3f(3, 2, 0);
	Vector3f u6 = Vector3f(4, 2, 0);
	Vector3f u7 = Vector3f(5, 1, 0);
	Vector3f u8 = Vector3f(6, 0, 0);
	Vector3f u9 = Vector3f(5, 3, 0);
	Vector3f u10 = Vector3f(6, 4, 0);
	Vector3f u11 = Vector3f(6, 2, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });
	VertexDescriptor v6 = graph.add_vertex({ u6 });
	VertexDescriptor v8 = graph.add_vertex({ u8 });
	VertexDescriptor v10 = graph.add_vertex({ u10 });
	VertexDescriptor v11 = graph.add_vertex({ u11 });


	DiscreteCurve c0({ u0, u1, u2 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v2, e0).first;

	DiscreteCurve c1({ u2, u4, u3 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v2, v3, e1).first;

	DiscreteCurve c2({ u2, u5, u6 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v2, v6, e2).first;


	DiscreteCurve c3({ u6, u7, u8 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v6, v8, e3).first;

	DiscreteCurve c4({ u10, u9, u6 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse3 = graph.add_edge(v10, v6, e4).first;

	DiscreteCurve c5({ u6, u11 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_remain = graph.add_edge(v6, v11, e5).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	std::pair<std::pair<VertexVector, EdgeVector>, std::pair<VertexVector, EdgeVector>> result = graph.split_edge_along_curve(to_split, { });

	std::cout << "graph after split : " << graph.to_string() << endl;

	std::cout << "removed stuff : " << std::endl;
	std::cout << " vertices : " << std::endl;
	for (auto vertex : result.first.first) {
		std::cout << vertex << std::endl;
	}
	std::cout << " edges : " << std::endl;
	for (auto edge : result.first.second) {
		std::cout << edge << std::endl;
	}

	std::cout << "added stuff : " << std::endl;
	std::cout << " vertices : " << std::endl;
	for (auto vertex : result.second.first) {
		std::cout << vertex << std::endl;
	}
	std::cout << " edges : " << std::endl;
	for (auto edge : result.second.second) {
		std::cout << edge << std::endl;
	}
}

void splitSingleEdgeAlongCurve() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(0, 1, 0);
	Vector3f u2 = Vector3f(1, 0, 0);
	Vector3f u3 = Vector3f(1, 1, 0);
	

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v1 = graph.add_vertex({ u1 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });


	DiscreteCurve c0({ u0, u1});
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v0, v1, e0).first;

	DiscreteCurve c1({ u2, u3});
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	//EdgeDescriptor to_leave = graph.add_edge(v2, v3, e1).first;

	std::cout << "graph at first : " << graph.to_string() << endl;

	std::pair<std::pair<VertexVector, EdgeVector>, std::pair<VertexVector, EdgeVector>> result = graph.split_edge_along_curve(to_split, {});

	std::cout << "graph after split : " << graph.to_string() << endl;

	std::cout << "removed stuff : " << std::endl;
	std::cout << " vertices : " << std::endl;
	for (auto vertex : result.first.first) {
		std::cout << vertex << std::endl;
	}
	std::cout << " edges : " << std::endl;
	for (auto edge : result.first.second) {
		std::cout << edge << std::endl;
	}

	std::cout << "added stuff : " << std::endl;
	std::cout << " vertices : " << std::endl;
	for (auto vertex : result.second.first) {
		std::cout << vertex << std::endl;
	}
	std::cout << " edges : " << std::endl;
	for (auto edge : result.second.second) {
		std::cout << edge << std::endl;
	}
}


void shortestPath() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 2, 0);
	Vector3f u3 = Vector3f(0, 4, 0);
	Vector3f u4 = Vector3f(1, 3, 0);
	Vector3f u5 = Vector3f(3, 2, 0);
	Vector3f u6 = Vector3f(4, 2, 0);
	Vector3f u7 = Vector3f(5, 1, 0);
	Vector3f u8 = Vector3f(6, 0, 0);
	Vector3f u9 = Vector3f(5, 3, 0);
	Vector3f u10 = Vector3f(6, 4, 0);
	Vector3f u11 = Vector3f(6, 2, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });
	VertexDescriptor v6 = graph.add_vertex({ u6 });
	VertexDescriptor v8 = graph.add_vertex({ u8 });
	VertexDescriptor v10 = graph.add_vertex({ u10 });
	VertexDescriptor v11 = graph.add_vertex({ u11 });


	DiscreteCurve c0({ u0, u1, u2 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse0 = graph.add_edge(v0, v2, e0).first;

	DiscreteCurve c1({ u2, u4, u3 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse1 = graph.add_edge(v2, v3, e1).first;

	DiscreteCurve c2({ u2, u5, u6 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_split = graph.add_edge(v2, v6, e2).first;


	DiscreteCurve c3({ u6, u7, u8 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse2 = graph.add_edge(v6, v8, e3).first;

	DiscreteCurve c4({ u10, u9, u6 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_collapse3 = graph.add_edge(v10, v6, e4).first;

	DiscreteCurve c5({ u6, u11 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_remain = graph.add_edge(v6, v11, e5).first;

	std::cout << "graph : " << graph.to_string() << endl;

	VertexVector path = graph.shortest_path(v0, v10);

	std::cout << "path : " << std::endl;
	for (auto vertex : path) {
		std::cout << graph.get_vertex(vertex).position.to_string() << std::endl;
	}
}


void linkEdges() {
	SkeletalGraph graph;

	GRuint window_width(1);

	Vector3f u0 = Vector3f(0, 0, 0);
	Vector3f u1 = Vector3f(1, 1, 0);
	Vector3f u2 = Vector3f(2, 2, 0);
	Vector3f u3 = Vector3f(0, 4, 0);
	Vector3f u4 = Vector3f(1, 3, 0);
	Vector3f u5 = Vector3f(3, 2, 0);
	Vector3f u6 = Vector3f(4, 2, 0);
	Vector3f u7 = Vector3f(5, 1, 0);
	Vector3f u8 = Vector3f(6, 0, 0);
	Vector3f u9 = Vector3f(5, 3, 0);
	Vector3f u10 = Vector3f(6, 4, 0);

	VertexDescriptor v0 = graph.add_vertex({ u0 });
	VertexDescriptor v2 = graph.add_vertex({ u2 });
	VertexDescriptor v3 = graph.add_vertex({ u3 });
	VertexDescriptor v5 = graph.add_vertex({ u5 });
	VertexDescriptor v6 = graph.add_vertex({ u6 });
	VertexDescriptor v8 = graph.add_vertex({ u8 });
	VertexDescriptor v10 = graph.add_vertex({ u10 });


	DiscreteCurve c0({ u0, u1, u2 });
	EdgeProperties e0({ *c0.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor edge0 = graph.add_edge(v0, v2, e0).first;

	DiscreteCurve c1({ u2, u4, u3 });
	EdgeProperties e1({ *c1.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor edge1 = graph.add_edge(v2, v3, e1).first;

	/*DiscreteCurve c2({ u2, u5, u6 });
	EdgeProperties e2({ *c2.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor asdfasd = graph.add_edge(v2, v6, e2).first;*/


	DiscreteCurve c3({ u6, u7, u8 });
	EdgeProperties e3({ *c3.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor edge2 = graph.add_edge(v6, v8, e3).first;

	DiscreteCurve c4({ u10, u9, u6 });
	EdgeProperties e4({ *c4.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor edge3 = graph.add_edge(v10, v6, e4).first;

	DiscreteCurve c5({ u5, u2 });
	EdgeProperties e5({ *c5.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor to_remain = graph.add_edge(v5, v2, e5).first;

	DiscreteCurve c6({ u5, u6 });
	EdgeProperties e6({ *c6.to_spline_curve(DiscreteCurve::FULL_CURVE, &window_width) });
	EdgeDescriptor sdfd = graph.add_edge(v5, v6, e6).first;

	std::cout << "graph : " << graph.to_string() << endl;

	graph.link_edges(edge0, edge2);

	std::cout << "graph after : " << graph.to_string()<<std::endl;

}

int main()
{
	linkEdges();

    return 0;
}

