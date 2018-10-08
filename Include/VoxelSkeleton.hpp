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
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <bitset>

#include "GrapholonTypes.hpp"
#include "common.hpp"

namespace grapholon {

#define NON_EXISTENT_ID (0xffff)
#define NON_EXISTENT_COORDINATE (0xffff)

#define K2Y_CONFIGURATIONS (262144) //2^18
#define K2_MASK_WIDTH 3
#define K2_MASK_HEIGHT 2
#define K2_MASK_SLICE 3


	enum VoxelState {
		visible = 0,
		fixed = 1,
		hidden = 2
	};

	struct Voxel {
		int value;
		int threshold;
		VoxelState state;
	};

	enum Clique{ NO_CLIQUE, CLIQUE3, CLIQUE2, CLIQUE1, CLIQUE0 };
	enum TopologicalClass{UNCLASSIFIED, INTERIOR_POINT, ISOLATED_POINT, BORDER_POINT, CURVES_POINT, CURVE_JUNCTION, SURFACE_CURVES_JUNCTION, SURFACE_JUNCTION, SURFACES_CURVE_JUNCTION};
	
	struct SkeletonVoxel {
		bool value_ = false;
		TopologicalClass topological_class_ = UNCLASSIFIED;
	};


	//todo
	class zero_neighbour_iterator {

	};


	class VoxelSkeleton {

	private:
		const GRuint width_;
		const GRuint height_;
		const GRuint slice_;

		const GRuint nb_voxels_;

		SkeletonVoxel* voxels_;

		std::vector<GRint> true_voxels_;


	public:

		typedef bool(VoxelSkeleton::*AdjencyFunction)(GRint, GRint);

		VoxelSkeleton(GRuint width, GRuint height, GRuint slice) : width_(width + 2), height_(height + 2), slice_(slice + 2), nb_voxels_(width_*height_*slice_) {
			voxels_ = (SkeletonVoxel*)calloc(nb_voxels_, sizeof(SkeletonVoxel));
			memset(voxels_, 0, nb_voxels_ *sizeof(SkeletonVoxel));
		}

		~VoxelSkeleton(){
			free(voxels_);
		}

		SkeletonVoxel voxel(GRint id) {
			if (id < 0 || id >= (GRint)nb_voxels_) {
				return { false, UNCLASSIFIED };
			}
			else {
				return voxels_[id];
			}
		}

		SkeletonVoxel voxel(GRint x, GRint y, GRint z) {
			return voxel(voxel_coordinates_to_id(x, y, z));
		}


		const std::vector<GRint>& true_voxels()const {
			return true_voxels_;
		}

		//id 0 is actually voxel (-1, -1, -1)
		GRint voxel_coordinates_to_id(GRint x, GRint y, GRint z) const {
			return x+1 + (y+1 + (z+1) * height_) * width_;
		}

		
		//TODO : test this with negative coordinates
		void voxel_id_to_coordinates(GRint id, GRint& x, GRint& y, GRint& z) const {
				z = id / (width_ * height_) - 1;
				GRint rem = id % (width_ * height_);
				y = rem / width_ - 1;
				x = rem % width_ - 1;
		}

		/**
		GRuint voxel_id_and_relative_coordinates_to_id(GRuint id, GRint rel_x, GRint rel_y, GRint rel_z) {
			GRuint x, y, z;
			voxel_id_to_coordinates(id, x, y, z);
			return voxel_coordinates_and_relative_coordinates_to_id(x, y, z, rel_x, rel_y, rel_z);
		}*/

		bool set_voxel(GRint id, bool value = true) {
			if (id < 0 || id >= (GRint)nb_voxels_) {
				return false;
			}

			voxels_[id].value_ = value;
			voxels_[id].topological_class_ = UNCLASSIFIED;

			if (value) {
				true_voxels_.push_back(id);
			}
			else {
				true_voxels_.erase(std::remove(true_voxels_.begin(), true_voxels_.end(), id), true_voxels_.end());
			}

			return true;
		}


		bool set_voxel(GRuint x, GRuint y, GRuint z, bool value = true) {
			return set_voxel(voxel_coordinates_to_id(x, y, z), value);
		}



		//only does interior and border points for now
		void compute_voxel_attributes() {

			for (GRuint i(0); i < true_voxels_.size(); i++) {
				GRuint voxel_id = true_voxels_[i];

				if (voxels_[voxel_id].value_) {
					GRint x, y, z;
					voxel_id_to_coordinates(voxel_id, x, y, z);

					//first check if it's a border (incomplete, only checks for voxels on the boundary)
					if (x == 0 || y == 0 || z == 0 || x == width_ - 1 || y == height_ - 1 || z == slice_ - 1) {
						voxels_[voxel_id].topological_class_ = BORDER_POINT;
					}
					else if (
						voxels_[voxel_coordinates_to_id(x + 1, y, z)].value_
						&& voxels_[voxel_coordinates_to_id(x - 1, y, z)].value_
						&& voxels_[voxel_coordinates_to_id(x, y + 1, z)].value_
						&& voxels_[voxel_coordinates_to_id(x, y - 1, z)].value_
						&& voxels_[voxel_coordinates_to_id(x, y, z + 1)].value_
						&& voxels_[voxel_coordinates_to_id(x, y, z - 1)].value_) {

						voxels_[voxel_id].topological_class_ = INTERIOR_POINT;
					}
					else {
						voxels_[voxel_id].topological_class_ = UNCLASSIFIED;
					}
				}
			}
		}


		/**This checks if the two ids correspond to 0-adjacent voxels
		NOTE : if id == id2 it will return false so this is not really 0-adjency*/
		bool are_0adjacent(GRint id, GRint id2) {
			if (id >= (GRint)nb_voxels_ || id2 >= (GRint)nb_voxels_) {
				return false;
			}
			return id + 1 + width_ + width_ * height_ == id2
				|| id + 1 + width_ - width_ * height_ == id2
				|| id + 1 - width_ + width_ * height_ == id2
				|| id + 1 - width_ - width_ * height_ == id2
				|| id - 1 + width_ + width_ * height_ == id2
				|| id - 1 + width_ - width_ * height_ == id2
				|| id - 1 - width_ + width_ * height_ == id2
				|| id - 1 - width_ - width_ * height_ == id2
				|| are_1adjacent(id, id2);
		}

		bool are_1adjacent(GRint id, GRint id2) {
			if (id >= (GRint)nb_voxels_ || id2 >= (GRint)nb_voxels_) {
				return false;
			}
			return id + 1 + width_ == id2
				|| id + 1 - width_ == id2
				|| id - 1 + width_ == id2
				|| id - 1 - width_ == id2
				|| id + 1 + width_ * height_ == id2
				|| id + 1 - width_ * height_ == id2
				|| id - 1 + width_ * height_ == id2
				|| id - 1 - width_ * height_ == id2
				|| id + width_ + width_ * height_ == id2
				|| id + width_ - width_ * height_ == id2
				|| id - width_ + width_ * height_ == id2
				|| id - width_ - width_ * height_ == id2
				|| are_2adjacent(id, id2);
		}


		bool are_2adjacent(GRint id, GRint id2) {
			if (id >= (GRint)nb_voxels_ || id2 >= (GRint)nb_voxels_) {
				return false;
			}
			return id + 1 == id2
				|| id - 1 == id2
				|| id + width_ == id2
				|| id - width_ == id2
				|| id + width_ * height_ == id2
				|| id - width_ * height_ == id2;
		}

		bool are_0adjacent(GRint x, GRint y, GRint z, GRint x2, GRint y2, GRint z2) {
			return are_0adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}
		bool are_1adjacent(GRint x, GRint y, GRint z, GRint x2, GRint y2, GRint z2) {
			return are_1adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}
		bool are_2adjacent(GRint x, GRint y, GRint z, GRint x2, GRint y2, GRint z2) {
			return are_2adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}


		bool is_k_connected(std::vector<GRint> voxel_ids, AdjencyFunction adjency_function) {
			/*std::cout << "checking connectdedness of voxels : " << std::endl;;
			for (GRuint i(0); i < voxel_ids.size(); i++) {
				GRint x, y, z;
				voxel_id_to_coordinates(voxel_ids[i], x, y, z);
				std::cout << voxel_ids[i] <<" : "<< x << " " << y << " " << z << std::endl;
			}
			std::cout << std::endl;
			*/

std::vector<bool> visited(voxel_ids.size(), false);
std::vector<bool> explored(voxel_ids.size(), false);

bool result(false);
bool uncertain(true);
GRuint last_explored_index(0);
GRuint iteration_count(0);
while (uncertain && iteration_count < visited.size()) {
	iteration_count++;

	//visit all neighbors of the last visited
	for (GRuint i(0); i < voxel_ids.size(); i++) {
		if (!visited[i] && (this->*adjency_function)(voxel_ids[last_explored_index],
			voxel_ids[i])) {
			visited[i] = true;
			//std::cout << "visited voxel : " << i << std::endl;
		}
	}
	explored[last_explored_index] = true;
	visited[last_explored_index] = true;

	bool found_unexplored(false);

	GRuint next_explored_index(0);
	while (next_explored_index < explored.size()
		&& !found_unexplored) {

		if (visited[next_explored_index] && !explored[next_explored_index]) {
			found_unexplored = true;
			last_explored_index = next_explored_index;
			//	std::cout << "found unexplored : " << next_explored_index << std::endl;
		}
		next_explored_index++;
	}

	//std::cout << "last explored index is now : " << last_explored_index << std::endl;

	//if we reached the end of the array, we check if all voxels have been visited
	if (!found_unexplored || last_explored_index == explored.size() - 1) {
		bool visited_voxels_are_also_explored(true);
		bool all_visited(true);
		for (GRuint i(0); i < visited.size(); i++) {
			if (visited[i]) {
				visited_voxels_are_also_explored &= explored[i];
			}
			all_visited &= visited[i];
		}
		if (visited_voxels_are_also_explored) {
			uncertain = false;
			result = all_visited;
		}
	}
	/*
	std::cout << "visited : ";
	for (GRuint i(0); i < visited.size(); i++) {
	std::cout << " " << visited[i];
	}
	std::cout << std::endl;
	std::cout << "explored : ";
	for (GRuint i(0); i < explored.size(); i++) {
	std::cout << " " << explored[i];
	}
	std::cout << std::endl;
	*/
}

//std::cout << " nb voxels : " << voxel_ids.size() << ", iterations : " << iteration_count << std::endl;

return result;
		}


		//NOTE : this is very greedy (O(n^2), n = voxels_ids.size()) but it is not used at runtime so it's fine 
		bool is_k_connected(std::vector<GRint> voxel_ids, GRuint k) {
			switch (k) {
			case 0: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_0adjacent);
				break;
			}
			case 1: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_1adjacent);
				break;
			}
			case 2: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_2adjacent);
				break;
			}
			default: {
				std::cerr << k << "-connectedness does not make sense with voxels. Returning false" << std::endl;
				return false;
			}
			}
		}


		bool is_reducible(std::vector<GRint> voxels_id) {
			if (voxels_id.size() == 0) {
				return false;
			}else if (voxels_id.size() == 1){
				return true;
			}
			else {
				bool x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible = false;

				for (GRuint i(0); i < voxels_id.size(); i++) {

					//first computing N0(x)
					std::vector<GRint> neighbors;
					for (GRuint j(0); j < voxels_id.size(); j++) {
						if (are_0adjacent(voxels_id[i], voxels_id[j])) {
							neighbors.push_back(voxels_id[j]);
						}
					}

					//then X without x
					std::vector<GRint> voxel_set_without_i = voxels_id;
					voxel_set_without_i.erase(
						std::remove(
							voxel_set_without_i.begin(), voxel_set_without_i.end(), voxels_id[i]),
						voxel_set_without_i.end()
					);

					x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible
						|= (is_reducible(neighbors) && is_reducible(voxel_set_without_i));
					
				}
				return x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible;
			}
		}

		bool is_simple(GRint x, GRint y, GRint z) {

			//std::cout << "checking if voxel " << x << " " << y << " " << z << " is simple " << std::endl;

			//first define the zero neighborhood*
			std::vector<GRint> zero_neighborhood_star;
			GRint neighbor_id;
			for (GRint i(0); i < 3; i++) {
				for (GRint j(0); j < 3; j++) {
					for (GRint k(0); k < 3; k++) {
						if (i != 1 || j!= 1 || k != 1) {
							neighbor_id = voxel_coordinates_to_id(x - 1 + i, y - 1 + j, z - 1 + k);
							//std::cout << "voxel : " << x - 1 + i << " " << y - 1 + j << " " << z - 1 + k;
							//std::cout << "rel : " << i << " " << j << " " << k << std::endl;
							if (voxel(neighbor_id).value_) {
								zero_neighborhood_star.push_back(neighbor_id);
							//	std::cout << " is in the neighborhood" << std::endl;
							}
							else {
								//std::cout << std::endl;
								//std::cout << " is not in the neighborhood" << std::endl;
							}
						}
					}
				}
			}

			return is_reducible(zero_neighborhood_star);
		}


		bool is_3_clique(GRuint x, GRuint y, GRuint z) {
			return !is_simple(x, y, z);
		}


		/**K_2 mask matchings. 
		\param axis 0:X-axis, 1:Y-axis, 2:Z-axis */
		bool clique_matches_K2_mask(GRuint x, GRuint y, GRuint z, GRuint axis) {

			if (axis > 2) {
				std::cerr << "wrong axis to apply K2 mask. Returning false" << std::endl;
				return false;
			}

			//compute voxel B's coordinates
			GRuint x2(x + (axis == 0));
			GRuint y2(y + (axis == 1));
			GRuint z2(z + (axis == 2));

			//first checking if the voxel and its neighbor in the axis' direction are set
			if (!voxel(x, y, z).value_ || !voxel(x2, y2, z2).value_) {
				return false;
			}


			//first create the list of voxels in {X0,...,X7, Y0,...,Y7}AND X (at most 16 voxels)
			std::vector<GRint> mask_neighborhood_intersection;

			//std::cout << "checking clique (" << x << ", " << y << ", " << z<<") - ("<<x2<<", "<<y2<<", "<<z2<<") on axis "<<axis<< std::endl;

			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 3; j++) {
					if (!(i == 1 && j == 1)) {

						GRint X_neighbor_id;
						GRint Y_neighbor_id;

						switch (axis) {
						case 0: {
							X_neighbor_id = voxel_coordinates_to_id(x,  y  - 1 + i, z  - 1 + j);
							Y_neighbor_id = voxel_coordinates_to_id(x2, y2 - 1 + i, z2 - 1 + j);
							break;
						}
						case 1: {
							X_neighbor_id = voxel_coordinates_to_id(x  - 1 + i, y,  z  - 1 + j);
							Y_neighbor_id = voxel_coordinates_to_id(x2 - 1 + i, y2, z2 - 1 + j);

							break;
						}
						case 2: {
							X_neighbor_id = voxel_coordinates_to_id(x  - 1 + i, y  - 1 + j, z);
							Y_neighbor_id = voxel_coordinates_to_id(x2 - 1 + i, y2 - 1 + j, z2);
							break;
						}
						default: {//the default shouldn't be necessary but you never know...
							std::cerr << "wrong axis to apply K2 mask. Returning false" << std::endl;
							return false;
						}
						}

						//std::cout << "checking interesection at coordinates " << x - 1 + i << ", " << y << ", " << z - 1 + j << std::endl;
						//std::cout << "X neighbor id : " << X_neighbor_id << std::endl;

						if (voxel(X_neighbor_id).value_) {
							mask_neighborhood_intersection.push_back(X_neighbor_id);
							//std::cout << "found X neighbor" << std::endl;
						}

						//std::cout << "checking interesection at coordinates " << x2 - 1 + i << ", " << y2 << ", " << z2 - 1 + j << std::endl;
						//std::cout << "Y neighbor id : " << Y_neighbor_id << std::endl;

						if (voxel(Y_neighbor_id).value_) {
							mask_neighborhood_intersection.push_back(Y_neighbor_id);
							//std::cout << "found Y neighbor" << std::endl;
						}
					}
				}
			}


			//first check if the set of intersection is empty
			if (!mask_neighborhood_intersection.size()) {
				return true;
			}

			//std::cout << "neighborhood is non-empty..." << std::endl;

			//then check if they are 0-connected 
			//(i.e. if even one voxel of the mask has no 0-neighbor
			if (!is_k_connected(mask_neighborhood_intersection, 0u)) {
				return true;
			}
			//	std::cout << "neighborhood is 0-connected" << std::endl;

			return false;
			//finally check if for each i in {0,2,4,6}, Xi or Yi is in X
			/*bool i_th_subset_is_in_neighborhood[4] = { false };
			for (GRuint i(0); i < mask_neighborhood_intersection.size(); i++) {
			if()
			}*/

			GRuint a = axis;//to lighten the expressions

			//TODO : replace these with calls to voxel()
			bool is_in_subset =
				(voxels_[voxel_coordinates_to_id(x + (a !=0), y + (a==0), z)].value_ //X0
					|| voxels_[voxel_coordinates_to_id(x2 + (a != 0), y2 + (a == 0), z2)].value_)//Y0
				&& (voxels_[voxel_coordinates_to_id(x + (a==2), y, z +  (a+=2))].value_ //X2
					|| voxels_[voxel_coordinates_to_id(x2 + (a == 2), y2, z2 + (a += 2))].value_)//Y2
				&& (voxels_[voxel_coordinates_to_id(x - (a != 0), y - (a==0), z)].value_ //X2
					|| voxels_[voxel_coordinates_to_id(x2 - (a != 0), y2 - (a == 0), z2)].value_)//Y2
				&& (voxels_[voxel_coordinates_to_id(x - (a==2), y, z - (a!=2))].value_ //X2
					|| voxels_[voxel_coordinates_to_id(x2 - (a == 2), y2, z2 - (a != 2))].value_);//Y2

			return is_in_subset;
		}


		/*Eventually this will be replaced with a mask table lookup*/
		bool is_critical_2_clique(GRuint x, GRuint y, GRuint z, GRuint axis) {
		
			return clique_matches_K2_mask(x, y, z, axis);
		}

		



		/*MASK PRECOMPUTATIONS */

		/** This takes the coordinates of the voxel A and the axis on which voxel B is 
		(always in the axis' direction so the two voxels are 
		This converts the standard K2_Y mask neighborhood into world coordinates
		after apply a rotation corresponding to the axis given in argument.
		If the axis is 1 (Y-axis) no rotation is done since it's the reference axis
		NOTE : check if loop unrolling helps*/
		GRuint extract_neighborhood_mask_value_on_axis(GRuint x, GRuint y, GRuint z, GRuint axis) {

			if (axis > 2) {
				std::cerr << " ERROR - axis should be in {0,1,2}. Returning NON_EXISTENT_ID " << std::endl;
				return NON_EXISTENT_ID;
			}

			//create bit mask
			std::bitset<18> bit_mask(0);

			//set neighborhood according to mask value
			for (GRuint i(0); i < K2_MASK_WIDTH; i++) {
				for (GRuint j(0); j < K2_MASK_HEIGHT; j++) {
					for (GRuint k(0); k < K2_MASK_SLICE; k++) {
						//std::cout << "checking bit " << i + (j + k * 2) * 3 << std::endl;

						GRint rotated_x = i - 1;
						GRint rotated_y = j;
						GRint rotated_z = k - 1;

						//rotate the relative coordinates (i,j,k) to match the axis
						switch (axis) {
						case 0: {
							rotated_x = j;
							rotated_y = i;
							break;
						}
						case 1: {
							//do nothing since the reference is the y axis
							break;
						}
						case 2: {
							rotated_y = k;
							rotated_z = j;
							break;
						}
						}

						//convert relative coordinates to world coordinates
						GRint world_x = x + rotated_x;
						GRint world_y = y + rotated_y;
						GRint world_z = z + rotated_z;

						//if the voxel at the coordinates is set then we update the mask bitset
						//NOTE this takes the root voxels ({A,B}) into account
						if (voxel(world_x, world_y, world_z).value_) {
							//std::cout << "bit " << i + (j + k * 2) * 3 << " is true " << std::endl;
							bit_mask[i + (j + k * K2_MASK_HEIGHT) * K2_MASK_WIDTH] = true;
						}
					}
				}
			}

			return bit_mask.to_ulong();
		}

		


		/** Precomputes the critical 2-cliques masks
		Basically, each mask is an integer which represents one of the 266'144 possible configuration
		for the neighborhood of a 2-clique. Each possible configuration is tested and then stored
		as a single bit in a 2^18-bits bitset. This allows to convert any neighborhood into 
		and integer 'mask' and then check if bitset[mask] is true in O(1).
		NOTE : around 15k configurations are critical 2-cliques among the 2^18 possible configs*/
		static void precompute_K2_masks(std::bitset<K2Y_CONFIGURATIONS>& critical_2_cliques_indices) {

			//std::vector<GRuint> masks;

			GRuint critical_masks_count(0);

			//this value corresponds to the mask for the isolated 2-clique
			GRuint first_valid_mask(1024 + 128);

			for (GRuint mask_value(first_valid_mask); mask_value < K2Y_CONFIGURATIONS; mask_value++) {

				//set up neighborhood skeleton
				VoxelSkeleton skeleton(10, 10, 10);

				//create bit mask
				std::bitset<18> bit_mask(mask_value);

				/*if the bits corresponding to both the center voxels ((1,0,1) and (1,1,1))
				 are not set we can skip testing this configuration*/
				if (bit_mask[7] && bit_mask[10]) {
					//set neighborhood according to mask value
					for (GRuint i(0); i < 3; i++) {
						for (GRuint j(0); j < 2; j++) {
							for (GRuint k(0); k < 3; k++) {
								//std::cout << "checking bit " << i + (j + k * 2) * 3 << std::endl;
								if (bit_mask[i + (j + k * 2) * 3]) {
									//std::cout << i << " " << j << " " <<k<< std::endl;
									//std::cout << "bit " << i + (j + k * 2) * 3 << " is true " << std::endl;
									skeleton.set_voxel(i, j, k);
								}
							}
						}
					}

					/**used to check the reciprocity of neighborhood->mask and mask->neighborhood*/
					/*
					GRuint computed_mask_value(skeleton.extract_neighborhood_mask_value_on_axis(1, 0, 1, 1));
					if (computed_mask_value != mask_value) {
						std::bitset<18> computed_bit_mask(computed_mask_value);
						std::cerr << "error  ! Expected " << mask_value << " and got " << computed_mask_value << std::endl;
						std::cerr << " bitwise comparison : " << std::endl;
						for (GRuint i(0); i < 18; i++) {
							std::cerr << bit_mask[i];
						}
						std::cerr << std::endl;
						for (GRuint i(0); i < 18; i++) {
							std::cerr << computed_bit_mask[i];
						}
						std::cerr << std::endl;
						std::cerr << std::endl;
					}
					else {
						//	std::cerr << " looking good for value " << mask_value << std::endl;
					}
					*/


					//and check if mask is indeed a critical 2-clique :
					if (skeleton.is_critical_2_clique(1, 0, 1, 1)) {

						//if yes, we set the corresponding bit to 1
						critical_2_cliques_indices[mask_value] = true;

						critical_masks_count++;
						/*
						std::cout << " __________________________" << std::endl;
						std::cout << " found 2-clique mask : " << std::endl;
						std::cout << "mask value : " << mask_value << std::endl;

						std::cout << "bit mask : " << std::endl;
						for (GRuint i(0); i < 18; i++) {
						std::cout << bit_mask[i];
						}
						std::cout << std::endl;

						std::cout << "voxel list : " << std::endl;
						for (GRuint i(0); i < skeleton.true_voxels_.size(); i++) {
						GRuint x, y, z;
						skeleton.voxel_id_to_coordinates(skeleton.true_voxels_[i], x, y, z);
						std::cout << " " << x << ", " << y << ", " << z << std::endl; ;
						}

						std::cout << std::endl;
						*/
					}
				}
			}

			std::cout << " among " << K2Y_CONFIGURATIONS << " possible configurations, " << critical_masks_count << " were critical 2-cliques : " << std::endl;
			
			/*std::cout << "masks  : " << std::endl;
			for (GRuint i(0); i < masks.size(); i++) {
				std::cout << masks[i] << " ";
				critical_2_cliques_indices[masks[i]] = true;
			}
			std::cout << std::endl;*/
		}


		/**SKELETON GENERATION METHODS (random, vessel-like, Bertrand, etc.)*/

		/** Generates the voxel set described in [Bertrand 2016]. 
		Used to compare the result of the various computations (e.g. critical clique detection)
		with those given in the aforementioned paper*/
		static VoxelSkeleton* BertrandStructure() {
			GRuint w(8), h(8), s(8);

			VoxelSkeleton* skeleton = new VoxelSkeleton(w, h, s);

			skeleton->set_voxel(1, 0, 0);
			skeleton->set_voxel(1, 1, 0);
			skeleton->set_voxel(4, 1, 0);
			skeleton->set_voxel(2, 1, 1);
			skeleton->set_voxel(3, 1, 1);
			skeleton->set_voxel(2, 2, 1);
			skeleton->set_voxel(3, 2, 1);
			skeleton->set_voxel(0, 1, 2);
			skeleton->set_voxel(0, 2, 2);
			skeleton->set_voxel(1, 2, 2);
			skeleton->set_voxel(3, 2, 2);
			skeleton->set_voxel(4, 3, 2);


			return skeleton;
		}


		/**Generates a random connected voxel set*/
		void generate_random(GRuint nb_voxels, GRuint seed = 1234) {
			srand(seed);

			//erasing voxel grid
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			GRint current_voxel_id = rand() % nb_voxels_;

			GRint x, y, z;

			for (GRuint i(0); i < nb_voxels; i++) {

				voxel_id_to_coordinates(current_voxel_id, x, y, z);

				set_voxel(current_voxel_id);


				GRint next_voxel_id = -1;
				GRuint random_voxel_index = rand() % true_voxels_.size();
				GRuint random_voxel_id = true_voxels_[random_voxel_index];
				/*std::cout << "random index : " << random_voxel_index << std::endl;
				std::cout << "true voxels size : " << true_voxels_.size() << std::endl;
				std::cout << "random voxels id : " << random_voxel_id << std::endl;*/

				while (next_voxel_id < 0 || next_voxel_id >= (GRint)nb_voxels_ || voxels_[next_voxel_id].value_) {
					random_voxel_index = rand() % true_voxels_.size();
					random_voxel_id = true_voxels_[random_voxel_index];

					GRuint face_index = rand() % 6;

					switch (face_index) {
					case 0: next_voxel_id = random_voxel_id + 1; break;//right
					case 1: next_voxel_id = random_voxel_id - 1; break;//left
					case 2: next_voxel_id = random_voxel_id + width_; break;//back
					case 3: next_voxel_id = random_voxel_id - width_; break;//front
					case 4: next_voxel_id = random_voxel_id + height_*width_; break;//top
					case 5: next_voxel_id = random_voxel_id - height_*width_; break;//bottom
					}
				}

				current_voxel_id = next_voxel_id;
				
			}

		}

		/** Generates a random skeleton-like voxel set.
		It first creates a random skeleton and then randomly add voxels to the skeleton*/
		void generate_random_skeleton_like(GRuint nb_voxels, GRuint seed = 1234) {

			srand(seed);
			//erasing voxel grid
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			GRuint nb_skeleton_voxels(nb_voxels / 10);
			//phase 1 : generate skeleton
			
			GRuint root_voxel_id = voxel_coordinates_to_id(width_/2, height_/2, slice_/2);
			set_voxel(root_voxel_id);

			set_voxel(root_voxel_id + 1);
			set_voxel(root_voxel_id + 2);
			set_voxel(root_voxel_id + 3);
			set_voxel(root_voxel_id + 4);
			set_voxel(root_voxel_id + 5);

			GRuint current_voxel_id = root_voxel_id;

			GRuint branch_length(nb_skeleton_voxels / 5);

			GRuint previous_face_index(4);

			for (GRuint i(0); i < nb_skeleton_voxels; i++) {
				GRint next_voxel_id = -1;

				GRuint tried_face_count(0);

				while (next_voxel_id < 0 || next_voxel_id >= (GRint)nb_voxels_ || voxels_[next_voxel_id].value_) {

					GRuint face_index = rand() % 6;
					if (rand() % 100 < 80) {
						face_index = previous_face_index;
					}

					switch (face_index) {
					case 0: next_voxel_id = current_voxel_id + 1; break;//right
					case 1: next_voxel_id = current_voxel_id - 1; break;//left
					case 2: next_voxel_id = current_voxel_id + width_; break;//back
					case 3: next_voxel_id = current_voxel_id - width_; break;//front
					case 4: next_voxel_id = current_voxel_id + height_*width_; break;//top
					case 5: next_voxel_id = current_voxel_id - height_*width_; break;//bottom
					}

					previous_face_index = face_index;

					if (tried_face_count++ >= 6) {
						current_voxel_id = true_voxels_[rand() % true_voxels_.size()];
					}
				}

				current_voxel_id = next_voxel_id;
				set_voxel(current_voxel_id);

				if ((i % branch_length) == 0) {
					current_voxel_id = true_voxels_[rand() % true_voxels_.size()];
					//std::cout << "branching at iteration " << i << std::endl;
					//std::cout << "voxel id : " << current_voxel_id << std::endl;
				}
			}


			//phase 2: thicken skeleton
			for (GRuint i(true_voxels_.size()); i < nb_voxels; i++) {
				//std::cout << "true voxel count : " << true_voxels_.size() << std::endl;

				GRint next_voxel_id = -1;
				GRuint random_voxel_index = rand() % true_voxels_.size();
				GRuint random_voxel_id = true_voxels_[random_voxel_index];
				/*std::cout << "random index : " << random_voxel_index << std::endl;
				std::cout << "true voxels size : " << true_voxels_.size() << std::endl;
				std::cout << "random voxels id : " << random_voxel_id << std::endl;*/

				GRuint tried_face_count(0);

				while (next_voxel_id < 0 || next_voxel_id >= (GRint)nb_voxels_ || voxels_[next_voxel_id].value_) {
					random_voxel_index = rand() % true_voxels_.size();
					random_voxel_id = true_voxels_[random_voxel_index];

					GRuint face_index = rand() % 6;

					switch (face_index) {
					case 0: next_voxel_id = random_voxel_id + 1; break;//right
					case 1: next_voxel_id = random_voxel_id - 1; break;//left
					case 2: next_voxel_id = random_voxel_id + width_; break;//back
					case 3: next_voxel_id = random_voxel_id - width_; break;//front
					case 4: next_voxel_id = random_voxel_id + height_*width_; break;//top
					case 5: next_voxel_id = random_voxel_id - height_*width_; break;//bottom
					}

					if (tried_face_count++ >= 6) {
						random_voxel_id = true_voxels_[rand() % true_voxels_.size()];
					}
				}

				//std::cout << "set voxel : " << random_voxel_id << std::endl;
				set_voxel(next_voxel_id);

			}

		}

	};

}
