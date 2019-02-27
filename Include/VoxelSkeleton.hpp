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
#include "SkeletalGraph.hpp"
#include "common.hpp"

namespace grapholon {

#define NON_EXISTENT_ID (0xffff)
#define NON_EXISTENT_COORDINATE (0xffff)

#define K2Y_CONFIGURATIONS (262144) //2^18
#define K2_MASK_WIDTH 3
#define K2_MASK_HEIGHT 2
#define K2_MASK_SLICE 3


#define THINNING_ITERATION_LIMIT 10000 ///< Hard limit to avoid infinite loop in the thinning algo

#define MIN_SMOOTHING_THRESHOLD 0.1f
#define MAX_SMOOTHING_THRESHOLD 1.f
#define MIN_SMOOTHING_DISTANCE 0
#define MAX_SMOOTHING_DISTANCE 2



#define IF_DEBUG_DO(command) {if(debug_log)command; }

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

	enum CriticalClique{ NON_CRITICAL, CLIQUE3, CLIQUE2, CLIQUE1, CLIQUE0};
	enum TopologicalClass{UNCLASSIFIED, INTERIOR_POINT, ISOLATED_POINT, BORDER_POINT, CURVES_POINT, CURVE_JUNCTION, SURFACE_CURVES_JUNCTION, SURFACE_JUNCTION, SURFACES_CURVE_JUNCTION};
	
	struct SkeletonVoxel {
		bool value_ = false;
		bool selected_ = false;
		TopologicalClass topological_class_ = UNCLASSIFIED;

		SkeletonVoxel(bool value, bool selected, TopologicalClass top_class) 
			: value_(value), selected_(selected), topological_class_(top_class) {}
	};

	typedef std::vector<GRuint> IndexVector;

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

		IndexVector true_voxels_;

		IndexVector anchor_voxels_;///< Voxels that cannot be removed during thinning


	public:



		/***********************************************************************************************/
		/*********************************************************************************** TYPEDEFS **/
		/***********************************************************************************************/

		typedef bool(VoxelSkeleton::*AdjencyFunction)(GRuint, GRuint);
		typedef GRuint(VoxelSkeleton::*SelectionFunction)(const std::vector<GRuint>&);
		typedef bool(VoxelSkeleton::*SkelFunction)(GRuint);



		/***********************************************************************************************/
		/******************************************************************************* CONSTRUCTORS **/
		/***********************************************************************************************/


		VoxelSkeleton(GRuint width, GRuint height, GRuint slice) : width_(width + 2), height_(height + 2), slice_(slice + 2), nb_voxels_(width_*height_*slice_) {
			voxels_ = (SkeletonVoxel*)calloc(nb_voxels_, sizeof(SkeletonVoxel));
			memset(voxels_, 0, nb_voxels_ *sizeof(SkeletonVoxel));
		}

		~VoxelSkeleton(){
			free(voxels_);
		}



		/***********************************************************************************************/
		/********************************************************************************** ACCESSORS **/
		/***********************************************************************************************/

		GRuint width() const {
			return width_;
		}

		GRuint height() const {
			return height_;
		}

		GRuint slice() const {
			return slice_;
		}

		GRuint voxel_count() const {
			return nb_voxels_;
		}


		bool same_dimensions_as(VoxelSkeleton* other_voxel_set)const {
			return width() == other_voxel_set->width() 
				&& height() == other_voxel_set->height() 
				&& slice() == other_voxel_set->slice();
		}

		GRuint set_voxel_count() const {
			return (GRuint)true_voxels_.size();
		}

		SkeletonVoxel voxel(GRuint id) const {
			if (id >= (GRint)nb_voxels_) {
				return SkeletonVoxel(false, false, UNCLASSIFIED);
			}
			else {
				return voxels_[id];
			}
		}

		SkeletonVoxel voxel(GRint x, GRint y, GRint z) const {
			return voxel(voxel_coordinates_to_id(x, y, z));
		}


		const std::vector<GRuint>& true_voxels()const {
			return true_voxels_;
		}


		bool set_voxel(GRuint id, bool value = true) {
			if (
				id >= nb_voxels_) {
				return false;
			}

			if (voxels_[id].value_ == value) {
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

		/** set the whole memory to zero and empty the list of true voxels*/
		bool set_voxel(GRuint x, GRuint y, GRuint z, bool value = true) {
			return set_voxel(voxel_coordinates_to_id(x, y, z), value);
		}



		bool set_anchor_voxel(GRuint id, bool value = true) {
			set_voxel(id, value);

			if (value) {
				anchor_voxels_.push_back(id);
			}
			else {
				anchor_voxels_.erase(std::remove(anchor_voxels_.begin(), anchor_voxels_.end(), id), anchor_voxels_.end());
			}

			return true;
		}

		bool set_anchor_voxel(GRuint x, GRuint y, GRuint z, bool value = true) {
			return set_anchor_voxel(voxel_coordinates_to_id(x,y,z), value);
		}


		void remove_all_voxels() {
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));
			true_voxels_ = std::vector<GRuint>();
			anchor_voxels_ = std::vector<GRuint>();
		}

		/***********************************************************************************************/
		/***************************************************************** COORDINATES-ID CONVERSIONS **/
		/***********************************************************************************************/

		//id 0 is actually voxel (-1, -1, -1)
		GRuint voxel_coordinates_to_id(GRuint x, GRuint y, GRuint z) const {
			return x+1 + (y+1 + (z+1) * height_) * width_;
		}

		
		void voxel_id_to_coordinates(GRuint id, GRuint& x, GRuint& y, GRuint& z) const {
				z = id / (width_ * height_) - 1;
				GRuint rem = id % (width_ * height_);
				y = rem / width_ - 1;
				x = rem % width_ - 1;
		}

		/***********************************************************************************************/
		/******************************************************************** NEIGHBORHOOD EXTRACTION **/
		/***********************************************************************************************/
		
		void extract_0_neighborhood_star(GRuint voxel_id, std::vector<GRuint>& neighborhood, bool bar = false) {
			GRuint x, y, z;
			voxel_id_to_coordinates(voxel_id, x, y, z);
			extract_0_neighborhood_star(x, y, z, neighborhood, bar);
		}

		void extract_1_neighborhood_star(GRuint voxel_id, std::vector<GRuint>& neighborhood, bool bar = false) {
			GRuint x, y, z;
			voxel_id_to_coordinates(voxel_id, x, y, z);
			extract_1_neighborhood_star(x, y, z, neighborhood, bar);
		}

		void extract_2_neighborhood_star(GRuint voxel_id, std::vector<GRuint>& neighborhood, bool bar = false) {
			GRuint x, y, z;
			voxel_id_to_coordinates(voxel_id, x, y, z);
			extract_2_neighborhood_star(x, y, z, neighborhood, bar);
		}

		void extract_0_neighborhood_star(GRuint x, GRuint y, GRuint z, std::vector<GRuint>& neighborhood, bool bar = false) {
			bool debug_log = false;
			
			GRuint neighbor_id;
			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 3; j++) {
					for (GRuint k(0); k < 3; k++) {
						if (i != 1 || j != 1 || k != 1) {
							neighbor_id = voxel_coordinates_to_id(x - 1 + i, y - 1 + j, z - 1 + k);
							IF_DEBUG_DO(std::cout << "voxel : " << x - 1 + i << " " << y - 1 + j << " " << z - 1 + k;)
								if (voxel(neighbor_id).value_ != bar) {
									neighborhood.push_back(neighbor_id);
									IF_DEBUG_DO(std::cout << " is in the neighborhood" << std::endl;)
								}
								else {
									IF_DEBUG_DO(std::cout << " is not in the neighborhood" << std::endl;)
								}
						}
					}
				}
			}
		}

		void extract_1_neighborhood_star(GRuint x, GRuint y, GRuint z, std::vector<GRuint>& neighborhood, bool bar = false) {
			GRuint voxel_id = voxel_coordinates_to_id(x, y, z);
			GRuint neighbor_id;
			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 3; j++) {
					for (GRuint k(0); k < 3; k++) {
						if (i != 1 || j != 1 || k != 1) {
							neighbor_id = voxel_coordinates_to_id(x - 1 + i, y - 1 + j, z - 1 + k);
							if (are_1adjacent(voxel_id, neighbor_id)){
								if (voxel(neighbor_id).value_ != bar) {
									neighborhood.push_back(neighbor_id);
								}
							}
						}
					}
				}
			}
			return;
		}
		
		void extract_2_neighborhood_star(GRuint x, GRuint y, GRuint z, std::vector<GRuint>& neighborhood, bool bar = false) {
			bool debug_log = false;
			
			GRuint neighbor_id;
			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 2; j++) {
					GRuint coords[3] = { x,y,z };
					coords[i] += (1 - 2 * j);
					neighbor_id = voxel_coordinates_to_id(coords[0], coords[1], coords[2]);
					IF_DEBUG_DO(std::cout << "checking neighbor " << neighbor_id << " :  " << coords[0] << " " << coords[1] << " " << coords[2] << std::endl;)
						if (neighbor_id < nb_voxels_ && !voxel(neighbor_id).value_) {
							neighborhood.push_back(neighbor_id);
							IF_DEBUG_DO(std::cout << "added to neighborhood" << std::endl;)
						}
				}
			}
		}
		
		/***********************************************************************************************/
		/************************************************************************* VOXEL COMPUTATIONS **/
		/***********************************************************************************************/

		/** Return the euclidian distance between two voxels*/
		GRfloat voxel_distance(GRuint voxel_id, GRuint other_voxel_id) const{
			GRuint x1, y1, z1;
			GRuint x2, y2, z2;

			voxel_id_to_coordinates(voxel_id, x1, y1, z1);
			voxel_id_to_coordinates(other_voxel_id, x2, y2, z2);

			return Vector3f(x1, y1, z1).distance(Vector3f(x2, y2, z2));
		}


		GRfloat min_voxel_radius(GRuint voxel_id) const {

			std::vector<bool> checked_voxel(voxel_count());
			checked_voxel[voxel_id] = true;

			IndexVector voxels_to_check;

			GRuint x, y, z;
			voxel_id_to_coordinates(voxel_id, x, y, z);

			GRuint iteration_count(0);
			GRuint distance_to_boundary(MIN(MIN(MIN(width_ - x, x), MIN(height_ - y, y)), MIN(slice_ - z, z)));

			//initialization (2-neighborhood)
			voxels_to_check.push_back(voxel_coordinates_to_id(x - 1, y, z));
			voxels_to_check.push_back(voxel_coordinates_to_id(x + 1, y, z));
			voxels_to_check.push_back(voxel_coordinates_to_id(x, y - 1, z));
			voxels_to_check.push_back(voxel_coordinates_to_id(x, y + 1, z));
			voxels_to_check.push_back(voxel_coordinates_to_id(x, y, z - 1));
			voxels_to_check.push_back(voxel_coordinates_to_id(x, y, z + 1));

			bool found_unset_voxel(false);
			GRfloat min_distance(std::numeric_limits<GRfloat>::max());

			while (!found_unset_voxel && iteration_count <= distance_to_boundary) {
				//std::cout << "iteration count : " << iteration_count << std::endl;
				IndexVector next_voxels_to_check;
				for (auto other_voxel_id : voxels_to_check) {
					checked_voxel[other_voxel_id] = true;
				}

				for (auto other_voxel_id : voxels_to_check) {

					if (!voxel(other_voxel_id).value_) {
						GRfloat distance(voxel_distance(voxel_id, other_voxel_id));
				//		std::cout << "found unset voxel  at distance : " << distance << std::endl;

						min_distance = MIN(min_distance, distance);
						found_unset_voxel = true;
					}
					else {

						//add neighborhood to next voxels to check

						GRuint x2, y2, z2;
						voxel_id_to_coordinates(other_voxel_id, x2, y2, z2);
						IndexVector two_neighborhood;
						two_neighborhood.push_back(voxel_coordinates_to_id(x2-1, y2, z2));
						two_neighborhood.push_back(voxel_coordinates_to_id(x2+1, y2, z2));
						two_neighborhood.push_back(voxel_coordinates_to_id(x2, y2-1, z2));
						two_neighborhood.push_back(voxel_coordinates_to_id(x2, y2+1, z2));
						two_neighborhood.push_back(voxel_coordinates_to_id(x2, y2, z2-1));
						two_neighborhood.push_back(voxel_coordinates_to_id(x2, y2, z2+1));

						for (auto neighbor_id : two_neighborhood) {
							if (!checked_voxel[neighbor_id]) {
								next_voxels_to_check.push_back(neighbor_id);
							}
						}
					}
				}

				voxels_to_check = next_voxels_to_check;
				iteration_count++;
			}

			return min_distance;
		}


		/***********************************************************************************************/
		/*********************************************************************** VOXEL CLASSIFICATION **/
		/***********************************************************************************************/


		/********************************************************************************** ADJACENCY **/


		/**This checks if the two ids correspond to 0-adjacent voxels
		NOTE : if id == id2 it will return false so this is not really 0-adjency*/
		bool are_0adjacent(GRuint id, GRuint id2) {
			if (id >= nb_voxels_ || id2 >= nb_voxels_) {
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

		bool are_1adjacent(GRuint id, GRuint id2) {
			if (id >= nb_voxels_ || id2 >= nb_voxels_) {
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


		bool are_2adjacent(GRuint id, GRuint id2) {
			if (id >= nb_voxels_ || id2 >= nb_voxels_) {
				return false;
			}
			return id + 1 == id2
				|| id - 1 == id2
				|| id + width_ == id2
				|| id - width_ == id2
				|| id + width_ * height_ == id2
				|| id - width_ * height_ == id2;
		}

		bool are_0adjacent(GRuint x, GRuint y, GRuint z, GRuint x2, GRuint y2, GRuint z2) {
			return are_0adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}
		bool are_1adjacent(GRuint x, GRuint y, GRuint z, GRuint x2, GRuint y2, GRuint z2) {
			return are_1adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}
		bool are_2adjacent(GRuint x, GRuint y, GRuint z, GRuint x2, GRuint y2, GRuint z2) {
			return are_2adjacent(voxel_coordinates_to_id(x, y, z), voxel_coordinates_to_id(x2, y2, z2));
		}


		/****************************************************************************** CONNECTEDNESS **/


		/** The idea is that the first voxel in the list is "explored" by "visiting"
		all its neighbors. We then "explore" all the "visited" voxels until every visited voxel
		is also explored. If and only if all voxels have been visited then it's 0 connected.*/
		bool is_k_connected(const std::vector<GRuint>& voxel_ids, AdjencyFunction adjency_function, GRuint n = 0) {
			bool debug_log = false;
			
			IF_DEBUG_DO(std::cout << "checking connectdedness of first "<<n<<" voxels : " << std::endl;)
			for (GRuint i(0); i < voxel_ids.size(); i++) {
				GRuint x, y, z;
				voxel_id_to_coordinates(voxel_ids[i], x, y, z);
				IF_DEBUG_DO(std::cout << voxel_ids[i] <<" : "<< x << " " << y << " " << z << std::endl;)
			}
			IF_DEBUG_DO(std::cout << std::endl;)
			
			

			if (n == 0 || n >= voxel_ids.size()) {
				n = (GRuint)voxel_ids.size();
			}
			
			if (voxel_ids.size() == 1) {
				return true;
			}

			if (voxel_ids.size() == 2) {
				return (this->*adjency_function)(voxel_ids[0], voxel_ids[1]);
			}

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
						IF_DEBUG_DO(std::cout << "visited voxel : " << i << std::endl;)
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
						IF_DEBUG_DO(std::cout << "found unexplored : " << next_explored_index << std::endl;)
					}
					next_explored_index++;
				}

				IF_DEBUG_DO(std::cout << "last explored index is now : " << last_explored_index << std::endl;)

				//if we reached the end of the array, we check if all voxels have been visited
				if (!found_unexplored || last_explored_index == explored.size() - 1) {
					bool visited_voxels_are_also_explored(true);
					bool all_visited(true);
					for (GRuint i(0); i < n; i++) {
						if (visited[i]) {
							visited_voxels_are_also_explored &= explored[i];
						}
						all_visited &= visited[i];
						IF_DEBUG_DO(std::cout << "all visited : " << all_visited << std::endl;);
					}
					if (visited_voxels_are_also_explored) {
						IF_DEBUG_DO(std::cout << " visited are also explored " << std::endl;)
						uncertain = false;
						result = all_visited;
					}
				}
				
				/*std::cout << "visited : ";
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


		/**
		\param n : the first n voxels only must be connected. if n == 0, then the all must be connected.
		This allows to check if some voxels are connected through others while those others might not necessarily be connected
		NOTE : this is very greedy (O(n^2), n = voxels_ids.size()) but it is not used at runtime so it's fine */
		bool is_k_connected(std::vector<GRuint> voxel_ids, GRuint k, GRuint n = 0) {
			switch (k) {
			case 0: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_0adjacent, n);
				break;
			}
			case 1: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_1adjacent, n);
				break;
			}
			case 2: {
				return is_k_connected(voxel_ids, &VoxelSkeleton::are_2adjacent, n);
				break;
			}
			default: {
				std::cerr << k << "-connectedness does not make sense with voxels. Returning false" << std::endl;
				return false;
			}
			}
		}


		/******************************************************************************* REDUCIBILITY **/


		bool is_reducible(std::vector<GRuint> voxels_id) {
			std::cout << "checking if voxel set is reducible : " << std::endl;
			for (GRuint i(0); i < voxels_id.size(); i++) {
				std::cout << " - " << voxels_id[i] << " ";
			}
			std::cout << std::endl;

			if (voxels_id.size() == 0) {
				return false;
			}else if (voxels_id.size() == 1){
				return true;
			}
			else {
				bool x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible = false;

				for (GRuint i(0); i < voxels_id.size(); i++) {
					std::cout << "	checking voxel " << voxels_id[i] << std::endl;

					//first computing N0(x)
					std::vector<GRuint> neighbors;
					for (GRuint j(0); j < voxels_id.size(); j++) {
						if (are_0adjacent(voxels_id[i], voxels_id[j])) {
							neighbors.push_back(voxels_id[j]);
						}
					}

					//then X without x
					std::vector<GRuint> voxel_set_without_i = voxels_id;
					voxel_set_without_i.erase(
						std::remove(
							voxel_set_without_i.begin(), voxel_set_without_i.end(), voxels_id[i]),
						voxel_set_without_i.end()
					);
					
					bool neighborhood_is_reducible = is_reducible(neighbors);
					bool voxel_set_without_i_is_reducible = is_reducible(voxel_set_without_i);

				
					std::cout << "    neigh of voxel " << voxels_id[i] << " is "<<(neighborhood_is_reducible ? "" : " not ")<<" reducible " << std::endl;
					std::cout << "    set without voxel " << voxels_id[i] << " is " << (voxel_set_without_i_is_reducible ? "" : " not ") << " reducible " << std::endl;


					x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible
						|= (neighborhood_is_reducible && voxel_set_without_i_is_reducible);
					
				}
				std::cout << std::endl << std::endl;
				return x_exists_st_N0x_is_reducible_and_X_without_x_is_reducible;
			}
		}


		/********************************************************************************* SIMPLICITY **/


		bool is_simple(GRuint x, GRuint y, GRuint z) {

			bool debug_log = false;

			IF_DEBUG_DO(std::cout << "checking if voxel " << x << " " << y << " " << z << " is simple " << std::endl;)
			
			//first extract the zero neighborhood*
			std::vector<GRuint> zero_neighborhood_star;
			
			extract_0_neighborhood_star(x, y, z, zero_neighborhood_star);

			//then check if it's empty and zero-connected
			if (zero_neighborhood_star.size() == 0 || !is_k_connected(zero_neighborhood_star, 0u)) {
				return false;
			}
			IF_DEBUG_DO(std::cout << "zero neigh is non-empty and 0 connected " << std::endl;)

			//then compute the 2-neighborhood that is not set
			std::vector<GRuint> two_neighborhood_bar;
			extract_2_neighborhood_star(x, y, z, two_neighborhood_bar, true);

			GRuint two_neighborhood_bar_size((GRuint)two_neighborhood_bar.size());
			//and check that it is not empty
			if (two_neighborhood_bar_size == 0) {
				return false;
			}
			IF_DEBUG_DO(std::cout << "2-neighborhood bar is non-empty : " << two_neighborhood_bar_size << std::endl;)
			
			//then add the one-neighborhood
			extract_1_neighborhood_star(x, y, z, two_neighborhood_bar, true);

			//and finally check if the 2-neighborhood is 2-connected in the 1-neighborhood bar
			return is_k_connected(two_neighborhood_bar, 2, two_neighborhood_bar_size);
		}


		/************************************************************************** CLIQUE DETECTION **/




		/**K_2 mask matchings. 
		\param axis 0:X-axis, 1:Y-axis, 2:Z-axis */
		bool clique_matches_K2_mask(const GRuint x, const GRuint y, const GRuint z, const AXIS axis) {

			if (axis > Z_AXIS) {
				std::cerr << "wrong axis to apply K2 mask. Returning false" << std::endl;
				return false;
			}

			//compute voxel B's coordinates
			GRuint x2(x + (axis == X_AXIS));
			GRuint y2(y + (axis == Y_AXIS));
			GRuint z2(z + (axis == Z_AXIS));

			//first checking if the voxel and its neighbor in the axis' direction are set
			if (!voxel(x, y, z).value_ || !voxel(x2, y2, z2).value_) {
				return false;
			}


			//first create the list of voxels in {X0,...,X7, Y0,...,Y7}AND X (at most 16 voxels)
			std::vector<GRuint> mask_neighborhood_intersection;

			//std::cout << "checking clique (" << x << ", " << y << ", " << z<<") - ("<<x2<<", "<<y2<<", "<<z2<<") on axis "<<axis<< std::endl;

			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 3; j++) {
					if (!(i == 1 && j == 1)) {

						GRuint X_neighbor_id;
						GRuint Y_neighbor_id;

						switch (axis) {
						case X_AXIS: {
							X_neighbor_id = voxel_coordinates_to_id(x,  y  - 1 + i, z  - 1 + j);
							Y_neighbor_id = voxel_coordinates_to_id(x2, y2 - 1 + i, z2 - 1 + j);
							break;
						}
						case Y_AXIS: {
							X_neighbor_id = voxel_coordinates_to_id(x  - 1 + i, y,  z  - 1 + j);
							Y_neighbor_id = voxel_coordinates_to_id(x2 - 1 + i, y2, z2 - 1 + j);

							break;
						}
						case Z_AXIS: {
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
			//std::cout << "neighborhood is 0-connected" << std::endl;

			//finally check if for each i in {0,2,4,6}, Xi or Yi is in X
			/*bool i_th_subset_is_in_neighborhood[4] = { false };
			for (GRuint i(0); i < mask_neighborhood_intersection.size(); i++) {
			if()
			}*/

			//std::cout << "checking last condition" << std::endl;

			const GRuint a = axis;//to lighten the expressions


			bool is_in_subset =
				(voxel(x + (a != 0), y - (a == 0), z).value_ //X0
					|| voxel(x2 + (a != 0), y2 - (a == 0), z2).value_)//Y0
				&& (voxel(x, y - (a == 2), z + (a != 2)).value_ //X2
					|| voxel(x2, y2 - (a == 2), z2 + (a != 2)).value_)//Y2
				&& (voxel(x - (a != 0), y + (a == 0), z).value_ //X0
					|| voxel(x2 - (a != 0), y2 + (a == 0), z2).value_)//Y2
				&& (voxel(x, y + (a == 2), z - (a != 2)).value_ //X2
					|| voxel(x2, y2 + (a == 2), z2 - (a != 2)).value_);//Y2

			//std::cout << " is critical 2-clique : " << is_in_subset << std::endl << std::endl << std::endl;

			return is_in_subset;
		}


		/** here the axis is the direction of the "normal" of the ABCD plane. 
		So the K1 mask is on the X axis in the reference literature
		
		IMPORTANT NOTE : this method assumes that all coordinates correspond tovoxels correctly located. 
		e.g. if axis = X_AXIS, and A = (0,0,0), then 
		B = (0,1,0), C = (0,0,1) and D = (0,1,1)

		i.e. if there is not verification that those are all neighbors or that the axis is right. 
		This is done in order to speed-up the process*/
		bool clique_matches_K1_mask(GRuint x, GRuint y, GRuint z,
			GRuint x_B, GRuint y_B, GRuint z_B,
			GRuint x_C, GRuint y_C, GRuint z_C,
			GRuint x_D, GRuint y_D, GRuint z_D,
			AXIS axis) {
			
			
			GRint axis_vector[3] = { axis == X_AXIS, axis == Y_AXIS, axis == Z_AXIS };

			/*std::cout << "A : " <<x<<" "<<y<<" "<<z<<" :: "<< voxel(x, y, z).value_ << std::endl;
			std::cout << "B : " << x_B << " " << y_B << " " << z_B << " :: " << voxel(x_B, y_B, z_B).value_ << std::endl;
			std::cout << "C : " << x_C << " " << y_C << " " << z_C << " :: " << voxel(x_C, y_C, z_C).value_ << std::endl;
			std::cout << "D : " << x_D << " " << y_D << " " << z_D << " :: " << voxel(x_D, y_D, z_D).value_ << std::endl;
			*/
			if (voxel(x, y, z).value_ && voxel(x_D, y_D, z_D).value_
				|| voxel(x_B, y_B, z_B).value_ && voxel(x_C, y_C, z_C).value_) {

				//first check whether the set {X0, X1, X2, X3} is empty or not
				bool X_set_non_empty
					= voxel(x - axis_vector[0], y - axis_vector[1], z - axis_vector[2]).value_
					|| voxel(x_B - axis_vector[0], y_B - axis_vector[1], z_B - axis_vector[2]).value_
					|| voxel(x_C - axis_vector[0], y_C - axis_vector[1], z_C - axis_vector[2]).value_
					|| voxel(x_D - axis_vector[0], y_D - axis_vector[1], z_D - axis_vector[2]).value_;

				//then check whether the set {Y0, Y1, Y2, Y3} is empty or not
				bool Y_set_non_empty
					= voxel(x + axis_vector[0], y + axis_vector[1], z + axis_vector[2]).value_
					|| voxel(x_B + axis_vector[0], y_B + axis_vector[1], z_B + axis_vector[2]).value_
					|| voxel(x_C + axis_vector[0], y_C + axis_vector[1], z_C + axis_vector[2]).value_
					|| voxel(x_D + axis_vector[0], y_D + axis_vector[1], z_D + axis_vector[2]).value_;

				//std::cout << " X set non empty : " << X_set_non_empty << std::endl;
				//std::cout << " Y set non empty : " << Y_set_non_empty << std::endl;

				//and return whether both are the same or not
				return X_set_non_empty == Y_set_non_empty;
			}
			else {
				return false;
			}
		}

		/** a bit ugly but it's to have some uniformity among mask matching methods*/
		bool clique_matches_K0_mask(GRuint x_A, GRuint y_A, GRuint z_A,
			GRuint x_B, GRuint y_B, GRuint z_B,
			GRuint x_C, GRuint y_C, GRuint z_C,
			GRuint x_D, GRuint y_D, GRuint z_D,
			GRuint x_E, GRuint y_E, GRuint z_E,
			GRuint x_F, GRuint y_F, GRuint z_F,
			GRuint x_G, GRuint y_G, GRuint z_G,
			GRuint x_H, GRuint y_H, GRuint z_H) {
			return voxel(x_A, y_A, z_A).value_ && voxel(x_H, y_H, z_H).value_
				|| voxel(x_B, y_B, z_B).value_ && voxel(x_G, y_G, z_G).value_
				|| voxel(x_C, y_C, z_C).value_ && voxel(x_F, y_F, z_F).value_
				|| voxel(x_D, y_D, z_D).value_ && voxel(x_E, y_E, z_E).value_;
		}


		/*Eventually this will be replaced with a mask table lookup*/
		bool is_critical_3_clique(GRuint x, GRuint y, GRuint z) {
			return !is_simple(x, y, z);
		}


		/*Eventually this will be replaced with a mask table lookup*/
		bool is_critical_2_clique(GRuint x, GRuint y, GRuint z, AXIS axis) {
			return clique_matches_K2_mask(x, y, z, axis);
		}


		bool is_critical_1_clique(GRuint x, GRuint y, GRuint z,
			GRuint x_B, GRuint y_B, GRuint z_B,
			GRuint x_C, GRuint y_C, GRuint z_C,
			GRuint x_D, GRuint y_D, GRuint z_D,
			AXIS axis) {
			return clique_matches_K1_mask(x, y, z, x_B, y_B, z_B, x_C, y_C, z_C, x_D, y_D, z_D, axis);
		}

		bool is_critical_0_clique(GRuint x_A, GRuint y_A, GRuint z_A,
			GRuint x_B, GRuint y_B, GRuint z_B,
			GRuint x_C, GRuint y_C, GRuint z_C,
			GRuint x_D, GRuint y_D, GRuint z_D,
			GRuint x_E, GRuint y_E, GRuint z_E,
			GRuint x_F, GRuint y_F, GRuint z_F,
			GRuint x_G, GRuint y_G, GRuint z_G,
			GRuint x_H, GRuint y_H, GRuint z_H) {
			return clique_matches_K0_mask(x_A, y_A, z_A, x_B, y_B, z_B, x_C, y_C, z_C, x_D, y_D, z_D,
				x_E, y_E, z_E, x_F, y_F, z_F, x_G, y_G, z_G, x_H, y_H, z_H);
		}


		//only does interior and border points for now
		void compute_voxel_attributes() {

			for (GRuint i(0); i < true_voxels_.size(); i++) {
				GRuint voxel_id = true_voxels_[i];

				if (voxels_[voxel_id].value_) {
					GRuint x, y, z;
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

		



		void extract_all_cliques(std::vector<std::vector<std::vector<GRuint>>>& critical_cliques) {

			//set the clique set of size 4 (one for each k-cliques sets)
			critical_cliques = std::vector<std::vector<std::vector<GRuint>>>(4);

			bool debug_log = false;
			IF_DEBUG_DO(std::cout << std::endl << "extracting all cliques from skeleton" << std::endl;)
				IF_DEBUG_DO(std::cout << "true voxels count : " << true_voxels_.size() << std::endl;)

			for (GRuint i(0); i < this->true_voxels().size(); i++) {


				GRuint voxel_id(this->true_voxels()[i]);
				GRuint x, y, z;
				this->voxel_id_to_coordinates(voxel_id, x, y, z);

				IF_DEBUG_DO(std::cout << "classifying voxel : ( " << x << ", " << y << ", " << z << " )" << std::endl;)

				//first detect 3-cliques
				if (this->is_critical_3_clique(x, y, z)) {
					critical_cliques[3].push_back({ voxel_id });
					//IF_DEBUG_DO(std::cout << "	it is a 3-clique " << std::endl;)
				}
				else {
					//IF_DEBUG_DO(std::cout << "	it is not a 3-clique" << std::endl;)
				}


				//then detect 2-cliques
				for (GRuint axis(X_AXIS); axis <= Z_AXIS; axis++) {
					//IF_DEBUG_DO(std::cout << "checkin 1-clique on axis " << axis << std::endl;)
					if (this->is_critical_2_clique(x, y, z, (AXIS)axis)) {
						GRuint voxel_B_id(this->voxel_coordinates_to_id(x + (axis == X_AXIS), y + (axis == Y_AXIS), z + (axis == Z_AXIS)));
						critical_cliques[2].push_back({ voxel_id, voxel_B_id });
					//	IF_DEBUG_DO(std::cout << "		there is a 2-clique on axis " << axis << std::endl;)
					}
				}

				//std::cout << "	checked 2-cliques" << std::endl << std::endl;


				//then detect 1-cliques
				for (GRuint j(0); j <= 1; j++) {
					for (GRuint axis(X_AXIS); axis <= Z_AXIS; axis++) {

						IF_DEBUG_DO(std::cout << "		checking 1-clique on axis " << axis << std::endl;)
						GRuint x_A(x - j*(axis == Z_AXIS));
						GRuint y_A(y);
						GRuint z_A(z - j*(axis != Z_AXIS));

						GRuint x_B(x_A + (axis == Y_AXIS));
						GRuint y_B(y_A + (axis != Y_AXIS));
						GRuint z_B(z_A);

						GRuint x_C(x_A + (axis == Z_AXIS));
						GRuint y_C(y_A);
						GRuint z_C(z_A + (axis != Z_AXIS));

						GRuint x_D(x_B + x_C - x_A);
						GRuint y_D(y_B + y_C - y_A);
						GRuint z_D(z_B + z_C - z_A);
						
						/*std::cout << "		voxels are " << std::endl;
						std::cout << "		( " << x_A << " " << y_A << " " << z_A << " )" << std::endl;
						std::cout << "		( " << x_B << " " << y_B << " " << z_B << " )" << std::endl;
						std::cout << "		( " << x_C << " " << y_C << " " << z_C << " )" << std::endl;
						std::cout << "		( " << x_D << " " << y_D << " " << z_D << " )" << std::endl;
						*/

						if (this->is_critical_1_clique(
							x_A, y_A, z_A,
							x_B, y_B, z_B,
							x_C, y_C, z_C,
							x_D, y_D, z_D,
							(AXIS)axis)) {

							//std::cout << "		there is a 1-clique on axis " << axis << std::endl;

							GRuint voxel_A_id(this->voxel_coordinates_to_id(x_A, y_A, z_A));
							GRuint voxel_B_id(this->voxel_coordinates_to_id(x_B, y_B, z_B));
							GRuint voxel_C_id(this->voxel_coordinates_to_id(x_C, y_C, z_C));
							GRuint voxel_D_id(this->voxel_coordinates_to_id(x_D, y_D, z_D));
							critical_cliques[1].push_back(std::vector<GRuint>());

							if (this->voxel(voxel_A_id).value_) {
								critical_cliques[1].back().push_back(voxel_A_id);
							}
							if (this->voxel(voxel_B_id).value_) {
								critical_cliques[1].back().push_back(voxel_B_id);
							}
							if (this->voxel(voxel_C_id).value_) {
								critical_cliques[1].back().push_back(voxel_C_id);
							}
							if (this->voxel(voxel_D_id).value_) {
								critical_cliques[1].back().push_back(voxel_D_id);
							}
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

						if (this->is_critical_0_clique(
							x_A, y_A, z_A,
							x_B, y_B, z_B,
							x_C, y_C, z_C,
							x_D, y_D, z_D,
							x_E, y_E, z_E,
							x_F, y_F, z_F,
							x_G, y_G, z_G,
							x_H, y_H, z_H)) {

							GRuint voxel_A_id(this->voxel_coordinates_to_id(x_A, y_A, z_A));
							GRuint voxel_B_id(this->voxel_coordinates_to_id(x_B, y_B, z_B));
							GRuint voxel_C_id(this->voxel_coordinates_to_id(x_C, y_C, z_C));
							GRuint voxel_D_id(this->voxel_coordinates_to_id(x_D, y_D, z_D));
							GRuint voxel_E_id(this->voxel_coordinates_to_id(x_E, y_E, z_E));
							GRuint voxel_F_id(this->voxel_coordinates_to_id(x_F, y_F, z_F));
							GRuint voxel_G_id(this->voxel_coordinates_to_id(x_G, y_G, z_G));
							GRuint voxel_H_id(this->voxel_coordinates_to_id(x_H, y_H, z_H));
							critical_cliques[0].push_back(std::vector<GRuint>());

							if (this->voxel(voxel_A_id).value_) {
								critical_cliques[0].back().push_back(voxel_A_id);
							}
							if (this->voxel(voxel_B_id).value_) {
								critical_cliques[0].back().push_back(voxel_B_id);
							}
							if (this->voxel(voxel_C_id).value_) {
								critical_cliques[0].back().push_back(voxel_C_id);
							}
							if (this->voxel(voxel_D_id).value_) {
								critical_cliques[0].back().push_back(voxel_D_id);
							}
							if (this->voxel(voxel_E_id).value_) {
								critical_cliques[0].back().push_back(voxel_E_id);
							}
							if (this->voxel(voxel_F_id).value_) {
								critical_cliques[0].back().push_back(voxel_F_id);
							}
							if (this->voxel(voxel_G_id).value_) {
								critical_cliques[0].back().push_back(voxel_G_id);
							}
							if (this->voxel(voxel_H_id).value_) {
								critical_cliques[0].back().push_back(voxel_H_id);
							}
						}

					}
				}

			}
		}


		/*********************************************************************** MASKS PRECOMPUTATION **/


		/** Precomputes the critical 2-cliques masks
		Basically, each mask is an integer which represents one of the 266'144 possible configuration
		for the neighborhood of a 2-clique. Each possible configuration is tested and then stored
		as a single bit in a 2^18-bits bitset. This allows to convert any neighborhood into
		and integer 'mask' and then check if bitset[mask] is true in O(1).
		NOTE : around 35k configurations are critical 2-cliques among the 2^18 possible configs*/
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


					//and check if mask is indeed a critical 2-clique :
					if (skeleton.is_critical_2_clique(1, 0, 1, Y_AXIS)) {

						//if yes, we set the corresponding bit to 1
						critical_2_cliques_indices[mask_value] = true;

						critical_masks_count++;
					}
				}
			}

			std::cout << " among " << K2Y_CONFIGURATIONS << " possible configurations, " << critical_masks_count << " were critical 2-cliques : " << std::endl;
		}


		/************************************************************************** ISTHMUS DETECTION **/
		/** NOTE : check if simply checking connectedness of zero-neighborhoo also works
		I'll use that for now since it's much faster*/
		bool is_1_isthmus(GRuint x, GRuint y, GRuint z) {
			
			/*std::vector<GRuint> neighborhood2;
			extract_0_neighborhood_star(x, y, z, neighborhood2);
			return !is_k_connected(neighborhood2, 0u);*/

			//first create a
			VoxelSkeleton neighborhood(3, 3, 3);
			GRuint neighbor_id;
			for (GRuint i(0); i < 3; i++) {
				for (GRuint j(0); j < 3; j++) {
					for (GRuint k(0); k < 3; k++) {
						neighbor_id = voxel_coordinates_to_id(x - 1 + i, y - 1 + j, z - 1 + k);
						if (voxel(neighbor_id).value_) {
							neighborhood.set_voxel(i, j, k);
						}
					}
				}
			}
			neighborhood.set_voxel(1, 1, 1, false);

			//then thin it
			neighborhood.AsymmetricThinning(&VoxelSkeleton::SimpleSelection, &VoxelSkeleton::AlwaysFalseSkel);
			
			//and return whether the thinning has two component or not
			return neighborhood.true_voxels().size() == 2;
		}


		bool is_1_isthmus(GRuint voxel_id) {
			GRuint x, y, z;
			voxel_id_to_coordinates(voxel_id, x, y, z);
			return is_1_isthmus(x, y, z);
		}



		/***********************************************************************************************/
		/*********************************************************************************** THINNING **/
		/***********************************************************************************************/

		//typedef GRuint(VoxelSkeleton::*SelectionFunction)(std::vector<GRuint>);
		//typedef bool(VoxelSkeleton::*SkelFunction)(GRuint);

		/************************************************************************ SELECTION FUNCTIONS **/

		/** Returns the index of the first selected voxel encountered or the first voxel
		if no voxel is selected yet. */
		GRuint SimpleSelection(const std::vector<GRuint>& voxel_ids) {
			GRuint i(0);
			while (i < voxel_ids.size() && !voxel(voxel_ids[i]).selected_) {
				i++;
			}
			if (i >= voxel_ids.size()) {
				return voxel_ids[0];
			}
			else {
				return voxel_ids[i];
			}
		}


		/***************************************************************************** SKEL FUNCTIONS **/


		bool AlwaysFalseSkel(GRuint voxel_id) {
			return false;
		}

		bool ManualTipSkel(GRuint voxel_id) {
			return voxel_id == voxel_coordinates_to_id(1,0,0) || voxel_id == voxel_coordinates_to_id(4, 3, 2);
		}

		bool AnchoredSkel(GRuint voxel_id) {
			return std::find(anchor_voxels_.begin(), anchor_voxels_.end(), voxel_id) != anchor_voxels_.end();
		}

		bool OneIsthmusSkel(GRuint voxel_id) {
			return is_1_isthmus(voxel_id);
		}

		/***************************************************************************** THINNING ALGOS **/


		void AsymmetricThinning(SelectionFunction Select, SkelFunction Skel) {
		

			/** description of the voxel sets :
			 - K : the set of voxels that must be kept no matter what (defined by the Skel function)
			 - Y : the set of voxels to keep. At the end of each iteration it will become the new true_voxels set
			 - Z : the set of voxels to keep from a particular level of cliques*/
			std::vector<GRuint> voxel_set_K;
			bool stability(false);
			GRuint iteration_count(0);
			GRuint voxel_count_at_iteration_start((GRuint)true_voxels_.size());
			GRuint x, y, z;

			bool debug_log(false);

			//initialize K (optional)
			/*for (GRuint i(0); i < true_voxels_.size(); i++) {
				if ((this->*Skel)(true_voxels_[i])) {
					voxel_set_K.push_back(true_voxels_[i]);
					voxels_[true_voxels_[i]].selected_ = true;
				}
			}*/
			//std::cout << "initialized K : " << voxel_set_K.size() << std::endl;

			while (!stability && iteration_count < THINNING_ITERATION_LIMIT) {
				iteration_count++;
				voxel_count_at_iteration_start = (GRuint)true_voxels_.size();

				IF_DEBUG_DO(std::cout << "	running iteration " << iteration_count << std::endl;)
				//critical cliques holder
				std::vector<std::vector<std::vector<GRuint>>> critical_cliques;

				//the set of voxels to keep starts with the voxels in K
				std::vector<GRuint> voxel_set_Y = voxel_set_K;

				extract_all_cliques(critical_cliques);
				

				for (GRint d(3); d >= 0; d--) {
					IF_DEBUG_DO(std::cout << "		checking " << d << "-cliques" << std::endl;)
					GRuint voxels_in_d_cliques_count(0);

					std::vector<GRuint> voxel_set_Z;
					for (GRuint i(0); i < critical_cliques[d].size(); i++) {
						voxels_in_d_cliques_count += (GRuint)critical_cliques[d][i].size();

						GRuint voxel_id_from_critical_clique = (this->*Select)(critical_cliques[d][i]);

						//select a voxel from the current clique
						voxel_id_to_coordinates(voxel_id_from_critical_clique, x, y, z);
						//IF_DEBUG_DO(std::cout << "			selected voxel from "<<d<<"-clique " << i << " : " << voxel_id_from_critical_clique << " ; " << x << " " << y << " " << z << std::endl;)
						
						//if it hasn't already been selected we add it to Z
						if (!voxel(voxel_id_from_critical_clique).selected_) {
							//IF_DEBUG_DO(std::cout << "				newly selected, added to Z" << std::endl;)
							
							voxels_[voxel_id_from_critical_clique].selected_ = true;
							voxel_set_Z.push_back(voxel_id_from_critical_clique);
						}
					}
					
					//and add the voxels in Z to Y
					for (GRuint i(0); i < voxel_set_Z.size(); i++) {
						voxel_set_Y.push_back(voxel_set_Z[i]);
						//IF_DEBUG_DO(std::cout << "			added voxel " << voxel_set_Z[i] << " to Y" << std::endl;)
					}

					//IF_DEBUG_DO(std::cout << std::endl;)
				}
				IF_DEBUG_DO(std::cout << "	Y now contains " << voxel_set_Y.size() << " voxels : " << std::endl;)


				//If no critical clique could be found we probably encountered a special case
				//e.g. "dunce hat". In that case we do not replace the voxel set and leave
					//it like it is
				if (voxel_set_Y.size() == 0) {
					stability = true;
				}
				else {

					//replace the previous voxel_set
					remove_all_voxels();
					//std::cout << "	removed all voxels, size : "<<true_voxels_.size() << std::endl;
					for (GRuint i(0); i < voxel_set_Y.size(); i++) {
						set_voxel(voxel_set_Y[i]);
					}
					GRuint removed_count = voxel_count_at_iteration_start - (GRuint)true_voxels_.size();
					//std::cout << "	and replaced them with Y" << std::endl;

					//and re-select the voxels in K (useful for the last step)
					for (GRuint i(0); i < voxel_set_K.size(); i++) {
						voxels_[voxel_set_K[i]].selected_ = true;
					}
					//std::cout << "	voxels in K are selected again " << std::endl;

					for (GRuint i(0); i < true_voxels_.size(); i++) {
						//if a voxel is selected it is because it's in K (from the previous loop)
						if (!voxel(true_voxels_[i]).selected_ && (this->*Skel)(true_voxels_[i])) {
							voxel_set_K.push_back(true_voxels_[i]);
						}
					}
					//and then un-select the voxels in K
					for (GRuint i(0); i < voxel_set_K.size(); i++) {
						voxels_[voxel_set_K[i]].selected_ = false;
					}
					IF_DEBUG_DO(std::cout << "	K now contains " << voxel_set_K.size() << " voxels " << std::endl;)

					IF_DEBUG_DO(std::cout << "	removed " << removed_count << " voxels at iteration " << iteration_count << std::endl << std::endl;;)

					//stability check based on added count
						
					stability = (removed_count == 0);
				}
			}
		}

		
		
		
		/***********************************************************************************************/
		/******************************************************************* SKELETON-WISE OPERATIONS **/
		/***********************************************************************************************/

		VoxelSkeleton* subdivide(GRuint subdivision_level) {
			if (subdivision_level > 3) {
				std::cerr << " subdividing in more that 3 is too risky performance-wise. returning nullptr" << std::endl;
				return nullptr;
			}

			VoxelSkeleton* subdivided_skeleton = new VoxelSkeleton(
				width_*subdivision_level,
				height_*subdivision_level,
				slice_*subdivision_level);

			GRuint x, y, z;
			for (auto voxel_id : true_voxels_) {
				voxel_id_to_coordinates(voxel_id, x, y, z);

				for (GRuint i(0); i < subdivision_level; i++) {
					for (GRuint j(0); j < subdivision_level; j++) {
						for (GRuint k(0); k < subdivision_level; k++) {
							subdivided_skeleton->set_voxel(
								subdivision_level * x + i,
								subdivision_level * y + j,
								subdivision_level * z + k);
						}
					}
				}
			}

			return subdivided_skeleton;
		}

		VoxelSkeleton* subdivide_smooth() {
			VoxelSkeleton* subdivided_skeleton = this->subdivide(2);

			std::vector<GRuint> voxels_to_add;

			//look for all pairs of voxels that are not 2-connected in every direction
			for (auto voxel_id : subdivided_skeleton->true_voxels_) {
				GRuint x, y, z;
				subdivided_skeleton->voxel_id_to_coordinates(voxel_id, x, y, z);

				//checking the 1-neighborhood
				if (subdivided_skeleton->voxel(x + 1, y, z - 1).value_
					|| subdivided_skeleton->voxel(x - 1, y, z - 1).value_
					|| subdivided_skeleton->voxel(x, y + 1, z - 1).value_
					|| subdivided_skeleton->voxel(x, y - 1, z - 1).value_) {
					voxels_to_add.push_back(subdivided_skeleton->voxel_coordinates_to_id(x, y, z - 1));
				}
				if (subdivided_skeleton->voxel(x + 1, y, z + 1).value_
					|| subdivided_skeleton->voxel(x - 1, y, z + 1).value_
					|| subdivided_skeleton->voxel(x, y + 1, z + 1).value_
					|| subdivided_skeleton->voxel(x, y - 1, z + 1).value_) {
					voxels_to_add.push_back(subdivided_skeleton->voxel_coordinates_to_id(x, y, z + 1));
				}

				//and the 0-neighborhood
				for (GRuint i(0); i < 2; i++) {
					for (GRuint j(0); j < 2; j++) {
						for (GRuint k(0); k < 2; k++) {
							if (subdivided_skeleton->voxel(x - 1 + i*2, y - 1 + j*2, z -1 + k*2).value_) {
								voxels_to_add.push_back(subdivided_skeleton->voxel_coordinates_to_id(x, y, z - 1 + k * 2));
								voxels_to_add.push_back(subdivided_skeleton->voxel_coordinates_to_id(x - 1 + i * 2, y, z - 1 + k * 2));
								voxels_to_add.push_back(subdivided_skeleton->voxel_coordinates_to_id(x, y - 1 + j * 2, z - 1 + k * 2));
							}
						}
					}
				}
			}

			//and then add all the listed voxels
			for (auto voxel_id : voxels_to_add) {
				subdivided_skeleton->set_voxel(voxel_id);
			}

			return subdivided_skeleton;
		}

		/* Smoothind of the 0-connected max_distance neighborhood.
		0.5 threshold means that half of the neighborhood must be set. i.e. each voxel will take the value of the majority over its neighbors**/
		VoxelSkeleton* smooth_moving_average(GRuint max_distance, GRfloat threshold = 0.5f) {
			if (max_distance > 2) {
				throw std::invalid_argument("You are trying to smooth over a distance greater than 2. That means smoothing over at least 125 voxels. This is not allowed");
			}
			if (threshold < MIN_SMOOTHING_THRESHOLD) {
				throw std::invalid_argument("You are trying to smooth with a threshold lower than the minimum threshold");
			}
			if (threshold > 1.f) {
				throw std::invalid_argument("You are trying to smooth with a threshold greater than 1");
			}

			IndexVector to_set;
			IndexVector to_unset;

			std::vector<bool> in_to_check(voxel_count());

			IndexVector to_check;

			VoxelSkeleton* smoothed_skeleton = copy();

			GRuint count(0);

			//first establish the list of voxels to treat. 
			//That is, the border voxels and their neighborhood
			//we also set all the current true voxels to save the trouble of checking the interior voxels
			for (auto voxel_id : true_voxels_) {
				GRuint x, y, z;
				voxel_id_to_coordinates(voxel_id, x, y, z);

				IndexVector neighborhood;
				extract_0_neighborhood_star(voxel_id, neighborhood, true);

				if (neighborhood.size()) {
					smoothed_skeleton->set_voxel(voxel_id);
					for (GRuint i(0); i <= max_distance * 2; i++) {
						for (GRuint j(0); j <= max_distance * 2; j++) {
							for (GRuint k(0); k <= max_distance * 2; k++) {
								GRuint neighbor_id = voxel_coordinates_to_id(x - max_distance + i, y - max_distance + j, z - max_distance + k);

								if (neighbor_id < voxel_count() && !in_to_check[neighbor_id]) {
									in_to_check[neighbor_id] = true;
									to_check.push_back(neighbor_id);
								}
							}
						}
					}
				}
			}

			//std::cout << "voxels to check : " << to_check.size() << std::endl;

			for (auto voxel_id : to_check){
				GRuint x, y, z;
				voxel_id_to_coordinates(voxel_id, x, y, z);

				GRfloat average(0.f);
				GRuint neighbors_count(0);

				for (GRuint i(0); i <= max_distance*2; i++) {
					for (GRuint j(0); j <= max_distance*2; j++) {
						for (GRuint k(0); k <= max_distance*2; k++) {
							average += voxel(x - max_distance + i, y - max_distance + j, z - max_distance + k).value_;
							neighbors_count++;
						}
					}
				}
				count = neighbors_count;

				average /= (GRfloat)neighbors_count;

				if (average >= threshold) {
					to_set.push_back(voxel_id);
				}
				else {
					to_unset.push_back(voxel_id);
				}
			}

			for (auto to_set_id : to_set) {
				smoothed_skeleton->set_voxel(to_set_id, true);
			}

			for (auto to_unset_id : to_unset) {
				smoothed_skeleton->set_voxel(to_unset_id, false);
			}


			return smoothed_skeleton;
		}


		/** Returns a new skeleton of just the right size to contain the current skeleton */
		VoxelSkeleton* fit_to_min_max() {

			GRuint x_min(width_), y_min(height_), z_min(slice_);
			GRuint x_max(0), y_max(0), z_max(0);

			GRuint x, y, z;
			for (auto voxel_id : true_voxels_) {
				voxel_id_to_coordinates(voxel_id, x, y, z);
				x_min = MIN(x, x_min);
				y_min = MIN(y, y_min);
				z_min = MIN(z, z_min);

				x_max = MAX(x, x_max);
				y_max = MAX(y, y_max);
				z_max = MAX(z, z_max);
				//std::cout << " voxel : " << x << " " << y << " " << z << std::endl;
			}

			GRuint new_width  = x_max - x_min;
			GRuint new_height = y_max - y_min;
			GRuint new_slice  = z_max - z_min;

			//std::cout << "max : " << x_max << " " << y_max << " " << z_max << std::endl;
		//	std::cout << "min : " << x_min << " " << y_min << " " << z_min << std::endl;

			VoxelSkeleton* fit_skeleton = new VoxelSkeleton(new_width, new_height, new_slice);

			for (auto voxel_id : true_voxels_) {
				voxel_id_to_coordinates(voxel_id, x, y, z);

				fit_skeleton->set_voxel(x - x_min, y - y_min, z - z_min);
			}

			return fit_skeleton;
		}


		/** returns an allocated copy of this skeleton*/
		VoxelSkeleton* copy() {
			VoxelSkeleton* skeleton_copy = new VoxelSkeleton(width_-2, height_-2, slice_-2);
			for (auto voxel_id : true_voxels_) {
				skeleton_copy->set_voxel(voxel_id);
			}

			return skeleton_copy;
		}


		SkeletalGraph* extract_skeletal_graph(
			VoxelSkeleton* original_voxel_set = nullptr,
			DiscreteCurve::CONVERSION_METHOD spline_extraction_method = DiscreteCurve::CURVE_FITTING,
			GRuint smoothing_window_width = 5,
			GRfloat curve_fitting_max_error = 0.1f) {

			bool debug_log(false);

			typedef enum{ISOLATED, TERMINAL, BRANCH, JUNCTION, UNCLASSIFIED} VOXEL_CLASS;

			bool original_voxel_set_is_correct(false);

			if (original_voxel_set != nullptr 
				&& this->same_dimensions_as(original_voxel_set)) {
				original_voxel_set_is_correct = true;
			}

			//create a new graph
			SkeletalGraph* graph = new SkeletalGraph();

			if (!set_voxel_count()) {
				std::cerr << "Cannot extract graph from empty skeleton. Returning empty graph" << std::endl;
				return graph;
			}

			GRuint true_voxel_count((GRuint)true_voxels_.size());
			/*the expected total count of treated voxels.
			It's one for the terminal and edge voxels and how many
			neighbors they have for junction voxels*/
			GRuint expected_total_treated_count(0);

			/*NOTE : most of the following vectors have the same size of the
			total number of voxel in this skeleton to allow for O(1) lookups
			e.g. : 
			GRuint first_voxel_id = true_voxels_[i];
			labels[first_voxel_id] = 2;
			*/

			//the number of neighbors for each true voxel
			std::vector<GRuint> labels(nb_voxels_, 0);

			//the class (see above) of each true voxel
			std::vector<VOXEL_CLASS> classes(nb_voxels_, UNCLASSIFIED);

			//the expected treated count for each true voxel
			std::vector<GRuint> expected_treated_count(nb_voxels_, 0);

			/*the current treated count for each true voxel. 
			The goal is to have those values match those of the vector right above*/
			std::vector<GRuint> treated_count(nb_voxels_, 0);

			/*a vector containing the vertexDescriptors of the voxels 
			identified as vertices (terminal, junction and isolated voxels)*/
			std::vector<VertexDescriptor> vertices(nb_voxels_);

			/*a vector containing the curves of the edges that will be added to the graph*/
			std::vector<DiscreteCurve> edge_curves;

			std::vector<std::pair<VertexDescriptor, VertexDescriptor>> edges_source_targets;

			/*whether a particular voxel is a vertex or not*/
			std::vector<bool> is_vertex(nb_voxels_, false);
			/*for each vertex, which was the incident voxel.
			This is used to avoid adding twice the same edge*/
			IndexVector incident_voxel(nb_voxels_, 0);

			IndexVector terminal_points_ids;

			GRuint total_treated_count(0);

			GRuint first_terminal_id(0);

			//in case there are no terminal voxels (i.e. only junctions and branches)
			GRuint back_up_terminal_id(0);

			//In case there are no vertices (i.e. only branches)
			bool at_least_one_non_branch_voxel(false);

			IF_DEBUG_DO(std::cout << "Identified the terminal points : " << std::endl;)


			//first step : label voxels depending on their neighborhood
			//TODO : parallelize
			for (auto voxel_id : true_voxels_) {
				IndexVector neighborhood;

				extract_0_neighborhood_star(voxel_id, neighborhood);

				labels[voxel_id] = (GRuint)neighborhood.size(); 
				classes[voxel_id] = (VOXEL_CLASS)labels[voxel_id];

				if (labels[voxel_id] >= 4) {
					classes[voxel_id] = JUNCTION;
				}

				GRuint x, y, z;
				voxel_id_to_coordinates(voxel_id, x, y, z);

				expected_treated_count[voxel_id] = (labels[voxel_id] == 2 ? 1 : labels[voxel_id]);

				expected_total_treated_count += expected_treated_count[voxel_id];

				if (classes[voxel_id] == JUNCTION || classes[voxel_id] == TERMINAL || classes[voxel_id] == ISOLATED) {
					at_least_one_non_branch_voxel = true;
					back_up_terminal_id = voxel_id;

					Vector3f vertex_point;
					vertex_point.X() = (GRfloat)x;
					vertex_point.Y() = (GRfloat)y;
					vertex_point.Z() = (GRfloat)z;

					VertexProperties vertex_properties;
					vertex_properties.position = vertex_point;
					if (original_voxel_set_is_correct) {
						vertex_properties.radius = original_voxel_set->min_voxel_radius(voxel_id);
					}
					else {
						vertex_properties.radius = DEFAULT_VERTEX_RADIUS;
					}

					vertices[voxel_id] = graph->add_vertex(vertex_properties);
					is_vertex[voxel_id] = true;

					if (classes[voxel_id] == TERMINAL || classes[voxel_id] == ISOLATED) {
						terminal_points_ids.push_back(voxel_id);
						if (!first_terminal_id) {
							first_terminal_id = voxel_id;
						}
						IF_DEBUG_DO(std::cout << voxel_id << " : (" << x << " " << y << " " << z << ") : " << expected_treated_count[voxel_id] << std::endl;)
					}
				}

			}

			IF_DEBUG_DO(std::cout << "labels/classes : " << std::endl;)
			for (auto voxel_id : true_voxels_) {
				GRuint x, y, z;
				voxel_id_to_coordinates(voxel_id, x, y, z);
				IF_DEBUG_DO(std::cout << " voxel : " << voxel_id << " : (" << x << " " << y << " " << z << ") : " << labels[voxel_id] << " / " << classes[voxel_id] << std::endl;)
			}

			//If we have found no terminal vertex
			if (!terminal_points_ids.size()) {
				//if there's at least one non-branch voxel we use the back-up id
				if (at_least_one_non_branch_voxel) {
					first_terminal_id = back_up_terminal_id;
				}
				//and if there are no vertex at all then we use the first true voxel
				else {
					GRuint first_voxel_id = true_voxels_[0];
					GRuint x, y, z;
					voxel_id_to_coordinates(first_voxel_id, x, y, z);

					Vector3f vertex_point;
					vertex_point.X() = (GRfloat)x;
					vertex_point.Y() = (GRfloat)y;
					vertex_point.Z() = (GRfloat)z;

					VertexProperties vertex_properties;
					vertex_properties.position = vertex_point;
					vertex_properties.radius = DEFAULT_VERTEX_RADIUS;

					vertices[first_voxel_id] = graph->add_vertex(vertex_properties);
					is_vertex[first_voxel_id] = true;
					expected_treated_count[first_voxel_id]++;

					first_terminal_id = first_voxel_id;
				}
			}

			IndexVector starts;

			starts.push_back(first_terminal_id);

			//this is just for debugging
			GRuint iteration_count(0);

			
			IF_DEBUG_DO(std::cout<<"true voxel count : "<<true_voxel_count<<std::endl;)
			IF_DEBUG_DO(std::cout << "expected total treated count : " << expected_total_treated_count << std::endl;)
			
			//we look for edges while the expecte total count is not reached	
			while (total_treated_count < expected_total_treated_count) {
				
				IF_DEBUG_DO(std::cout << " Starting iteration " << iteration_count << std::endl);
				IF_DEBUG_DO(std::cout << "	treated count : " << total_treated_count << std::endl);

				/*If there are no starts left, it means we hav fully treated the current 
				connected component so we look for a new one*/
				if (!starts.size()) {
					IF_DEBUG_DO(std::cout << "		no more ids in the start ids. Looking for a new one" << std::endl;)
						GRuint j(0);
					//find the first vertex id that hasn't be fully treated
					while (j < (GRuint)terminal_points_ids.size() 
						&& (treated_count[terminal_points_ids[j]] == expected_treated_count[terminal_points_ids[j]])) {
						j++;
					}
					if (j < (GRuint)terminal_points_ids.size()) {
						std::cout << "			added voxel " << terminal_points_ids[j] << " to the starting points" << std::endl;
						starts.push_back(terminal_points_ids[j]);
					}
					else {
						IF_DEBUG_DO(std::cout << " ERROR : Could not find new untreated vertex to start with" << std::endl;)
							break;
					}
				}

				IF_DEBUG_DO(std::cout << "		list of starting points : " << std::endl);
				for (auto start_id : starts) {
					GRuint x, y, z;
					voxel_id_to_coordinates(start_id, x, y, z);
					IF_DEBUG_DO(std::cout << "			"<<start_id<<" : (" << x << " " << y << " " << z << ")" << std::endl);
				}

				IndexVector next_starts;
				//for each starting points, we look for the next vertex
				for (auto start_id : starts) {
					if (treated_count[start_id] != expected_treated_count[start_id]) {
						IF_DEBUG_DO(std::cout << "		exploring from id " << start_id << std::endl);


						IndexVector neighborhood;
						IndexVector untreated_neighborhood;
						extract_0_neighborhood_star(start_id, neighborhood);
						IF_DEBUG_DO(std::cout << "		untreated neighbors : " << std::endl);

						for (auto neighbor_id : neighborhood) {
							if (treated_count[neighbor_id] != expected_treated_count[neighbor_id]) {

								untreated_neighborhood.push_back(neighbor_id);

								GRuint x, y, z;
								voxel_id_to_coordinates(neighbor_id, x, y, z);
								IF_DEBUG_DO(std::cout << "		" << neighbor_id << " : (" << x << " " << y << " " << z << ")" << std::endl);
							}
						}

						//among all the adjacent voxels, 
						for (auto untreated_neighbor_id : untreated_neighborhood) {

							GRuint x, y, z;
							voxel_id_to_coordinates(start_id, x, y, z);

							DiscreteCurve discrete_edge_curve;
							//adding the source voxel's position to the edge's curve
							discrete_edge_curve.push_back(Vector3f((GRfloat)x, (GRfloat)y, (GRfloat)z));

							IF_DEBUG_DO(std::cout << "		starting from id " << untreated_neighbor_id << std::endl;)
							bool found_vertex = false;
							GRuint current_id = untreated_neighbor_id;
							GRuint last_id = start_id;


							while (!found_vertex) {

								if (treated_count[current_id] != expected_treated_count[current_id]
									&& is_vertex[current_id]) {

									voxel_id_to_coordinates(current_id, x, y, z);

									bool already_in_next_starts(false);
									for (auto next_start_id : next_starts) {
										already_in_next_starts |= (next_start_id == current_id);
									}
									if (!already_in_next_starts) {
										next_starts.push_back(current_id);
									}

									treated_count[start_id]++;
									treated_count[current_id]++;
									total_treated_count += 2;

									found_vertex = true;

									//adding the target voxel's position to the edge's curve
									discrete_edge_curve.push_back(Vector3f((GRfloat)x, (GRfloat)y, (GRfloat)z));

									discrete_edge_curve.smooth_moving_average(smoothing_window_width);

									EdgeProperties edge_properties({ *(discrete_edge_curve.to_spline_curve(spline_extraction_method, &curve_fitting_max_error)) });

									graph->add_edge(vertices[start_id], vertices[current_id], edge_properties);

									IF_DEBUG_DO(std::cout << "			voxel " << current_id << " (" << x << " " << y << " " << z << ") is a vertex " << std::endl;)
									IF_DEBUG_DO(std::cout << "			added an edge from " << start_id << " to " << current_id << std::endl;)
									IF_DEBUG_DO(std::cout << "		    their treated count are now " << treated_count[start_id] << " and " << treated_count[current_id] << std::endl);

								}else if (treated_count[current_id] != expected_treated_count[current_id] 
									&& !is_vertex[current_id]) {

									GRuint x, y, z;
									voxel_id_to_coordinates(current_id, x, y, z);

									//if the neighbor is untreated and not a vertex, we add it to the edge's
									//curve and set the neighbor as the new current id

									IF_DEBUG_DO(std::cout << "			added voxel " << current_id << " (" << x << " " << y << " " << z << ") to edge list starting from " << start_id << std::endl);

									discrete_edge_curve.push_back(Vector3f((GRfloat)x, (GRfloat)y, (GRfloat)z));

									treated_count[current_id] = 1;
									total_treated_count++;

									IF_DEBUG_DO(std::cout << "			Now looking for the next untreated neighbor"<< std::endl;)


									IndexVector secondary_neighborhood;
									extract_0_neighborhood_star(current_id, secondary_neighborhood);
									GRuint expected_treated_neighbor_count(0);
									GRuint next_untreated_id(current_id);

									IF_DEBUG_DO(std::cout << "			secondary neighbors of " << current_id << " : " << std::endl);
									for (auto secondary_neighbor_id : secondary_neighborhood) {
										expected_treated_neighbor_count += expected_treated_count[secondary_neighbor_id];
										voxel_id_to_coordinates(secondary_neighbor_id, x, y, z);
										IF_DEBUG_DO(std::cout << "				" << secondary_neighbor_id << " : (" << x << " " << y << " " << z << ")" << std::endl);
										if (treated_count[secondary_neighbor_id] != expected_treated_count[secondary_neighbor_id]
											&& secondary_neighbor_id != last_id) {
											next_untreated_id = secondary_neighbor_id;
										}
									}

									IF_DEBUG_DO(std::cout << "			expected treated neighbor count : " << expected_treated_neighbor_count << std::endl);

									if (next_untreated_id == current_id) {
										IF_DEBUG_DO(std::cout << " ERROR : Could not find a non-fully-treated neighbor to " << current_id << std::endl;)
											found_vertex = true;
									}

									last_id = current_id;
									current_id = next_untreated_id;

									IF_DEBUG_DO(std::cout << "			last id is now : " << last_id << " and current id is : " << current_id << std::endl;)

								}
								else if (current_id == untreated_neighbor_id
									&& treated_count[current_id] == expected_treated_count[current_id]) {
									IF_DEBUG_DO(std::cout << " This path was already treated. This probably means that there is a loop from and to voxel "<<start_id<< std::endl;)
										found_vertex = true;
								} else{
										IF_DEBUG_DO(std::cout << " ERROR : Found a fully treatead vertex on a non-treated path" << std::endl;)
											found_vertex = true;
								}
							}
						}
					}
				}
				starts = next_starts;
				iteration_count++;
			}


			return graph;
		}




		/***********************************************************************************************/
		/************************************************************************ SKELETON GENERATORS **/
		/***********************************************************************************************/
		/**(random, vessel-like, Bertrand, etc.)*/

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

		void generate_single_edge_skeleton() {
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			//horizontal branch
			GRuint start_x(width_ / 2), start_y(height_ / 4), start_z(slice_ / 2);
			GRuint x = start_x;
			GRuint y = start_y;
			GRuint z = start_z;

			for (GRuint i(0); i < MIN(20,width_/2); i++) {
				set_voxel(x, y++, z);
			}

		}


		void generate_sinusoidal_skeleton() {
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			//horizontal branch
			GRuint start_x(width_/4), start_y(height_/2), start_z(slice_/2);
			GRuint x = start_x;
			GRuint y = start_y;
			GRuint z = start_z;

			for (GRuint j(0); j < 5; j++) {
				for (GRuint i(0); i < 5; i++) {
					set_voxel(x++, y--, z++);
					std::cout << "set voxel " << x << " " << y << " " << z << std::endl;
				}
				for (GRuint i(0); i < 5; i++) {
					set_voxel(x++, y++, z--);
					std::cout << "set voxel " << x << " " << y << " " << z << std::endl;
				}

				if (j == 3) {
					start_x = x;
					start_y = y;
					start_z = z;
				}
			}

			x = start_x;
			y = start_y;
			z = start_z;

			for (GRuint j(0); j < 5; j++) {
				for (GRuint i(0); i < 5; i++) {
					set_voxel(x--, y++, z++);
					std::cout << "set voxel " << x << " " << y << " " << z << std::endl;
				}
				for (GRuint i(0); i < 5; i++) {
					set_voxel(x++, y--, z++);
					std::cout << "set voxel " << x << " " << y << " " << z << std::endl;
				}
			}

		}

		/**Generates a random connected voxel set*/
		void generate_random(GRuint nb_voxels, GRuint seed = 1234) {
			srand(seed);

			//erasing voxel grid
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			GRuint current_voxel_id = voxel_coordinates_to_id(width_/2, height_/2, slice_/2);


			GRuint x, y, z;

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


		void generate_predefined_thicken_skeleton(GRuint nb_voxels, GRuint seed = 1234) {
			srand(seed);

			//erasing voxel grid
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			//phase 1 : generate skeleton
			GRuint x(width_/2), y(height_/2), z(slice_/2);

			for (GRuint i(0); i < 5; i++) {
				set_voxel(x, y, z+i);
			}
			set_voxel(x + 2, y + 2, z);

			for (GRuint i(1); i < 5; i++) {
				set_voxel(x - i, y, z + i);
				set_voxel(x + i, y, z + i);
			}


			//phase 2: thicken skeleton
			for (GRuint i((GRuint)true_voxels_.size()); i < nb_voxels; i++) {
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
					//std::cout << "random voxel id : " << random_voxel_id << std::endl;

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

				//std::cout << "set voxel : " << next_voxel_id << std::endl;
				set_voxel(next_voxel_id);

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
					set_anchor_voxel(current_voxel_id);

					current_voxel_id = true_voxels_[rand() % true_voxels_.size()];
					//std::cout << "branching at iteration " << i << std::endl;
				//	std::cout << "voxel id : " << current_voxel_id << std::endl;
				}
			}

			//std::cout << "done generating skeleton. Now thickening it" << std::endl;

			//phase 2: thicken skeleton
			for (GRuint i((GRuint)true_voxels_.size()); i < nb_voxels; i++) {
				//std::cout << "true voxel count : " << true_voxels_.size() << std::endl;

				GRint next_voxel_id = -1;
				GRuint random_voxel_index = rand() % true_voxels_.size();
				GRuint random_voxel_id = true_voxels_[random_voxel_index];
				//std::cout << "random index : " << random_voxel_index << std::endl;
				//std::cout << "true voxels size : " << true_voxels_.size() << std::endl;
				//std::cout << "random voxels id : " << random_voxel_id << std::endl;


				GRuint face_index = rand() % 6;

				switch (face_index) {
				case 0: next_voxel_id = random_voxel_id + 1; break;//right
				case 1: next_voxel_id = random_voxel_id - 1; break;//left
				case 2: next_voxel_id = random_voxel_id + width_; break;//back
				case 3: next_voxel_id = random_voxel_id - width_; break;//front
				case 4: next_voxel_id = random_voxel_id + height_*width_; break;//top
				case 5: next_voxel_id = random_voxel_id - height_*width_; break;//bottom
				}

				set_voxel(next_voxel_id);				
			//	std::cout << "set voxel : " << next_voxel_id << std::endl;
			}
		}


		void generate_artificial_simple_kissing(GRuint edges_length) {
			if (width_ < 100 || height_ < 100) {
				std::cerr << "can't generate that structure mate" << std::endl;
				exit(EXIT_FAILURE);
			}
			memset(voxels_, 0, nb_voxels_ * sizeof(SkeletonVoxel));

			GRuint x_start(10);
			GRuint y_start(10);

			//bottom left branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + i, y_start + i);
			}

			//bottom right branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + edges_length*2 - i, y_start + i);
			}

			//bottom vertical branch
			for (GRuint i(0); i < edges_length*2; i++) {
				set_voxel(0, x_start + edges_length, y_start + edges_length + i);
			}

			//bottom left loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + edges_length-i, y_start + edges_length*3 + i);
			}

			//bottom right loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + edges_length + i, y_start + edges_length * 3 + i);
			}

			//left vertical loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start, y_start + edges_length * 4 + i);
			}

			//right vertical loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + edges_length*2, y_start + edges_length * 4 + i);
			}

			//upper left loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + i, y_start + edges_length * 5 + i);
			}

			//upper right loop branch
			for (GRuint i(0); i < edges_length; i++) {
				set_voxel(0, x_start + edges_length*2 - i, y_start + edges_length * 5 + i);
			}

			//upper vertical branch
			for (GRuint i(0); i < edges_length*2; i++) {
				set_voxel(0, x_start + edges_length, y_start + edges_length*6 + i);
			}
		}

	};

}
