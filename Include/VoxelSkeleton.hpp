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

#include "GrapholonTypes.hpp"
#include "common.hpp"

namespace grapholon {

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

	enum Clique{CLIQUE3, CLIQUE2, CLIQUE1, CLIQUE0, NO_CLIQUE};

	struct SkeletonVoxel {
		bool value_ = false;
	};



	class VoxelSkeleton {
	public:

		const GRuint width_;
		const GRuint height_;
		const GRuint slice_;

		SkeletonVoxel* voxels_;

		VoxelSkeleton(GRuint width, GRuint height, GRuint slice) : width_(width), height_(height), slice_(slice) {
			voxels_ = (SkeletonVoxel*)calloc(width_*height_*slice_, sizeof(SkeletonVoxel));
			memset(voxels_, false, sizeof(SkeletonVoxel));
		}

		~VoxelSkeleton(){
			free(voxels_);
		}

		SkeletonVoxel& voxel(GRuint x, GRuint y, GRuint z) {
			return voxels_[voxel_coordinates_to_id(x, y, z)];
		}

		const SkeletonVoxel& voxel(GRuint x, GRuint y, GRuint z) const{
			return voxels_[voxel_coordinates_to_id(x, y, z)];
		}

		GRuint voxel_coordinates_to_id(GRuint x, GRuint y, GRuint z) const {
			return x + (y + z * height_) * width_;
		}

		/**
		current convention : if the relative coordinates are out-of-bounds they're clamped to within-bounds coordinates
		e.g. (-1, 3, 4) -> (0, 3, 4);  (5, 2, slice + 3) -> (5, 2, slice - 1)*/
		GRuint voxel_coordinates_and_relative_coordinates_to_id(GRuint x, GRuint y, GRuint z, GRint rel_x, GRint rel_y, GRint rel_z) const {
			GRint final_x = x + rel_x;
			GRint final_y = x + rel_y;
			GRint final_z = x + rel_z;

			final_x = MAX(0, MIN(final_x, width_ - 1));
			final_y = MAX(0, MIN(final_y, height_ - 1));
			final_z = MAX(0, MIN(final_z, slice_ - 1));

			return voxel_coordinates_to_id(final_x, final_y, final_z);
		}
		
		void voxel_id_to_coordinates(GRuint id, GRuint& x, GRuint& y, GRuint& z) const {
			z = id / (width_ * height_);
			GRint rem = id % (width_ * height_);
			y = rem / width_;
			x = rem % width_;
		}

		GRuint voxel_id_and_relative_coordinates_to_id(GRuint id, GRint rel_x, GRint rel_y, GRint rel_z) {
			GRuint x, y, z;
			voxel_id_to_coordinates(id, x, y, z);
			return voxel_coordinates_and_relative_coordinates_to_id(x, y, z, rel_x, rel_y, rel_z);
		}

		bool set_voxel(GRuint x, GRuint y, GRuint z, bool value = true) {
			if (x >= width_ || y >= height_ || z >= slice_) {
				return false;
			}
			voxels_[voxel_coordinates_to_id(x,y,z)].value_ = value;

			return true;
		}

		bool match_mask(GRuint x, GRuint y, GRuint z, GRuint mask_width, GRuint mask_height, GRuint mask_slice, SkeletonVoxel*** mask) {

			return true;
		}

	};

}
