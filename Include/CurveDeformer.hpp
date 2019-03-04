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


#include "curve.hpp"
#include "igl/arap.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace grapholon {
	
	class CurveDeformer {


	public:



		static Eigen::Vector3d to_eigen(Vector3f vec) {
			return Eigen::Vector3d(vec.X(), vec.Y(), vec.Z());
		}

		static Vector3f to_vec3(Eigen::Vector3d eigen) {
			return Vector3f(eigen.x(), eigen.y(), eigen.z());
		}

		/** Deforms the curve in a As-Rigid-As-Possible manner using the Interactive Geometry Library (IGL)*/
		static bool deform_curve(DeformableSplineCurve& in_curve, GRuint control_point_index, Vector3f target_position) {


			if (in_curve.size() < 4) {
				return false;
			}

			if (control_point_index > in_curve.size() - 1) {
				return false;
			}

			if (in_curve.original_points_.size() != in_curve.size()) {

				in_curve.set_original_shape();
			}


			std::vector<Vector3f> curve = in_curve.original_points_;

			Eigen::MatrixXd V(curve.size(), 3);
			Eigen::MatrixXd U(V.cols(), V.rows());
			Eigen::MatrixXi F((size_t)((GRdouble)curve.size() / 2.0), 3);


			for (GRuint i(0); i < curve.size(); i++) {
				V.row(i) = to_eigen(curve[i]);
			}

			for (GRuint i(0); i < F.rows() - 1; i++) {
				F.row(i) = Eigen::Vector3i(i * 2, i * 2 + 1, i * 2 + 2);
			}
			F.row(F.rows() - 1) = Eigen::Vector3i(V.rows() - 3, V.rows() - 2, V.rows() - 1);

			U = V;



			Eigen::VectorXi S, b;

			if (control_point_index == 0 || control_point_index == curve.size() - 1) {
				b = Eigen::VectorXi(2);
			}
			else {
				b = Eigen::VectorXi(3);
				b(2) = control_point_index;
			}

			b(0) = 0;
			b(1) = (int)(curve.size() - 1);

			Eigen::MatrixXd bc(b.size(), V.cols());

			Eigen::VectorXd new_position(to_eigen(target_position));

			if (control_point_index == 0) {
				bc.row(0) = new_position;
			}
			else {
				bc.row(0) = to_eigen(in_curve.front().first);
			}

			if (control_point_index == (V.rows() - 1)) {
				bc.row(1) = new_position;
			}else{
				bc.row(1) = to_eigen(in_curve.back().first);
			}

			if (bc.rows() == 3) {
				bc.row(2) = new_position;
			}


			/*std::cout << " V : " << V << std::endl;
			std::cout << "F : " << F << std::endl;
			std::cout << " b : " << b << std::endl;
			std::cout << " bc : " << bc << std::endl;*/


			igl::ARAPData arap_data;
			arap_data.energy = igl::ARAP_ENERGY_TYPE_SPOKES;
			arap_data.max_iter = 100;
			arap_data.with_dynamics = true;

			igl::arap_precomputation(V, F, V.cols(), b, arap_data);


			igl::arap_solve(bc, arap_data, U);

			for (GRuint i(0); i < V.rows(); i++) {
				in_curve[i].first = to_vec3(U.row(i));
			}
			in_curve.update_tangents();


			return true;
		}

	};
};
