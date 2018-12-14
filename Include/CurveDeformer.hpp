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

		/*static Point_3 to_point3(Vector3f vec) {
			return Point_3(vec.X(), vec.Y(), vec.Z());
		}

		static Vector3f to_vec3(Point_3 point) {
			return Vector3f(point.x(), point.y(), point.z());
		}*/

		static Eigen::Vector3d to_eigen(Vector3f vec) {
			return Eigen::Vector3d(vec.X(), vec.Y(), vec.Z());
		}

		static Vector3f to_vec3(Eigen::Vector3d eigen) {
			return Vector3f(eigen.x(), eigen.y(), eigen.z());
		}

		static bool deform_curve(DeformableSplineCurve& curve, bool source_control_point, Vector3f target_position) {


			Eigen::MatrixXd V(curve.size(), 3);
			Eigen::MatrixXd U(curve.size(), 3);
			Eigen::MatrixXi F(curve.size()-2, 3);

			Eigen::VectorXi S, b(2);
			Eigen::MatrixXd bc(b.size(), V.cols());

			for (GRuint i(0); i < curve.size(); i++) {
				V.row(i) = to_eigen(curve[i].first);
			}

			for (GRuint i(1); i < curve.size() - 1; i++) {
				F.row(i-1) = Eigen::Vector3i(i, i - 1, i + 1);
			}

			U = V;

			b(0) = 0 ;
			b(1) = (int)(curve.size() - 1);

			Eigen::VectorXd new_position(to_eigen(target_position));

			bc.row(0) = source_control_point ? new_position : V.row(0);
			bc.row(1) = source_control_point ? V.row(V.rows()-1) : new_position;


			/*std::cout << " V : " << V << std::endl;
			std::cout << "F : " << F << std::endl;
			std::cout << " b : " << b << std::endl;
			std::cout << " bc : " << bc << std::endl;*/


			igl::ARAPData arap_data;
			arap_data.energy = igl::ARAP_ENERGY_TYPE_ELEMENTS;
			arap_data.max_iter = 100;
			arap_data.with_dynamics = true;

			igl::arap_precomputation(V, F, V.cols(), b, arap_data);


			igl::arap_solve(bc, arap_data, U);

			for (GRuint i(0); i < V.rows(); i++) {
				curve[i].first = to_vec3(U.row(i));
			}
			curve.update_tangents();


			return true;
		}

	};
};
