#pragma once

#include "Eigen/SparseQR"
#include "curve.hpp"

namespace grapholon {

	typedef Eigen::SparseMatrix<GRfloat> EigenSparseMatrix;
	typedef Eigen::Matrix3f EigenMatrix3;
	typedef Eigen::Vector3f EigenVector3;
	typedef Eigen::VectorXf EigenVector;
	typedef Eigen::SparseQR<EigenSparseMatrix, Eigen::COLAMDOrdering<int>> EigenSolver;


	template<class _CURVE_TYPE>
	class CurveDeformer {

		const bool USE_L0 = true;
		const bool USE_L0_R = false;
		//const bool SHOW_FRAMES = true;
		const float W = 0.1f;//USE_L0 ? 1 : 0.1;
		const bool ENABLE_FINAL_L1_ADJUST = true;

		GRuint n_laplacians_P_;	// = n_involved -2;
		GRuint n_laplacians_R_;
		GRuint n_involved_;		// = n_laplacians +2;
		GRuint n_fixed_;		// 5 (or 4,3)

		GRuint index_low_involved_;
		GRuint index_high_involved_;

		//not sure about int
		std::vector<GRuint> fixed_vertices_;
		std::vector<bool> fixed_;

		std::vector<EigenVector3> vs_;
		std::vector<EigenVector3> original_vs_;

		std::vector<EigenVector3> original_laplacians_;
		std::vector<EigenVector3> original_laplacians_L1;
		GRuint n_laplacians_;
		GRuint final_laplacian_L1_adjust_id_ = (GRuint)-1;

		//static GRuint A = -1;

		EigenSolver solver_;

		EigenSparseMatrix A_;
		EigenVector B_;

		EigenVector vxyz_rxyz_;

		std::vector<EigenMatrix3> R_;

		const GRuint CONSTRAINT_WEIGHT = 10000;

	public:


		void compile(_CURVE_TYPE& path, GRuint index_low, GRuint index_handle, GRuint index_high) {

			index_low_involved_ = index_low - 1;
			index_high_involved_ = index_high + 1;

			if (!index_low) {
				index_low_involved_ = 0;
			}
			if (index_high == path.size() - 1) {
				index_high_involved_ = index_high;
			}


			n_involved_ = index_high_involved_ - index_low_involved_ + 1;
			n_laplacians_P_ = USE_L0 ? n_involved_ - 1 : n_involved_ - 2;
			n_laplacians_R_ = USE_L0_R ? n_involved_ - 1 : n_involved_ - 2;

			//involved vertices
			vs_ = std::vector<EigenVector3>(n_involved_);
			for (GRuint i(index_low_involved_); i <= index_high_involved_; i++) {
				vs_[i - index_low_involved_] = EigenVector3(path[i].first.X(), path[i].first.Y(), path[i].first.Z());
			}

			fixed_vertices_ = std::vector<GRuint>();
			fixed_vertices_.push_back(index_low_involved_);

			if (index_low != index_low_involved_){
				fixed_vertices_.push_back(index_low);
			}
			if (index_handle > index_low && index_handle < index_high) {
				fixed_vertices_.push_back(index_handle);
			}
			if (index_high != index_high_involved_) {
				fixed_vertices_.push_back(index_high);
			}
			fixed_vertices_.push_back(index_high_involved_);

			n_fixed_ = (GRuint) fixed_vertices_.size();

			fixed_ = std::vector<bool>(n_involved_);
			fixed_[index_low_involved_ - index_low_involved_] = true;
			fixed_[index_low - index_low_involved_] = true;
			fixed_[index_low - index_low_involved_] = true;
			fixed_[index_handle - index_low_involved_] = true;
			fixed_[index_high_involved_ - index_low_involved_] = true;

			original_laplacians_ = std::vector<EigenVector3>(n_laplacians_P_);
			for (GRuint i(0); i < n_laplacians_P_; i++) {
				if (USE_L0) {
					original_laplacians_[i] = vs_[i + 1] - vs_[i];
				}
				else {
					original_laplacians_[i] = vs_[i + 1] - (vs_[i] + vs_[i+2])*0.5f;
				}
			}

			if (USE_L0 && ENABLE_FINAL_L1_ADJUST) {
				prepare_for_laplacian_L1_adjust();
			}

			R_ = std::vector<EigenMatrix3>(n_involved_);
			for (auto& single_R : R_) {
				single_R = EigenMatrix3::Identity();
			}

			original_vs_ = vs_;

			/*original_vs = new Vertex[n_involved];
			for (int i = 0; i < vs.length; i++) {
				original_vs[i] = new Vertex(vs[i]);
			}*/

			construct_A_and_B();
		}


		void update() {

			// start from rest state, discard previous deformation
			// slower (need more iterations) but more stable
			for (auto& single_R : R_) {
				single_R = EigenMatrix3::Identity();
			}

			for (GRuint i(0); i < 4; i++) {
				solve();
				update_R();
				update_B();
			}


			if (USE_L0 && ENABLE_FINAL_L1_ADJUST) {
				final_laplacian_L1_adjust();
			}
			else {
				set_positions();
			}
		}

		void construct_A_and_B() {
			//big TODO
		}

		void update_B() {

			//  // L v = dR * R * L v0
			//  L v = dR * L v0
			if (USE_L0) {
				for (GRuint i(0); i < n_laplacians_P_; i++) {
					EigenMatrix3 current_R = R_[i];
					EigenVector3 l0 = original_laplacians_[i];
					for (GRuint j(0); j < 3; j++) {
						B_[i * 3 + j] = EigenVector3(current_R.block(0, j, 1, 3)).dot(l0);
					}
					
				}
			}
		}

		/*
#include <Eigen/RequiredModuleName>
		// ...
		SparseMatrix<double> A;
	// fill A
	VectorXd b, x;
	// fill b
	// solve Ax = b
	SolverClassName<SparseMatrix<double> > solver;
	solver.compute(A);
	if (solver.info() != Success) {
		// decomposition failed
		return;
	}
	x = solver.solve(b);
	if (solver.info() != Success) {
		// solving failed
		return;
	}
	// solve for another right hand side:
	x1 = solver.solve(b1);
	*/

		void solve() {
			solver_.compute(A_);
			if (solver_.info() != Eigen::Success) {
				std::cerr << " ERROR - CurveDeformer solving failed" << std::endl;
				return;
			}
			vxyz_rxyz_ = solver_.solve(B_);
		}

		void update_R() {

		}

		EigenSparseMatrix cross_multiply(GRfloat rx, GRfloat ry, GRfloat rz, EigenSparseMatrix A) {

			return A;
		}

		EigenSparseMatrix get_closest_orthonormal(EigenSparseMatrix A) {

			return A;
		}

		void set_positions() {

		}


		void prepare_for_laplacian_L1_adjust() {

		}

		void final_laplacian_L1_adjust() {

		}




	};
};
