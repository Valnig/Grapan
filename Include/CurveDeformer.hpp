#pragma once

#include "Eigen/SparseQR"
#include "Eigen/SVD"
#include "curve.hpp"

namespace grapholon {

	typedef Eigen::SparseMatrix<GRfloat> EigenSparseMatrix;
	typedef Eigen::MatrixXf EigenMatrix;
	typedef Eigen::Vector3f EigenVector3;
	typedef Eigen::VectorXf EigenVector;
	typedef Eigen::SparseQR<EigenSparseMatrix, Eigen::COLAMDOrdering<int>> EigenSolver;
	typedef Eigen::JacobiSVD<EigenMatrix> EigenSVD;
	typedef Eigen::Triplet< GRfloat> EigenTriplet;

	template<class _CURVE_TYPE>
	class CurveDeformer {
	public:

		const bool USE_L0 = true;
		const bool USE_L0_R = false;
		//const bool SHOW_FRAMES = true;
		const float W = 0.1f;//USE_L0 ? 1 : 0.1;
		const bool ENABLE_FINAL_L1_ADJUST = false;

		GRuint n_laplacians_P;	// = n_involved -2;
		GRuint n_laplacians_R;
		GRuint n_involved;		// = n_laplacians +2;
		GRuint n_fixed;		// 5 (or 4,3)

		GRuint index_low_involved;
		GRuint index_high_involved;

		//not sure about int
		std::vector<GRuint> fixed_vertices;
		std::vector<bool> fixed_;

		std::vector<EigenVector3> vs;
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

		std::vector<EigenMatrix> R_;

		const GRfloat CONSTRAINT_WEIGHT = 10000.f;

	public:


		std::string to_string(EigenVector vec) {
			std::stringstream msg;
			msg << "( "<<vec.x() << ", " << vec.y() << ", " << vec.z() << " )";
			return msg.str();
		}

		void compile(_CURVE_TYPE& path, GRuint index_low, GRuint index_handle, GRuint index_high) {

			index_low_involved = index_low - 1;
			index_high_involved = index_high + 1;

			if (!index_low) {
				index_low_involved = 0;
			}
			if (index_high == path.size() - 1) {
				index_high_involved = index_high;
			}


			n_involved = index_high_involved - index_low_involved + 1;
			n_laplacians_P = USE_L0 ? n_involved - 1 : n_involved - 2;
			n_laplacians_R = USE_L0_R ? n_involved - 1 : n_involved - 2;

			std::cout << " n involved : " << n_involved << std::endl;
			std::cout << " n_laplacians_P : " << n_laplacians_P << std::endl;
			std::cout << " n_laplacians_R : " << n_laplacians_R << std::endl;

			//involved vertices
			vs = std::vector<EigenVector3>(n_involved);
			for (GRuint i(index_low_involved); i <= index_high_involved; i++) {
				vs[i - index_low_involved] = EigenVector3(path[i].X(), path[i].Y(), path[i].Z());
			}

			fixed_vertices = std::vector<GRuint>();
			fixed_vertices.push_back(index_low_involved);

			if (index_low != index_low_involved){
				fixed_vertices.push_back(index_low);
			}
			if (index_handle > index_low && index_handle < index_high) {
				fixed_vertices.push_back(index_handle);
			}
			if (index_high != index_high_involved) {
				fixed_vertices.push_back(index_high);
			}
			fixed_vertices.push_back(index_high_involved);

			n_fixed = (GRuint) fixed_vertices.size();

			fixed_ = std::vector<bool>(n_involved);
			fixed_[index_low_involved - index_low_involved] = true;
			fixed_[index_low - index_low_involved] = true;
			fixed_[index_low - index_low_involved] = true;
			fixed_[index_handle - index_low_involved] = true;
			fixed_[index_high_involved - index_low_involved] = true;

			original_laplacians_ = std::vector<EigenVector3>(n_laplacians_P);
			for (GRuint i(0); i < n_laplacians_P; i++) {
				if (USE_L0) {
					original_laplacians_[i] = vs[i + 1] - vs[i];
				}
				else {
					original_laplacians_[i] = vs[i + 1] - (vs[i] + vs[i+2])*0.5f;
				}
			}
			std::cout << "original laplacians : " << std::endl;
			for (auto lap : original_laplacians_) {
				std::cout<<to_string(lap)<<std::endl;
			}

			if (USE_L0 && ENABLE_FINAL_L1_ADJUST) {
				prepare_for_laplacian_L1_adjust();
			}

			R_ = std::vector<EigenMatrix>(n_involved);
			for (auto& single_R : R_) {
				single_R = EigenMatrix::Identity(3,3);
			}

			original_vs_ = vs;

			std::cout << "original vs : " << std::endl;
			for (auto v : original_vs_) {
				std::cout << to_string(v) << std::endl;
			}

			/*original_vs = new Vertex[n_involved];
			for (int i = 0; i < vs.length; i++) {
				original_vs[i] = new Vertex(vs[i]);
			}*/

			construct_A_and_B();
		}


		void update(_CURVE_TYPE& result_curve) {

			// start from rest state, discard previous deformation
			// slower (need more iterations) but more stable
			for (auto& single_R : R_) {
				single_R = EigenMatrix::Identity(3,3);
			}

			for (GRuint i(0); i < 1; i++) {
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

			result_curve.clear();
			for (auto position : vs) {
				result_curve.push_back({ position.x(), position.y(), position.z() });
			}
		}

		void construct_A_and_B() {

			A_ = EigenSparseMatrix(
				n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 2 * 3, 
				n_involved * 3 * 2 );

			B_ = EigenVector(
				n_laplacians_P * 3
				+ n_laplacians_R * 3
				+ n_fixed * 3
				+ 2 * 3);

			//	laplacian of position
			//  // L v = dR * R * L v0
			//  L v = dR * L v0

			std::vector<EigenTriplet> triplets;

			if (USE_L0) {
				for (GRuint i(0); i < n_laplacians_P; i++) {
					EigenVector3 l0 = original_laplacians_[i];

					//vix
					triplets.push_back(EigenTriplet(i * 3 + 0, (i + 0) * 3 + 0, -1));					// vix
					triplets.push_back(EigenTriplet(i * 3 + 0, (i + 1) * 3 + 0, 1));
					triplets.push_back(EigenTriplet(i * 3 + 0, n_involved * 3 + (i + 0) * 3 + 0, 0));	// rix
					triplets.push_back(EigenTriplet(i * 3 + 0, n_involved * 3 + (i + 0) * 3 + 1, -l0.z()));	//dot_product(r[2], l0)));	// riy
					triplets.push_back(EigenTriplet(i * 3 + 0, n_involved * 3 + (i + 0) * 3 + 2, l0.y()));	//dot_product(r[1], l0)));	// riz
					B_[i * 3 + 0] = l0.x();//there was a + so maybe it's supposed to be abs() ? //dot_product(r[0], l0));	// const

					// viy
					triplets.push_back(EigenTriplet(i * 3 + 1, (i + 0) * 3 + 1, -1));					// viy
					triplets.push_back(EigenTriplet(i * 3 + 1, (i + 1) * 3 + 1, 1));
					triplets.push_back(EigenTriplet(i * 3 + 1, n_involved * 3 + (i + 0) * 3 + 0, l0.z()));	//dot_product(r[2], l0)));	// rix
					triplets.push_back(EigenTriplet(i * 3 + 1, n_involved * 3 + (i + 0) * 3 + 1, 0));						// riy
					triplets.push_back(EigenTriplet(i * 3 + 1, n_involved * 3 + (i + 0) * 3 + 2, -l0.x()));	//dot_product(r[0], l0)));	// riz
					B_[i * 3 + 1] = l0.y();//dot_product(r[1], l0));	// const

					// viz
					triplets.push_back(EigenTriplet(i * 3 + 2, (i + 0) * 3 + 2, -1));					// viz
					triplets.push_back(EigenTriplet(i * 3 + 2, (i + 1) * 3 + 2, 1));
					triplets.push_back(EigenTriplet(i * 3 + 2, n_involved * 3 + (i + 0) * 3 + 0, -l0.y()));	//dot_product(r[0], l0)));	// rix
					triplets.push_back(EigenTriplet(i * 3 + 2, n_involved * 3 + (i + 0) * 3 + 1, l0.x()));	//dot_product(r[1], l0)));	// riy
					triplets.push_back(EigenTriplet(i * 3 + 2, n_involved * 3 + (i + 0) * 3 + 2, 0));						// riz
					B_[i * 3 + 2] = l0.z();//dot_product(r[2], l0);	// const
				}
			}

			//  laplacian of rotation matrix
			//  //  L dR*R = 0
			//	L dR = 0;
			//  3*3 all entries

			if (USE_L0_R) {
			}
			else {
				for (GRuint i(0); i < n_laplacians_R; i++) {
					// L rx = 0
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 0, n_involved * 3 + (i + 0) * 3 + 0, W* -0.5f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 0, n_involved * 3 + (i + 1) * 3 + 0, W * 1.f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 0, n_involved * 3 + (i + 2) * 3 + 0, W* -0.5f));
					B_[n_laplacians_P * 3 + i * 3 + 0] = 0;

					// L ry = 0
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 1, n_involved * 3 + (i + 0) * 3 + 1, W* -0.5f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 1, n_involved * 3 + (i + 1) * 3 + 1, W * 1.f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 1, n_involved * 3 + (i + 2) * 3 + 1, W* -0.5f));
					B_[n_laplacians_P * 3 + i * 3 + 1] = 0;

					// L rz = 0
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 2, n_involved * 3 + (i + 0) * 3 + 2, W* -0.5f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 2, n_involved * 3 + (i + 1) * 3 + 2, W * 1.f));
					triplets.push_back(EigenTriplet( n_laplacians_P * 3 + i * 3 + 2, n_involved * 3 + (i + 2) * 3 + 2, W* -0.5f));
					B_[n_laplacians_P * 3 + i * 3 + 2] = 0;
				}
			}

			//  position of fixed vertices
			//  v = v0
			for (GRuint i(0); i < fixed_vertices.size(); i++) {
				GRuint j = fixed_vertices[i];
				triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 0, (j - index_low_involved) * 3 + 0, CONSTRAINT_WEIGHT));	// vix
				triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 1, (j - index_low_involved) * 3 + 1, CONSTRAINT_WEIGHT));	// viy
				triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 2, (j - index_low_involved) * 3 + 2, CONSTRAINT_WEIGHT));	// viz

				B_[n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 0] = CONSTRAINT_WEIGHT * vs[j - index_low_involved].x();
				B_[n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 1] = CONSTRAINT_WEIGHT * vs[j - index_low_involved].y();
				B_[n_laplacians_P * 3 + n_laplacians_R * 3 + 3 * i + 2] = CONSTRAINT_WEIGHT * vs[j - index_low_involved].z();
			}


			//  rotation matrix of end points
			//  R = R0
			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 0, n_involved * 3 + 0, CONSTRAINT_WEIGHT));	// rx0 = 0);
			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 1, n_involved * 3 + 1, CONSTRAINT_WEIGHT));	// ry0 = 0);
			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 2, n_involved * 3 + 2, CONSTRAINT_WEIGHT));	// rz0 = 0);
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 0] = 0;
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 1] = 0;
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 2] = 0;

			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 3, n_involved * 3 + (n_involved - 1) * 3 + 0, CONSTRAINT_WEIGHT));	// rxn-1 = 0);
			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 4, n_involved * 3 + (n_involved - 1) * 3 + 1, CONSTRAINT_WEIGHT));	// ryn-1 = 0);
			triplets.push_back(EigenTriplet( n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 5, n_involved * 3 + (n_involved - 1) * 3 + 2, CONSTRAINT_WEIGHT));	// rzn-1 = 0);
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 3] = 0;
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 4] = 0;
			B_[n_laplacians_P * 3 + n_laplacians_R * 3 + n_fixed * 3 + 5] = 0;


			A_.setFromTriplets(triplets.begin(), triplets.end());

			std::cout << " A: " << std::endl << A_ << std::endl;
			std::cout << " B : " << std::endl << B_ << std::endl;

		}

		void update_B() {

			//  // L v = dR * R * L v0
			//  L v = dR * L v0
			if (USE_L0) {
				for (GRuint i(0); i < n_laplacians_P; i++) {
					EigenMatrix current_R = R_[i];
					EigenVector3 l0 = original_laplacians_[i];
					for (GRuint j(0); j < 3; j++) {
						B_[i * 3 + j] = EigenVector3(current_R.row(0)).dot(l0);
					}
					
				}
			}
			std::cout << "B after update : " << B_ << std::endl;
		}


		void solve() {
			solver_.compute(A_);
			if (solver_.info() != Eigen::Success) {
				std::cerr << " ERROR - CurveDeformer solving failed" << std::endl;
				return;
			}
			vxyz_rxyz_ = solver_.solve(B_);

			std::cout << "vxyz_rxyz : " << vxyz_rxyz_ << std::endl;
		}


		EigenMatrix cross_multiply(GRfloat rx, GRfloat ry, GRfloat rz, EigenMatrix A) {
			EigenMatrix B(3,3);
			B << 1,       -rz / 2, ry / 2, 
				 rz / 2,  1,       -rx / 2,
				 -ry / 2, rx / 2,  1;
			return B*A;
		}

		/*JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
cout << "Its singular values are:" << endl << svd.singularValues() << endl;
cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;*/

		EigenMatrix get_closest_orthonormal(EigenMatrix A) {
			EigenSVD svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

			return svd.matrixU() * svd.matrixV().transpose();
		}


		void update_R() {

			std::cout << "Rs after update : " << std::endl;
			for (GRuint i(0); i < n_involved; i++) {
				GRfloat rx = vxyz_rxyz_[n_involved * 3 + i * 3 + 0];
				GRfloat ry = vxyz_rxyz_[n_involved * 3 + i * 3 + 1];
				GRfloat rz = vxyz_rxyz_[n_involved * 3 + i * 3 + 2];

				R_[i] = cross_multiply(rx, ry, rz, R_[i]);
				R_[i] = get_closest_orthonormal(R_[i]);
				std::cout << R_[i] << std::endl;
			}
		}


		void set_positions() {

			GRfloat max = -std::numeric_limits<GRfloat>::max();
			for (GRuint i(0); i < n_involved; i++) {
				if (!fixed_[i]) {
					GRfloat d = sqrtf(vs[i].x() - vxyz_rxyz_[i + 3 + 0])
						+ sqrtf(vs[i].y() - vxyz_rxyz_[i + 3 + 1])
						+ sqrtf(vs[i].z() - vxyz_rxyz_[i + 3 + 2]);
						
					max = d > max ? d : max;

					vs[i].x() = vxyz_rxyz_[i * 3 + 0];
					vs[i].y() = vxyz_rxyz_[i * 3 + 1];
					vs[i].z() = vxyz_rxyz_[i * 3 + 2];
				}
			}
		}


		void prepare_for_laplacian_L1_adjust() {

		}

		void final_laplacian_L1_adjust() {

		}




	};
};
