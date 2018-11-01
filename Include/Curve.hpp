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

#include "Vector.hpp"

namespace grapholon {

	class Spline {
	private:
		Vector3f start_pos_;
		Vector3f start_tangent_;
		Vector3f end_pos_;
		Vector3f end_tangent_;

	public:
		Spline(Vector3f start_pos, Vector3f start_tangent, Vector3f end_pos, Vector3f end_tangent)
			: start_pos_(start_pos), start_tangent_(start_tangent), end_pos_(end_pos), end_tangent_(end_tangent) {}
	};

	class Curve {

	};


	typedef std::pair<Vector3f, Vector3f> PointTangent;

	class SplineCurve : public Curve, public std::vector<PointTangent> {
	private:
		/*std::vector<PointTangent> points_and_tangents_; single vector of pairs to ensure we have the same
													   number of points than tangents*/
	public:
		/** Creates a default SplineCurve : a straigt line going from the origin to (1,0,0)*/
		SplineCurve() {
			push_back(PointTangent(Vector3f(0, 0, 0), Vector3f(1, 0, 0)));
			push_back(PointTangent(Vector3f(1, 0, 0), Vector3f(1, 0, 0)));
		}

		SplineCurve(PointTangent start, PointTangent end) {
			push_back(start);
			push_back(end);
		}

		SplineCurve(std::vector<PointTangent> points_and_tangents)
			: std::vector<PointTangent >(points_and_tangents) {
			if (points_and_tangents.size() < 2) {
				throw std::invalid_argument("Cannot create spine curve with less than two points and tangents");
			}
		}

		//updates the tangents based on the points
		void update_tangents() {
			front().second = (*this)[1].first - front().first;
			for (GRuint i(1); i < size()-1; i++) {
				(*this)[i].second *= ((*this)[i + 1].first - (*this)[i - 1].first).norm();
			}
			back().second = back().first - (*this)[size() - 2].first;
		}

		void pseudo_elastic_deform(bool source, Vector3f new_position) {
			GRfloat max_max_displacement(0.1f);
			GRfloat elastic_constant(0.5f);
			//GRfloat time_step(0.01f);
			GRfloat mass(1.f);

			std::cout << " size : " << size() << std::endl;

			//length[i] = length from [i] to [i+1]
			std::vector<GRfloat> original_lengths(size()-1);
			for (GRuint i(0); i < size() - 1;i++) {
				original_lengths[i] = ((*this)[i + 1].first - (*this)[i].first).norm();
				std::cout << "original length " << i << " : " << original_lengths[i] << std::endl;
			}

			if (source) {
				front().first = new_position;
			}
			else {
				back().first = new_position;
			}
			GRuint iteration_count(0);

			GRfloat max_displacement(-std::numeric_limits<GRfloat>::max());
			do {
				/*std::cout << "------------------------" << std::endl;
				std::cout << "	iteration : " << iteration_count << std::endl;*/
				max_displacement = -std::numeric_limits<GRfloat>::max();

				for (GRuint i(1); i < size() - 1; i++) {
					Vector3f left_direction = (*this)[i - 1].first - (*this)[i].first;
					Vector3f left_force = left_direction * (1.f - original_lengths[i-1] / left_direction.norm());

					Vector3f right_direction = (*this)[i + 1].first - (*this)[i].first;
					Vector3f right_force = right_direction * (1.f - original_lengths[i] / right_direction.norm());


					Vector3f displacement = (left_force + right_force)*(elastic_constant/mass);
					GRfloat displacement_norm = displacement.norm();

					(*this)[i].first += displacement;

					/*std::cout << "left direction   : " << left_direction.to_string() << std::endl;
					std::cout << "right direction  : " << right_direction.to_string() << std::endl;
					std::cout << "left force       : " << left_force.to_string()   << std::endl;
					std::cout << "right force      : " << right_force.to_string()  << std::endl;
					std::cout << "displacement     : " << displacement.to_string() << std::endl;
					std::cout << "norm             : " << displacement_norm        << std::endl;*/

					max_displacement = MAX(max_displacement, displacement_norm);
				}
				//std::cout << "max displacement : "<< max_displacement << std::endl;

				iteration_count++;
			} while (max_displacement > max_max_displacement && iteration_count < 10);

			update_tangents();
		}



		void add_middle_point(PointTangent middle_point) {
			PointTangent end = back();
			pop_back();

			push_back(middle_point);

			push_back(end);
		}

		void add_middle_points(std::vector<PointTangent> points_and_tangents) {
			PointTangent end = back();
			pop_back();

			for (auto point_and_tangent : points_and_tangents) {
				push_back(point_and_tangent);
			}

			push_back(end);
		}


		std::string to_string() const {
			std::stringstream msg;
			std::stringstream points_line;
			std::stringstream tangents_line;

			points_line << " points   : ";
			tangents_line << " tangents : ";
			for (auto point_and_tangent : (*this)) {
				points_line << point_and_tangent.first.to_string() << " ";
				tangents_line << point_and_tangent.second.to_string() << " ";
			}
			points_line << std::endl;

			msg << points_line.str() << tangents_line.str();
			return msg.str();
		}
	};


	/** Inherits from std::vector so we can use the same interface on it*/
	class DiscreteCurve : public Curve, public std::vector<Vector3f> {
	private:

	public:

		typedef enum { START_AND_END, CURVE_FITTING, FULL_CURVE, MIDDLE_POINT } CONVERSION_METHOD;

#define MAX_CURVE_FITTING_ITERATIONS 10
#define DEFAULT_MAX_ERROR 0.1f

		DiscreteCurve(std::vector<Vector3f> points) : std::vector<Vector3f>(points) {}

		DiscreteCurve(GRuint size) : std::vector<Vector3f>((size_t)size) {}

		DiscreteCurve(size_t size) : std::vector<Vector3f>(size) {}

		DiscreteCurve() {}


		GRuint nearest_point_index(GRuint start, GRuint end, Vector3f point) {
			if (start > end
				|| start >= size()
				|| end >= size()) {
				return -1;
			}
			GRuint min_index(0);
			GRfloat min(std::numeric_limits<GRfloat>::max());
			for (GRuint i(start); i <= end; i++) {
				GRfloat distance((*this)[i].distance(point));
				if (distance < min) {
					min = distance;
					min_index = i;
				}
			}
			return min_index;
		}

		GRuint furthest_point_to_line_index(GRuint start, GRuint end, Vector3f from, Vector3f to, GRfloat& max_distance) const {
			GRuint max_index(0);
			max_distance = -std::numeric_limits<GRfloat>::max();
			for (GRuint i(start); i <= end; i++) {
				GRfloat distance((*this)[i].distance_to_line(from, to));
				if (distance > max_distance) {
					max_distance = distance;
					max_index = i;
				}
			}
			return max_index;
		}



		void fit_curve_rec(GRuint start,
			GRuint end,
			GRuint current_iteration,
			GRfloat max_error,
			std::vector<PointTangent>& result,
			std::vector<bool>& is_set) {
			if (start <= end
				&& start < size()
				&& end < size()
				&& current_iteration <= MAX_CURVE_FITTING_ITERATIONS) {
				//std::cout << "current iteration : " << current_iteration << std::endl;
				//std::cout << " from " << start << " to " << end << std::endl;
				

				GRfloat error;

				GRuint furthest_point_index(furthest_point_to_line_index(start, end, (*this)[start], (*this)[end], error));

				
				//std::cout<< "furthest_point_index : " << furthest_point_index << std::endl;
				//std::cout << "error : " << error << std::endl;
				if (!is_set[furthest_point_index]) {
					is_set[furthest_point_index] = true;

					Vector3f point = (*this)[furthest_point_index];
					//std::cout << "point : " << point.to_string() << std::endl;

					Vector3f local_tangent;
					if (furthest_point_index > 0 && furthest_point_index < size() - 1) {
						local_tangent = (*this)[furthest_point_index + 1] - (*this)[furthest_point_index - 1];
					}
					//std::cout << "tangent : " << local_tangent.to_string() << std::endl;

					result[furthest_point_index] = PointTangent(point, local_tangent);

					if (error > max_error) {
						fit_curve_rec(start, furthest_point_index, current_iteration + 1, max_error, result, is_set);
						fit_curve_rec(furthest_point_index, end, current_iteration + 1, max_error, result, is_set);
					}
				}
			}
		}

		std::vector<PointTangent> fit_curve(GRfloat max_error = DEFAULT_MAX_ERROR) {

			//to avoid 0.f max error
			if (max_error < FLT_EPSILON) {
				max_error = FLT_EPSILON;
			}

			std::vector<PointTangent> result_rec(size());
			std::vector<bool> is_set(size());
			std::vector<PointTangent> result;

			fit_curve_rec(0, (GRuint)size() - 1, 0, max_error, result_rec, is_set);


			for (GRuint i(0); i < size(); i++) {
				if (is_set[i]) {
					
					result.push_back(result_rec[i]);
					//here we set the norm as the one from the two new points.
					if (i > 0 && i < size() - 1) {
						result_rec[i].second.normalize();
						result_rec[i].second *= (result_rec[i + 1].first - result_rec[i - 1].first).norm();
					}
				}
			}

			return result;
		}




		/** NOTE : allocates a new SplineCurve -> call 'delete' on the return value*/
		SplineCurve* to_spline_curve(CONVERSION_METHOD method, void* extra_parameter = nullptr) {
			if (size() < 2) {
				throw std::invalid_argument("Cannot convert DiscreteCurve with less than two points to SplineCurve. Returning nullptr");
			}
			else if (size() == 2) {
				Vector3f tangent = back() - front();
				return new SplineCurve(PointTangent(front(), tangent), PointTangent(back(), tangent));
			}
			else if (size() == 3) {
				std::vector<PointTangent> points_and_tangents;

				points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));
				points_and_tangents.push_back(PointTangent(
					(*this)[1],
					(back() - front())*0.5f));
				points_and_tangents.push_back(PointTangent(back(), back() - (*this)[1]));
				return new SplineCurve(points_and_tangents);
			}
			else {
				switch (method) {
				case START_AND_END: {

					std::vector<PointTangent> points_and_tangents;

					points_and_tangents.push_back(PointTangent(front(), back() - front()));

					points_and_tangents.push_back(PointTangent(back(), back() - front()));

					return new SplineCurve(points_and_tangents);

					break;
				}
				case MIDDLE_POINT: {

					GRuint middle_point_index = (GRuint)size() / 2;

					std::vector<PointTangent> points_and_tangents;

					points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));
					points_and_tangents.push_back(PointTangent(
						(*this)[middle_point_index],
						((*this)[middle_point_index + 1] - (*this)[middle_point_index - 1])*0.5f));
					points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size() - 2]));
					return new SplineCurve(points_and_tangents);

					break;
				}
				case CURVE_FITTING: {
					std::vector<PointTangent> points_and_tangents;
					points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));

					GRfloat default_error;
					if (extra_parameter == nullptr) {
						extra_parameter = &default_error;
					}

					std::vector<PointTangent> reduced_curve(fit_curve(*((GRfloat*)extra_parameter)));

					/*if (reduced_curve.size() > 1) {
						points_and_tangents.push_back(PointTangent(reduced_curve[0], reduced_curve[1] - front()));
					}


					for (GRuint point_index(1); point_index < reduced_curve.size()-1; point_index++) {
			
						points_and_tangents.push_back(PointTangent(
							reduced_curve[point_index],
							(reduced_curve[point_index + 1] - reduced_curve[point_index - 1])*0.5f));
					}


					points_and_tangents.push_back(PointTangent(reduced_curve.back(), back()- reduced_curve[reduced_curve.size() - 1]));
					*/

					for (auto point_tangent : reduced_curve) {
						points_and_tangents.push_back(point_tangent);
					}
					
					points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size() - 2]));
					return new SplineCurve(points_and_tangents);
				}
				case FULL_CURVE: {

					std::vector<PointTangent> points_and_tangents;

					points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));

					for (GRuint point_index(1); point_index < size() - 1; point_index++) {
						points_and_tangents.push_back(PointTangent(
							(*this)[point_index],
							((*this)[point_index + 1] - (*this)[point_index - 1])*0.5f));
					}

					points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size() - 2]));

					return new SplineCurve(points_and_tangents);

					break;
				}
				default: {
					return to_spline_curve(START_AND_END);
				}
				}
			}
		}

		void smooth_moving_average(GRuint window_width) {
			if (window_width > 2 && window_width < size()) {
				DiscreteCurve smoothed;
				smoothed.push_back(front());

				for (GRuint i(1); i < size() - 1; i++) {
					//std::cout << "smoothing element " << i << std::endl;
					GRuint start((GRuint)(i < window_width / 2 ? 0 : i - window_width / 2));
					GRuint end((GRuint)((i + window_width / 2) >= size() ? size() - 1 : i + window_width / 2));

					//std::cout << "from " << start << " to " << end << std::endl;
					Vector3f local_average(0.f);
					for (GRuint j(start); j <= end; j++) {
						local_average += (*this)[j];
					}
					smoothed.push_back(local_average / (GRfloat)(end - start + 1));
				}
				smoothed.push_back(back());
				(*this) = smoothed;
			}
		}

		std::string to_string() const {
			std::stringstream msg;

			msg << " points   : ";
			for (auto point : (*this)) {
				msg << point.to_string() << " ";
			}

			return msg.str();
		}
	};

};