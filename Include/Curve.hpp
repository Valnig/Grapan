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



class Spline{
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
		: std::vector<PointTangent >(points_and_tangents){
		if (points_and_tangents.size() < 2) {
			throw std::invalid_argument("Cannot create spine curve with less than two points and tangents");
		}
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
	DiscreteCurve(std::vector<Vector3f> points) : std::vector<Vector3f>(points) {}

	DiscreteCurve() {}

	typedef enum { MIDDLE_POINT, FULL_CURVE, START_AND_END, LOCAL_CURVATURE_EXTREMA} CONVERSION_METHOD;


	/** NOTE : allocates a new SplineCurve -> call 'delete' on the return value*/
	SplineCurve* to_spline_curve(CONVERSION_METHOD method) {
		if (size() < 2) {
			throw std::invalid_argument("Cannot convert DiscreteCurve with less than two points to SplineCurve. Returning nullptr");
		}else if (size() == 2) {
			std::cout << "size is 2" << std::endl;
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
			case MIDDLE_POINT: {
				
				GRuint middle_point_index = (GRuint)size() / 2;

				std::vector<PointTangent> points_and_tangents;

				points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));
				points_and_tangents.push_back(PointTangent(
					(*this)[middle_point_index], 
					((*this)[middle_point_index + 1] - (*this)[middle_point_index - 1])*0.5f));
				points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size()-2]));
				return new SplineCurve(points_and_tangents);
				
				break;
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
			case START_AND_END: {

				std::vector<PointTangent> points_and_tangents;

				points_and_tangents.push_back(PointTangent(front(), back() - front()));

				points_and_tangents.push_back(PointTangent(back(), back() - front()));

				return new SplineCurve(points_and_tangents);

				break;
			}
			case LOCAL_CURVATURE_EXTREMA: {
				//first, compute the angle at each point
				std::vector<GRfloat> angles;
				
				//we add a first element to have the angles and positions aligned
				angles.push_back(0.f);

				//note : we only care about the middle points so that's why we start from i=1
				for (GRuint i(1); i < size() - 1; i++) {
					//TODO : check if same result with cos_theta directly
					GRfloat cos_theta 
						= ((*this)[i + 1].dot((*this)[i])) 
						/ ((*this)[i + 1].norm() * (*this)[i].norm());
					angles.push_back(acos(cos_theta));
					if(i>1){
						angles[i] = fabs(angles[i] - angles[i - 1]);
					}
				}
				//we copy the first and last angles so ensure that they won't be considered as extrema
				angles[0] = 0.f;
				angles.push_back(0.f);

				std::vector<PointTangent> points_and_tangents;
				points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));

				//then add the positions where there are local curvature extrema
				for (GRuint i(1); i < size() - 1; i++) {
					if ((angles[i - 1] < angles[i] && angles[i + 1] < angles[i])
						|| (angles[i - 1] > angles[i] && angles[i + 1] > angles[i])) {
						points_and_tangents.push_back(PointTangent((*this)[i], (*this)[i + 1] - (*this)[i - 1]));
					}
				}
				points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size() - 2]));

				return new SplineCurve(points_and_tangents);
				break;
			}
			default: {
				return to_spline_curve(MIDDLE_POINT);
			}
			}
		}
	}

	void smooth_moving_average(GRuint window_width) {
		if (window_width > 2 && window_width < size()) {
			DiscreteCurve smoothed;
			smoothed.push_back(front());

			for (GRuint i(1); i < size() - 1; i++) {
				std::cout << "smoothing element " << i << std::endl;
				GRuint start((GRuint)(i < window_width / 2 ? 0 : i - window_width / 2));
				GRuint end((GRuint)((i + window_width / 2) >= size() ? size()-1 : i + window_width / 2));

				std::cout << "from " << start << " to " << end << std::endl;
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
			msg << point.to_string()<<" ";
		}

		return msg.str();
	}
};

