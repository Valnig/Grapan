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

class SplineCurve : Curve {
private:
	std::vector<PointTangent> points_and_tangents_;/** single vector of pairs to ensure we have the same
												   number of points than tangents*/
public:
	SplineCurve(PointTangent start, PointTangent end) {
		points_and_tangents_.push_back(start);
		points_and_tangents_.push_back(end);
	}

	SplineCurve(std::vector<PointTangent> points_and_tangents){
		if (points_and_tangents.size() < 2) {
			throw std::invalid_argument("Cannot create spine curve with less than two points and tangents");
		}
		points_and_tangents_ = points_and_tangents;
	}

	void add_middle_point(PointTangent middle_point) {
		PointTangent end = points_and_tangents_.back();
		points_and_tangents_.pop_back();

		points_and_tangents_.push_back(middle_point);

		points_and_tangents_.push_back(end);
	}

	void add_middle_points(std::vector<PointTangent> points_and_tangents) {
		PointTangent end = points_and_tangents_.back();
		points_and_tangents_.pop_back();

		for (auto point_and_tangent : points_and_tangents) {
			points_and_tangents_.push_back(point_and_tangent);
		}

		points_and_tangents_.push_back(end);
	}


	std::string to_string() const {
		std::stringstream msg;
		std::stringstream points_line;
		std::stringstream tangents_line;

		points_line << " points   : ";
		tangents_line << " tangents : ";
		for (auto point_and_tangent : points_and_tangents_) {
			points_line << point_and_tangent.first.to_string() << " ";
			tangents_line << point_and_tangent.second.to_string() << " ";
		}
		points_line << std::endl;

		msg << points_line.str() << tangents_line.str();
		return msg.str();
	}

	/*bool add_middle_points_starting_from_index(std::vector<PointTangent> points_and_tangents, GRuint start_index) {
	if (start_index >= points_and_tangents_.size) {
	std::cerr << "Cannot add middle points starting from " << start_index << " in curve with " << points_and_tangents_.size() << " points" << std::endl;
	return false;
	}

	std::vector<PointTangent> end_points;
	for (GRuint i(start_index); i < points_and_tangents_.size(); i++) {
	end_points.push_back(points_and_tangents_[i]);

	}
	for (GRuint i(start_index); i < MIN(points_and_tangents_.size(), start_index + points_and_tangents.size()); i++) {
	points_and_tangents_[i] = points_and_tangents[i - start_index];
	}

	return true;
	}
	*/
};


/** Inherits from std::vector so we can use the same interface on it*/
class DiscreteCurve : public Curve, public std::vector<Vector3f> {
private:

public:
	DiscreteCurve(std::vector<Vector3f> points) : std::vector<Vector3f>(points) {}

	DiscreteCurve() {}

	typedef enum {SINGLE_POINT, FULL_CURVE, HIGH_CURVATURE} CONVERSION_METHOD;

	/** NOTE : allocates a new SplineCurve -> call 'delete' on the return value*/
	SplineCurve* to_spline_curve(CONVERSION_METHOD method) {
		if (size() < 2) {
			throw invalid_argument("Cannot convert DiscreteCurve with less than two points to SplineCurve. Returning nullptr");
		}else{
			switch (method) {
			case SINGLE_POINT: {
				if (size() == 2) {
					Vector3f tangent = back() - front();
					return new SplineCurve(PointTangent(front(), tangent), PointTangent(back(), tangent));
				}
				else {
					GRuint middle_point_index = ((GRuint)size() + 1) / 2;
					std::vector<PointTangent> points_and_tangents;
					points_and_tangents.push_back(PointTangent(front(), (*this)[1] - front()));
					points_and_tangents.push_back(PointTangent(
						(*this)[middle_point_index], 
						((*this)[middle_point_index + 1] - (*this)[middle_point_index - 1])*0.5f));
					points_and_tangents.push_back(PointTangent(back(), back() - (*this)[size()-2]));
					return new SplineCurve(points_and_tangents);
				}
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
			case HIGH_CURVATURE: {
				return nullptr;
				//TODO
				break;
			}
			default: {
				return to_spline_curve(SINGLE_POINT);
			}
			}
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

