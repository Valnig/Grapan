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

#include "GrapholonTypes.hpp"
#include "common.hpp"

namespace grapholon {
	template<typename _TYPE, GRuint _SIZE>
	class Vector {
		static_assert(_SIZE > 0, "Cannot create zero-sized Vector");

	private:

	protected:
		_TYPE data_[_SIZE];

		int data2;

	public:

		Vector() {
			std::memset(data_, 0, _SIZE * sizeof(_TYPE));
		}


		//string stuff
		std::string to_string() const {
			std::stringstream msg;
			msg << "(";
			for (GRuint i(0); i < _SIZE - 1; i++) {
				msg << data_[i] << ", ";
			}
			msg << data_[_SIZE - 1] << ")";
			return msg.str();
		}
	};


	template<typename _TYPE>
	class Vector3 : public Vector<_TYPE, 3>{
	private:
		///<this allows for more compact representation
		typedef Vector3<_TYPE> _Vector3;
	public:



		Vector3() : Vector<_TYPE, 3>() {}

		Vector3(_TYPE x, _TYPE y, _TYPE z) {
			X() = x;
			Y() = y;
			Z() = z;
		}

		Vector3(_TYPE value) : Vector3(value, value, value) {}

		/**Copy constructor to avoid the side effect of copying the member reference X(),Y()& Z()*/
		Vector3(const Vector3& other) {
			X() = other.X();
			Y()= other.Y();
			Z() = other.Z();
		}


		_TYPE& X() {
			return Vector<_TYPE, 3>::data_[0];
		}

		_TYPE& Y() {
			return Vector<_TYPE, 3>::data_[1];
		}

		_TYPE& Z() {
			return Vector<_TYPE, 3>::data_[2];
		}

		const _TYPE& X() const {
			return Vector<_TYPE, 3>::data_[0];
		}

		const _TYPE& Y() const {
			return Vector<_TYPE, 3>::data_[1];
		}

		const _TYPE& Z() const {
			return Vector<_TYPE, 3>::data_[2];
		}

		//math operators
		void operator=(const _Vector3& other) {
			X() = other.X();
			Y() = other.Y();
			Z() = other.Z();
		}

		_Vector3 operator+(const _Vector3& other) const {
			return _Vector3(X() + other.X(), Y()+ other.Y(), Z() + other.Z());
		}

		_Vector3 operator-(const _Vector3& other) const {
			return _Vector3(X() - other.X(), Y()- other.Y(), Z() - other.Z());
		}

		_Vector3 operator*(const _TYPE& scalar) const {
			return _Vector3(X()*scalar, Y()*scalar, Z()*scalar);
		}

		_Vector3 operator/(const _TYPE& scalar) const {
			return _Vector3(X() / scalar, Y()/ scalar, Z() / scalar);
		}

		_Vector3 operator+=(const _Vector3& other) {
			(*this) = (*this) + other;
			return *this;
		}

		_Vector3 operator-=(const _Vector3& other) {
			(*this) = (*this) - other;
			return *this;
		}

		_Vector3 operator*=(const _TYPE& scalar) {
			(*this) = (*this)*scalar;
			return *this;
		}

		_Vector3 operator/=(const _TYPE& scalar) {
			(*this) = (*this) / scalar;
			return *this;
		}

		_TYPE dot(const _Vector3& other) const {
			return X()*other.X() + Y()*other.Y()+ Z()*other.Z();
		}

		_Vector3 cross(const _Vector3& other) const {
			return _Vector3(
				Y()*other.Z() - Z()*other.Y(),
				Z()*other.X() - X()*other.Z(),
				X()*other.Y()- Y()*other.X());
		}

		GRfloat norm() const {
			return sqrtf((GRfloat)(this->dot(*this)));
		}

		Vector3<GRfloat> normalize() {
			GRfloat _norm(norm());
			return _norm > FLT_EPSILON ? (*this) /= norm() : (*this);
		}

		Vector3<GRfloat> normalized() {
			GRfloat _norm(norm());
			return _norm > FLT_EPSILON ? (*this) / norm() : (*this);
		}

		static _Vector3 axis_vector(AXIS axis) {
			return _Vector3(axis == X_AXIS, axis == Y_AXIS, axis == Z_AXIS);
		}

		GRfloat distance(const _Vector3& other) const {
			return ((*this) - other).norm();
		}

		GRfloat distance_to_line(const Vector3<GRfloat>& from, const Vector3<GRfloat>& to) const {
			return fabs(((*this) - from).cross((*this) - to).norm() / (to - from).norm());
		}

		GRfloat angular_distance(const Vector3<GRfloat>& other)const {
			GRfloat this_norm = norm();
			GRfloat other_norm = other.norm();
			if (fabs(this_norm) < FLT_EPSILON || fabs(other_norm) < FLT_EPSILON) {
				return 0.f;
			}
			return acosf(MAX(MIN(this->dot(other) / (this_norm * other_norm), 1.f), -1.f));
		}


		//uncomment and test if it becomes useful
		/*_Vector3 move_from_axis(const _Vector3& other) {
			if ((this->normalized() - other->normalized()).norm() <= FLT_EPSILON) {
				(*this) += axis_vector(Z_AXIS))*2.f*FLT_EPSILON;
				move_from_axis(axis_vector(Z_AXIS));
			}
			return (*this);
		}*/

		/** If this vector is perfectly aligned with an axis we add a small ammount
		of another axis.
		NOTE : modification is done in-place*/
		_Vector3 move_from_axis(AXIS axis) {
			if ((this->normalized() - axis_vector(axis)).norm() <= FLT_EPSILON
				|| (this->normalized() + axis_vector(axis)).norm() <= FLT_EPSILON) {
				(*this) += axis_vector((AXIS)((axis + 1) % 3))*128.f*FLT_EPSILON;
			}
			return (*this);
		}

	};

	typedef Vector3<GRfloat> Vector3f;
	typedef Vector3<GRint> Vector3d;
	typedef Vector3<GRuint> Vector3u;

}