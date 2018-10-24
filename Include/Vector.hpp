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

template<typename _TYPE, GRuint _SIZE>
class Vector {
	static_assert(_SIZE > 0, "Cannot create zero-sized Vector");

private:

protected:
	_TYPE data_[_SIZE];

public:

	Vector() {
		std::memset(data_, 0, _SIZE * sizeof(_TYPE));
	}


	//math operators


	/*Vector<_TYPE, _SIZE> operator+(const Vector<_TYPE, _SIZE>& other) {

		for (GRuint i(0); i < _SIZE; i++) {

		}
	}*/


	//string stuff
	std::string to_string() const {
		std::stringstream msg;
		msg << "(";
		for (GRuint i(0); i < _SIZE-1; i++) {
			msg << data_[i] << " ";
		}
		msg << data_[_SIZE - 1] << ")";
		return msg.str();
	}
};

template<typename _TYPE>
class Vector3 : public Vector<_TYPE, 3> {
private:
	///<this allows for more compact representation
	typedef Vector3<_TYPE> _Vector3;
public:
	Vector3(_TYPE x, _TYPE y, _TYPE z) {
		X = x;
		Y = y;
		Z = z;
	}
	Vector3() : Vector() {}

	/**Copy constructor to avoid the side effect of copying the member reference X,Y & Z*/
	Vector3(const Vector3& other) {
		X = other.X;
		Y = other.Y;
		Z = other.Z;
	}

	_TYPE& X = data_[0];
	_TYPE& Y = data_[1];
	_TYPE& Z = data_[2];


	//math operators
	void operator=(const _Vector3& other) {
		X = other.X;
		Y = other.Y;
		Z = other.Z;
	}

	_Vector3 operator+(const _Vector3& other) const {
		return _Vector3(X + other.X, Y + other.Y, Z + other.Z);
	}

	_Vector3 operator-(const _Vector3& other) const {
		return _Vector3(X - other.X, Y - other.Y, Z - other.Z);
	}

	_Vector3 operator*(const _TYPE& scalar) const {
		return _Vector3(X*scalar, Y*scalar, Z*scalar);
	}

	_Vector3 operator/(const _TYPE& scalar) const {
		return _Vector3(X/scalar, Y/scalar, Z/scalar);
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
		(*this) = (*this)/scalar;
		return *this;
	}

	_TYPE dot(const _Vector3& other) const {
		return X*other.X + Y*other.Y + Z*other.Z;
	}

	GRfloat norm() const {
		return sqrtf((GRfloat)(this->dot(*this)));
	}

	Vector3<GRfloat> normalize() {
		return (*this) / norm();
	}

};

typedef Vector3<GRfloat> Vector3f;
typedef Vector3<GRint> Vector3d;
typedef Vector3<GRuint> Vector3u;


