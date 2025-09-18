#ifndef _XYZ_DATA_H
#define _XYZ_DATA_H

#include <Arduino.h>

template <typename T>
struct xyz_data {
	T x, y, z;

	xyz_data<T>() : x(0), y(0), z(0) {}

	xyz_data<T>(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
	
	xyz_data<T> operator=(xyz_data<T> _a) {
		x = _a.x;
		y = _a.y;
		z = _a.z;

		return *this;
	}

	template <typename T2>
	xyz_data<T> operator*(T2 _a) {
		return xyz_data<T>(x * _a, y * _a, z * _a);
	}

	template <typename T2>
	xyz_data<T> operator/(T2 _a) {
		return xyz_data<T>(x / _a, y / _a, z / _a);
	}
};

#endif // _XYZ_DATA_H
