/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:31:19
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 10:31:25
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/include/Utils.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#define M_PI       3.14159265358979323846
using namespace Eigen;

namespace utils {
	inline Matrix4d DH(double d, double alpha, double a, double theta) {
		Matrix4d T;
		T << cos(theta), -sin(theta) * cos(alpha), sin(theta)* sin(alpha), a* cos(theta),
			sin(theta), cos(theta)* cos(alpha), -cos(theta) * sin(alpha), a* sin(theta),
			0, sin(alpha), cos(alpha), d,
			0, 0, 0, 1;
		return T;
	}

	inline Matrix3d skew(const Vector3d& v) {
		Matrix3d S;
		S << 0, -v.z(), v.y(),
			v.z(), 0, -v.x(),
			-v.y(), v.x(), 0;
		return S;
	}

	inline double safeCos(double v) {
		if (v > 1.0) return 1.0;
		if (v < -1.0) return -1.0;
		return v;
	}
}