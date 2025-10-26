/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:31:54
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 13:23:01
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/include/SRSanalyticalSolver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "Utils.h"
#include <vector>
class SRSAnalyticalSolver
{
public:
	struct DHParam
	{
		double d, alpha, a, theta;
	};

	SRSAnalyticalSolver()
	{
		world_to_base = Matrix4d::Identity();
		flange_to_ee = Matrix4d::Identity();
		dof = 7;
	}

	virtual ~SRSAnalyticalSolver() = default;

	virtual Matrix4d forwardKinematics(const VectorXd &qpos) = 0;

	virtual bool inverseKinematics(const Matrix4d &targetPose, const VectorXd &jointSeed, VectorXd &qSolution, double psi, int rconf) = 0;

private:
	Matrix4d world_to_base;
	Matrix4d flange_to_ee;
	int dof;
};
