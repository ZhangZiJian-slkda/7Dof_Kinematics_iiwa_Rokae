/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:30:46
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 10:30:52
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/src/KUKAiiwaSolver.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "KUKAiiwaSolver.h"

using namespace utils;

KUKAiiwaSolver::KUKAiiwaSolver()
{
	double half_pi = M_PI / 2.0;
	link_lengths = Vector4d(0.34, 0.4, 0.4, 0.126);

	dh_params = {
		{link_lengths[0], -half_pi, 0, 0},
		{0,  half_pi, 0, 0},
		{link_lengths[1],  half_pi, 0, 0},
		{0, -half_pi, 0, 0},
		{link_lengths[2], -half_pi, 0, 0},
		{0,  half_pi, 0, 0},
		{link_lengths[3], 0, 0, 0}
	};
}

Matrix4d KUKAiiwaSolver::forwardKinematics(const VectorXd& qpos)
{
	if (qpos.size() != 7)
	{
		std::cerr << "[Error] FK requires 7 joint angles!" << std::endl;
		return Matrix4d::Identity();
	}

	Matrix4d T = Matrix4d::Identity();

	for (int i = 0; i < 7; i++)
	{
		Matrix4d A = DH(dh_params[i].d,
			dh_params[i].alpha,
			dh_params[i].a,
			dh_params[i].theta + qpos(i)
		);
		T = T * A;
	}
	return T;
}

bool KUKAiiwaSolver::inverseKinematics(const Matrix4d& targetPose, const VectorXd& jointSeed, VectorXd& qSolution, double psi, int rconf)
{
	return false;
}