/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:30:46
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 23:32:49
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
		{0, half_pi, 0, 0},
		{link_lengths[1], half_pi, 0, 0},
		{0, -half_pi, 0, 0},
		{link_lengths[2], -half_pi, 0, 0},
		{0, half_pi, 0, 0},
		{link_lengths[3], 0, 0, 0}};
}

Matrix4d KUKAiiwaSolver::forwardKinematics(const VectorXd &qpos)
{
	if (qpos.size() != 7)
	{
		std::cerr << "[Error] FK requires 7 joint angles!" << std::endl;
		return Matrix4d::Identity();
	}

	Matrix4d T = Matrix4d::Identity();

	for (int i = 0; i < 7; ++i)
	{
		Matrix4d A = DH(
			dh_params[i].d,
			dh_params[i].alpha,
			dh_params[i].a,
			qpos(i) + dh_params[i].theta // θ = q_i + offset
		);
		T = T * A;
	}
	return T;
}

Vector3d KUKAiiwaSolver::computeElbowPosition(const Vector3d &P_s, const Vector3d &P_w, double L1, double L2, double psi, bool elbow_up)
{
	Vector3d SW = P_w - P_s;
	double d = SW.norm();

	if (d > (L1 + L2))
	{
		cerr << "[IK Error] Target is out of reach!" << endl;
		return Vector3d::Zero();
	}

	Vector3d e_sw = SW.normalized();

	// 构造参考平面基向量
	Vector3d z_ref(0, 0, 1);
	Vector3d n = e_sw.cross(z_ref);
	if (n.norm() < 1e-6)
	{
		n = Vector3d(1, 0, 0);
	}
	n.normalize();
	Vector3d e_prep = n.cross(e_sw).normalized();
	// 求解几何量
	double a = (L1 * L1 - L2 * L2 + d * d) / (2 * d);
	double h = sqrt(max(0.0, L1 * L1 - a * a));

	double dir = elbow_up ? 1.0 : -1.0;

	// ψ 控制绕 SW 的旋转
	// Vector3d P_e = P_s + a * e_sw + dir * h * (cos(psi) * e_prep + sind(psi) * n);
	Eigen::AngleAxisd rot_psi(psi, e_sw);
	Vector3d offset = dir * h * (rot_psi * e_prep);
	Vector3d P_e = P_s + a * e_sw + offset;

	return P_e;
}

// bool KUKAiiwaSolver::inverseKinematics(const Matrix4d& targetPose, const VectorXd& jointSeed, VectorXd& qSolution, double psi, int rconf)
// {
// 	if (qSolution.size() != 7)
// 	{
// 		qSolution.resize(7);
// 	}
// 	// 提取位置和旋转
// 	Vector3d P_w = targetPose.block<3,1>(0,3);
// 	Matrix3d R_07 = targetPose.block<3,3>(0,0);
// 	// 手腕中心（移除法兰偏移）
// 	Vector3d P_wrist = P_w - link_lengths[3] * R_07.col(2);
// 	// 肩部位置（基本偏移）
// 	Vector3d P_s(0,0,link_lengths[0]);
// 	// 根据ψ计算肘部位置
// 	bool elbow_up = (rconf >= 0);
// 	Vector3d P_e = computeElbowPosition(P_s, P_wrist, link_lengths[1], link_lengths[2], psi, elbow_up);
// 	// ==============================
//     // Step 1: Shoulder joint θ1, θ2, θ3
//     // ==============================
// 	Vector3d v1 = P_e - P_s;
// 	Vector3d v2 = P_wrist - P_e;
// 	double theta1 = atan2(v1.y(), v1.x());
// 	double r = sqrt(v1.x()*v1.x() + v1.y()*v1.y());
// 	double s = v1.z();
// 	double L1 = link_lengths[1], L2 = link_lengths[2];
// 	double D = (r*r + s*s - L1*L1 - L2*L2) / (2 * L1 * L2);

// 	D = std::clamp(D,-1.0, 1.0);
// 	double theta3 = atan2(elbow_up ? sqrt(1 - D*D) : -sqrt(1 - D*D), D);
// 	double theta2 = atan2(s, r) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));

// 	// ==============================
//     // Step 2: Wrist joints θ4–θ7
//     // ==============================
//     // Compute rotation up to wrist
// 	Matrix4d T_03 = Matrix4d::Identity();
// 	VectorXd q_temp(7);
// 	q_temp << theta1, theta2, theta3, 0, 0, 0, 0;

// 	T_03 = forwardKinematics(q_temp);

// 	Matrix3d R_03 = T_03.block<3,3>(0,0);
// 	Matrix3d R_36 = R_03.transpose() * R_07;

// 	double theta4 = atan2(R_36(2,1), R_36(2,2));
// 	double theta5 = atan2(sqrt(R_36(2,1)*R_36(2,1) + R_36(2,2)*R_36(2,2)), R_36(2,0));
// 	double theta6 = atan2(R_36(1,0), -R_36(0,0));
// 	double theta7 = 0.0; // End-effector rotation about its own axis (not defined here)
// 	// ==============================
//     // Step 3: Return solution
//     // ==============================
//     qSolution << theta1, theta2, theta3, theta4, theta5, theta6, theta7;

// 	Matrix4d T_fk = forwardKinematics(qSolution);
// 	Vector3d dp = T_fk.block<3,1>(0,3) - targetPose.block<3,1>(0,3);

// 	return true;
// }

bool KUKAiiwaSolver::inverseKinematics(const Matrix4d &targetPose,
									   const VectorXd &jointSeed,
									   VectorXd &qSolution,
									   double psi,
									   int rconf)
{
	if (qSolution.size() != 7)
		qSolution.resize(7);

	// 提取目标末端位置与姿态
	Vector3d P_target = targetPose.block<3, 1>(0, 3);
	Matrix3d R_target = targetPose.block<3, 3>(0, 0);

	// ==============================
	// Step 1: Wrist center calculation
	// ==============================
	// 移除法兰偏移，得到 wrist 中心
	Vector3d P_wrist = P_target - link_lengths[3] * R_target.col(2); // z-axis offset
	Vector3d P_s(0, 0, link_lengths[0]);							 // shoulder base height

	// ==============================
	// Step 2: Elbow position with ψ
	// ==============================
	bool elbow_up = (rconf >= 0);
	Vector3d P_e = computeElbowPosition(P_s, P_wrist, link_lengths[1], link_lengths[2], psi, elbow_up);

	// ==============================
	// Step 3: Shoulder joint θ₁–θ₃
	// ==============================
	Vector3d v1 = P_e - P_s;
	double r = sqrt(v1.x() * v1.x() + v1.y() * v1.y());
	double s = v1.z();
	double L1 = link_lengths[1], L2 = link_lengths[2];

	double theta1 = atan2(v1.y(), v1.x());

	double D = (r * r + s * s - L1 * L1 - L2 * L2) / (2 * L1 * L2);
	D = std::clamp(D, -1.0, 1.0);
	double theta3 = atan2(elbow_up ? sqrt(1 - D * D) : -sqrt(1 - D * D), D);
	double theta2 = atan2(s, r) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));

	// ==============================
	// Step 4: Wrist joint θ₄–θ₇
	// ==============================
	// 计算 R₀₃
	VectorXd q_temp(7);
	q_temp << theta1, theta2, theta3, 0, 0, 0, 0;
	Matrix4d T_03 = Matrix4d::Identity();

	for (int i = 0; i < 3; ++i)
	{
		Matrix4d A = DH(dh_params[i].d, dh_params[i].alpha, dh_params[i].a, q_temp(i));
		T_03 *= A;
	}

	Matrix3d R_03 = T_03.block<3, 3>(0, 0);
	Matrix3d R_36 = R_03.transpose() * R_target; // wrist rotation

	// Z-Y-Z Euler 分解（适配 iiwa 构型）
	double theta5 = acos(std::clamp(R_36(2, 2), -1.0, 1.0));
	double theta4, theta6;

	if (fabs(sin(theta5)) < 1e-6)
	{
		// 奇异位姿（θ₅ ≈ 0）
		theta4 = atan2(R_36(1, 0), R_36(0, 0));
		theta6 = 0;
	}
	else
	{
		theta4 = atan2(R_36(1, 2), R_36(0, 2));
		theta6 = atan2(R_36(2, 1), -R_36(2, 0));
	}

	// θ₇：末端绕自身Z轴旋转（冗余）
	double theta7 = atan2(R_target(1, 0), R_target(0, 0)) - (theta1 + theta4);

	// ==============================
	// Step 5: 输出解并验证
	// ==============================
	qSolution << theta1, theta2, theta3, theta4, theta5, theta6, theta7;

	Matrix4d T_check = forwardKinematics(qSolution);
	Vector3d pos_err = T_check.block<3, 1>(0, 3) - P_target;
	double pos_error_norm = pos_err.norm();

	if (pos_error_norm > 1e-3)
	{
		std::cerr << "[IK Warning] Position error = " << pos_error_norm << " m\n";
	}

	return (pos_error_norm < 1e-2); // 简单容差
}
