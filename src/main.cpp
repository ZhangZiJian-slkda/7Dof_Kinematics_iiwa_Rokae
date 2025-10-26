/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:29:51
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 23:46:45
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <iomanip>
#include "KUKAiiwaSolver.h"

using namespace std;

void testZeroPositionFK()
{
	KUKAiiwaSolver solver;
	std::cout << "KUKA iiwa Solver initialized!" << std::endl;
	VectorXd q(7);
	q << 0, M_PI / 4, -M_PI / 4, M_PI / 2, -M_PI / 4, M_PI / 3, 0;
	Matrix4d T = solver.forwardKinematics(q);
	cout << fixed << std::setprecision(4);
	cout << "===== FK Test =====\n";
	cout << "Joint angles (rad): " << q.transpose() << endl;
	cout << "End-effector pose:\n"
		 << T << endl;
	cout << "Position (x,y,z): " << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3) << "\n\n";
	cout << "Position (x,y,z): " << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3) << endl;
}

void testElbowPosition()
{
	KUKAiiwaSolver solver;
	Vector3d P_s(0, 0, 0.34);
	Vector3d P_w(0.5, 0.1, 0.4);
	double L1 = 0.4, L2 = 0.4;

	cout << "===== Elbow ψ Sweep Test =====\n";
	for (double psi = 0; psi <= 2 * M_PI; psi += M_PI / 6)
	{
		Vector3d P_e = solver.computeElbowPosition(P_s, P_w, L1, L2, psi, true);
		cout << "ψ = " << std::setw(5) << psi << " rad -> Elbow: "
			 << P_e.transpose() << endl;
	}
	cout << endl;
}

void testIKWithElbowAngle()
{
	KUKAiiwaSolver solver;
	Matrix4d targetPose = Matrix4d::Identity();
	targetPose(0, 3) = 0.5; // x
	targetPose(1, 3) = 0.1; // y
	targetPose(2, 3) = 0.4; // z

	VectorXd jointSeed(7);
	jointSeed << 0, 0, 0, 0, 0, 0, 0;

	VectorXd qSolution(7);
	double psi = M_PI / 4; // Example elbow angle
	int rconf = 1;		   // Example robot configuration

	bool success = solver.inverseKinematics(targetPose, jointSeed, qSolution, psi, rconf);
	if (success)
	{
		cout << "IK Solution with ψ=" << psi << ":\n"
			 << qSolution.transpose() << endl;
	}
	else
	{
		cout << "IK failed for ψ=" << psi << endl;
	}
}

double computeRotationError(const Matrix3d &R_desired, const Matrix3d &R_actual)
{
	double val = (R_desired.transpose() * R_actual).trace();
	val = std::min(3.0, std::max(-1.0, val));
	return acos((val - 1.0) / 2.0);
}
void testIKConsistency()
{
	KUKAiiwaSolver solver;
	cout << "===== FK/IK Consistency Test =====\n";

	for (int i = 0; i < 5; ++i)
	{
		VectorXd q_rand = VectorXd::Random(7) * (M_PI / 3);
		Matrix4d T_goal = solver.forwardKinematics(q_rand);

		VectorXd q_sol(7);
		double psi = M_PI / 6;
		int rconf = 1;

		bool ok = solver.inverseKinematics(T_goal, q_rand, q_sol, psi, rconf);
		Matrix4d T_check = solver.forwardKinematics(q_sol);

		Vector3d pos_err = T_goal.block<3, 1>(0, 3) - T_check.block<3, 1>(0, 3);
		double pos_error = pos_err.norm();
		double ori_error = computeRotationError(T_goal.block<3, 3>(0, 0), T_check.block<3, 3>(0, 0));

		cout << "Test " << i + 1 << ":\n";
		cout << "q_goal = " << q_rand.transpose() << endl;
		cout << "q_sol  = " << q_sol.transpose() << endl;
		cout << "Position error = " << pos_error << " m\n";
		cout << "Orientation error = " << ori_error << " rad\n\n";
	}
}

int main()
{
	// KUKAiiwaSolver solver;

	testZeroPositionFK();

	testElbowPosition();

	testIKConsistency();

	return 0;
}
