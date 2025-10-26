/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:31:37
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 21:39:47
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/include/KUKAiiwaSolver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "SRSAnalyticalSolver.h"
#include <algorithm>
class KUKAiiwaSolver : public SRSAnalyticalSolver
{
public:
	KUKAiiwaSolver();

	Matrix4d forwardKinematics(const VectorXd& qpos) override;

	// =======================
	// 阶段3：肘点位置计算（包含ψ）
	// =======================
	Vector3d computeElbowPosition(const Vector3d& P_s,const Vector3d& P_w,double L1,double L2,double psi,bool elbow_up);

	bool inverseKinematics(const Matrix4d& targetPose, const VectorXd& jointSeed, VectorXd& qSolution, double psi, int rconf) override;
	
private:
	std::vector<DHParam> dh_params;
	Vector4d link_lengths;
};
