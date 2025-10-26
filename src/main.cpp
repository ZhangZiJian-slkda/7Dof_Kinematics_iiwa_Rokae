/*
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:29:51
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 12:12:40
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include "KUKAiiwaSolver.h"

using namespace std;

void testZeroPositionFK()
{
	KUKAiiwaSolver solver;
	std::cout << "KUKA iiwa Solver initialized!" << std::endl;
	VectorXd q(7);
	q << 0, M_PI / 4, -M_PI / 4, M_PI / 2, -M_PI / 4, M_PI / 3, 0;
	Matrix4d T = solver.forwardKinematics(q);
	cout << "FK at Zero Position:\n"
		<< T << "\n\n";
	cout << "Position (x,y,z): " << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3) << endl;
}

int main()
{
	testZeroPositionFK();

	return 0;
}
