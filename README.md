<!--
 * @Author: Zhang-sklda 845603757@qq.com
 * @Date: 2025-10-26 10:22:51
 * @LastEditors: Zhang-sklda 845603757@qq.com
 * @LastEditTime: 2025-10-26 13:23:39
 * @FilePath: /7Dof_Kinematics_iiwa_Rokae/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
你好，这是我的第一个项目，我将在这个项目中，采用c++语言，编写基于kuka-iiwa的SRS构型的七自由度冗余机械臂的

项目架构如下:
7Dof_Kinematics_iiwa_Rokae/ 
├── include/
│   ├── SRSAnalyticalSolver.hpp     ✅（核心逆解算法）
│   ├── KUKAiiwaSolver.hpp          ✅（封装机器人参数+接口）
│   └── Utils.hpp                   ✅（数学工具函数）
├── src/
│   ├── SRSAnalyticalSolver.cpp     ✅（逆运动学具体实现）
│   ├── KUKAiiwaSolver.cpp          ✅（负责正解、参数、接口）
│   └── main.cpp                    ✅（测试）
└── CMakeLists.txt

如何复现：
阶段	模块	            内容
1️⃣	    基础结构	        创建类、定义DH结构、矩阵计算工具（Eigen）
2️⃣	    正运动学FK	        根据DH参数实现FK验证正确性
3️⃣	    肘角求解	        用余弦定理计算肘部角度θ₄
4️⃣	    参考平面与自运动角ψ	  建立SRS几何模型，计算ψ参数化
5️⃣	    解算θ₁–θ₇	        逐步推导肩部和腕部角度
6️⃣	    搜索多解与验证	     枚举GC和ψ，验证与目标位姿匹配