#include <iostream>
#include <vector>
#include <iomanip>
#include <controller/dynamics.hpp>
#include <math.h>
int main(int argc, char** argv) {
    // 1. 设置工作空间里的 URDF 路径 
    // 根据用户提供的双臂模型文件名称
    std::string urdf_path = "/home/xiaokai/OpenArm/openarm_bimanual.urdf"; 
    
    // 根据 openarm_bimanual.urdf，我们计算左臂的动力学
    // 根连杆(base): "openarm_left_link0" 
    // 末端连杆(end): "openarm_left_link7" (如果是7自由度测试)
    std::string base_link = "openarm_left_link0";   
    std::string end_link = "openarm_left_link7";    

    std::cout << "[INFO] 开始初始化由于 KDL 动力学求解器..." << std::endl;
    std::cout << "[INFO] 读取 URDF: " << urdf_path << std::endl;
    std::cout << "[INFO] 运动链: " << base_link << " -> " << end_link << std::endl;

    Dynamics dyn(urdf_path, base_link, end_link);

    if (!dyn.Init()) {
        std::cerr << "[ERROR] 动力学模型初始化失败！" << std::endl;
        return -1;
    }
    std::cout << "\n[INFO] 初始化成功！" << std::endl;

    // 根据 URDF 解析到的链路，左臂本身是 7 自由度机械臂
    const int dof = 7; 
    
    // 2. 构造虚拟状态测试输入：当前各个关节的角度(rad)和角速度(rad/s)
    std::vector<double> joint_positions  = {10.0*M_PI/180.0, -15.0*M_PI/180.0, 30.0*M_PI/180.0, 20.0*M_PI/180.0, 0.0, -20*M_PI/180.0, 30.0*M_PI/180.0};
    std::vector<double> joint_velocities = {0.1, 0.2,  0.0, -0.1, 0.0,  0.1, 0.0};

    // 提前为输出数据分配内存
    std::vector<double> gravity(dof, 0.0);
    std::vector<double> coriolis(dof, 0.0);
    Eigen::MatrixXd mass_matrix;
    
    // 以下为之前保留的其它结果
    std::vector<double> inertia_diag(dof, 0.0);
    Eigen::MatrixXd jacobian;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    // 3. 调用动力学获取接口获取这3个核心数据以及其他周边数据
    dyn.GetGravity(joint_positions.data(), gravity.data());
    dyn.GetCoriolis(joint_positions.data(), joint_velocities.data(), coriolis.data());
    dyn.GetMassMatrix(joint_positions.data(), mass_matrix);
    
    dyn.GetMassMatrixDiagonal(joint_positions.data(), inertia_diag.data());
    dyn.GetJacobian(joint_positions.data(), jacobian);
    dyn.GetEECordinate(joint_positions.data(), R, p);

    // 4. 美化格式并打印结果
    std::cout << "========================== 动力学三大矩阵与核心数据 ==========================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "\n[输入姿态] 各关节角度 (rad/q):        ";
    for(int i = 0; i < dof; i++) std::cout << std::setw(8) << joint_positions[i] << " ";
    
    std::cout << "\n[输入速度] 各关节角速度 (rad/s/q_dot):";
    for(int i = 0; i < dof; i++) std::cout << std::setw(8) << joint_velocities[i] << " ";
    std::cout << "\n----------------------------------------------------------------------" << std::endl;

    std::cout << "\n1. 重力向心矩阵 (Gravity Vector) G(q) [" << dof << "x1] [Nm]:\n";
    for(int i = 0; i < dof; i++) {
        std::cout << std::setw(10) << gravity[i] << "\n";
    }

    std::cout << "\n2. 科里奥利/离心力向心矩阵 (Coriolis Vector) C(q, q_dot)*q_dot [" << dof << "x1] [Nm]:\n";
    for(int i = 0; i < dof; i++) {
        std::cout << std::setw(10) << coriolis[i] << "\n";
    }

    std::cout << "\n3. 广义质量惯性矩阵 (Mass Matrix) M(q) [" << dof << "x" << dof << "]:\n";
    std::cout << mass_matrix << "\n";
    
    std::cout << "\n----------------------------- 补充正向运动学数据 -----------------------------" << std::endl;
    
    std::cout << "4. [惯性矩阵对角线] 各自由度自身等效阻力 (Mass Diagonal) [kg·m^2]:\n   ";
    for(int i = 0; i < dof; i++) std::cout << std::setw(8) << inertia_diag[i] << " ";
    std::cout << "\n\n";

    std::cout << "5. [正运动学] 末端笛卡尔空间坐标 (EE Position XYZ) [m]:\n   ";
    std::cout << "X: " << p(0) << ", Y: " << p(1) << ", Z: " << p(2) << "\n\n";

    std::cout << "6. [正运动学] 末端齐次旋转矩阵 (EE Rotation) [3x3]:\n";
    std::cout << R << "\n\n";

    std::cout << "7. [雅可比矩阵] 将关节速度映射到笛卡尔速度的矩阵 (Jacobian Matrix) [6x" << dof << "]:\n";
    std::cout << jacobian << "\n\n";

    std::cout << "======================================================================" << std::endl;

    return 0;
}