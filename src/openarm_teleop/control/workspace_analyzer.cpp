// 项目内动力学接口：
// - 使用 Dynamics 类做 FK（GetEECordinate）和雅可比（GetJacobian）计算
#include <controller/dynamics.hpp>

// Eigen矩阵/向量与SVD：
// - Eigen::MatrixXd / Matrix3d / Vector3d
// - Eigen::JacobiSVD（通过 Dense 头引入）
#include <Eigen/Dense>

// 通用算法：
// - std::min / std::max / std::swap
#include <algorithm>

// 数学函数：
// - std::isfinite / std::log / std::exp
#include <cmath>

// 文件读URDF：
// - std::ifstream
#include <fstream>

// 控制台格式化输出：
// - std::setprecision / std::fixed
#include <iomanip>

// 标准输入输出：
// - std::cout / std::cerr / std::endl
#include <iostream>

// 数值上下界：
// - std::numeric_limits<double>::infinity()
#include <limits>

// 随机采样：
// - std::mt19937 / std::uniform_real_distribution
#include <random>

// 字符串流（把URDF文件读入内存字符串）：
// - std::stringstream
#include <sstream>

// 字符串类型：
// - std::string
#include <string>

// 动态数组容器：
// - std::vector<...>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// 单个关节的采样边界（弧度）
// - 对于URDF中continuous关节，使用[-pi, pi]作为一圈采样范围
// - 对于有明确limit的关节，使用URDF给出的lower/upper
struct JointBound {
  std::string name;
  double lower;
  double upper;
};

// 三维包围盒，用于统计工作空间范围
// 通过不断更新所有采样点的min/max，最终得到x/y/z轴向范围
struct Range3D {
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();

  void update(const Eigen::Vector3d& p) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
    min_z = std::min(min_z, p.z());
    max_z = std::max(max_z, p.z());
  }

  bool valid() const {
    return std::isfinite(min_x) && std::isfinite(max_x) && std::isfinite(min_y) &&
           std::isfinite(max_y) && std::isfinite(min_z) && std::isfinite(max_z);
  }
};

// 全局配置
// - world_link: 最终输出坐标系（通常是world）
// - base_link/end_link: 机械臂关节链的起止link（用于关节采样与雅可比计算）
// - dex_manip_min/dex_cond_max: 灵巧空间判定阈值
//   1) manipulability >= dex_manip_min
//   2) condition_number <= dex_cond_max
struct Config {
  std::string urdf_path = "/home/xiaokai/OpenArm/openarm_bimanual.urdf";
  std::string world_link = "world";
  std::string base_link = "openarm_left_link0";
  std::string end_link = "openarm_left_hand_tcp";
  size_t samples = 200000;
  double dex_manip_min = 0.01;
  double dex_cond_max = 80.0;
  uint32_t seed = 42;
};

// 打印命令行帮助
void PrintUsage(const char* prog) {
  std::cout
      << "Usage:\n"
      << "  " << prog
      << " [--urdf <path>] [--world-link <name>] [--base-link <name>] [--end-link <name>]\\n"
      << "     [--samples <int>] [--dex-manip-min <double>] [--dex-cond-max <double>]\\n"
      << "     [--seed <int>]\\n\\n"
      << "Example:\n"
      << "  " << prog
      << " --urdf /home/xiaokai/OpenArm/openarm_bimanual.urdf --world-link world "
      << "--base-link openarm_left_link0 --end-link openarm_left_hand_tcp --samples 300000\\n";
}

// 解析命令行参数
// 返回true表示参数有效；返回false表示参数无效或用户请求help
bool ParseArgs(int argc, char** argv, Config& cfg) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto need_value = [&](const std::string& key) -> bool {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for argument: " << key << std::endl;
        return false;
      }
      return true;
    };

    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return false;
    } else if (arg == "--urdf") {
      if (!need_value(arg)) return false;
      cfg.urdf_path = argv[++i];
    } else if (arg == "--world-link") {
      if (!need_value(arg)) return false;
      cfg.world_link = argv[++i];
    } else if (arg == "--base-link") {
      if (!need_value(arg)) return false;
      cfg.base_link = argv[++i];
    } else if (arg == "--end-link") {
      if (!need_value(arg)) return false;
      cfg.end_link = argv[++i];
    } else if (arg == "--samples") {
      if (!need_value(arg)) return false;
      cfg.samples = static_cast<size_t>(std::stoull(argv[++i]));
    } else if (arg == "--dex-manip-min") {
      if (!need_value(arg)) return false;
      cfg.dex_manip_min = std::stod(argv[++i]);
    } else if (arg == "--dex-cond-max") {
      if (!need_value(arg)) return false;
      cfg.dex_cond_max = std::stod(argv[++i]);
    } else if (arg == "--seed") {
      if (!need_value(arg)) return false;
      cfg.seed = static_cast<uint32_t>(std::stoul(argv[++i]));
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      PrintUsage(argv[0]);
      return false;
    }
  }

  if (cfg.samples == 0) {
    std::cerr << "--samples must be > 0" << std::endl;
    return false;
  }
  if (cfg.dex_manip_min < 0.0 || cfg.dex_cond_max <= 1.0) {
    std::cerr << "Invalid dexterity thresholds." << std::endl;
    return false;
  }
  return true;
}

// 读取URDF并提取从base_link到end_link链路上的“可动关节”限位
// 输出：
// - bounds_out: 每个关节采样上下限
// - chain_joint_names_out: 关节名称（便于后续扩展调试/日志）
bool LoadJointBounds(const std::string& urdf_path, const std::string& base_link,
                     const std::string& end_link, std::vector<JointBound>& bounds_out,
                     std::vector<std::string>& chain_joint_names_out) {
  std::ifstream file(urdf_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open URDF file: " << urdf_path << std::endl;
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  auto model = urdf::parseURDF(buffer.str());
  if (!model) {
    std::cerr << "Failed to parse URDF: " << urdf_path << std::endl;
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
    std::cerr << "Failed to extract KDL tree from URDF." << std::endl;
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(base_link, end_link, chain)) {
    std::cerr << "Failed to get KDL chain: " << base_link << " -> " << end_link << std::endl;
    return false;
  }

  bounds_out.clear();
  chain_joint_names_out.clear();
  bounds_out.reserve(chain.getNrOfJoints());
  chain_joint_names_out.reserve(chain.getNrOfJoints());

  for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
    const auto& joint = chain.getSegment(i).getJoint();
    // 跳过固定关节
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }

    const std::string joint_name = joint.getName();
    chain_joint_names_out.push_back(joint_name);

    auto urdf_joint = model->getJoint(joint_name);
    JointBound jb;
    jb.name = joint_name;

    if (!urdf_joint) {
      // 兜底策略：若URDF中查不到关节定义，给一圈范围
      jb.lower = -kPi;
      jb.upper = kPi;
    } else {
      if (urdf_joint->type == urdf::Joint::CONTINUOUS) {
        // 连续关节无显式上下限，按[-pi, pi]采样
        jb.lower = -kPi;
        jb.upper = kPi;
      } else if (urdf_joint->limits) {
        // 普通关节使用URDF限制
        jb.lower = urdf_joint->limits->lower;
        jb.upper = urdf_joint->limits->upper;
      } else {
        // 无limit信息时兜底
        jb.lower = -kPi;
        jb.upper = kPi;
      }
    }

    if (jb.upper < jb.lower) {
      std::swap(jb.lower, jb.upper);
    }

    bounds_out.push_back(jb);
  }

  if (bounds_out.empty()) {
    std::cerr << "No actuated joint found on chain." << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  Config cfg;
  if (!ParseArgs(argc, argv, cfg)) {
    return (argc > 1 ? 1 : 0);
  }

  std::vector<JointBound> bounds;
  std::vector<std::string> chain_joint_names;
  if (!LoadJointBounds(cfg.urdf_path, cfg.base_link, cfg.end_link, bounds, chain_joint_names)) {
    return 1;
  }

  std::cout << "[INFO] URDF       : " << cfg.urdf_path << std::endl;
  std::cout << "[INFO] Kin Chain  : " << cfg.base_link << " -> " << cfg.end_link << std::endl;
  std::cout << "[INFO] Out Frame  : " << cfg.world_link << " -> " << cfg.end_link << std::endl;
  std::cout << "[INFO] Samples    : " << cfg.samples << std::endl;
  std::cout << "[INFO] Dexterity  : manipulability >= " << cfg.dex_manip_min
            << ", condition number <= " << cfg.dex_cond_max << std::endl;
  std::cout << "[INFO] Joint DOF  : " << bounds.size() << std::endl;

  // 这里分两个Dynamics求解器，目的是“物理解耦”：
  // 1) dyn_kin: base_link->end_link
  //    - 用于雅可比J(q)与灵巧性指标计算
  //    - 关节维度与机械臂本体一致（例如7DOF）
  // 2) dyn_world: world_link->end_link
  //    - 用于输出末端在world坐标系下的位置
  //    - 保证你拿到的是可直接用于规划边界的world范围
  Dynamics dyn_kin(cfg.urdf_path, cfg.base_link, cfg.end_link);
  if (!dyn_kin.Init()) {
    std::cerr << "Failed to initialize kinematic Dynamics." << std::endl;
    return 1;
  }

  Dynamics dyn_world(cfg.urdf_path, cfg.world_link, cfg.end_link);
  if (!dyn_world.Init()) {
    std::cerr << "Failed to initialize world-frame Dynamics." << std::endl;
    return 1;
  }

  const size_t dof = bounds.size();
  std::vector<double> q(dof, 0.0);
  std::vector<std::uniform_real_distribution<double>> dists;
  dists.reserve(dof);
  for (const auto& b : bounds) {
    dists.emplace_back(b.lower, b.upper);
  }

  std::mt19937 rng(cfg.seed);

  Range3D reachable_range;
  Range3D dexterous_range;

  size_t reachable_count = 0;
  size_t dexterous_count = 0;

  Eigen::MatrixXd J;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;

  for (size_t i = 0; i < cfg.samples; ++i) {
    // Step 1) 在关节限位内做均匀随机采样
    for (size_t j = 0; j < dof; ++j) {
      q[j] = dists[j](rng);
    }

    // Step 2) 用world->ee链得到末端点位（用于可达空间）
    dyn_world.GetEECordinate(q.data(), R, p);
    if (!p.allFinite()) {
      continue;
    }

    // 采样点可求得有效FK，即计入可达空间统计
    reachable_range.update(p);
    ++reachable_count;

    // Step 3) 用base->ee链计算雅可比（用于灵巧性判定）
    dyn_kin.GetJacobian(q.data(), J);
    if (!J.allFinite()) {
      continue;
    }

    // Step 4) SVD分解雅可比，得到奇异值谱
    // J = U * S * V^T，其中S对角为奇异值
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd s = svd.singularValues();
    if (s.size() == 0) {
      continue;
    }

    const double s_min = s.minCoeff();
    const double s_max = s.maxCoeff();
    // 极小奇异值意味着接近奇异位形，直接过滤
    if (s_min <= 1e-12 || s_max <= 1e-12) {
      continue;
    }

    // 灵巧性指标1：manipulability
    // 采用奇异值乘积（等价于sqrt(det(J*J^T))，对非方阵也适用）
    // 为防止数值下溢，这里在log域累加后再exp回去
    double log_sum = 0.0;
    for (int k = 0; k < s.size(); ++k) {
      log_sum += std::log(std::max(s(k), 1e-12));
    }
    const double manipulability = std::exp(log_sum);

    // 灵巧性指标2：条件数 condition number = sigma_max / sigma_min
    // 条件数越小，局部速度映射越均匀、数值稳定性越好
    const double cond = s_max / s_min;

    // Step 5) 灵巧空间判定
    // 满足双阈值：
    // - manipulability足够大（远离低可控区域）
    // - condition number足够小（避免病态方向放大）
    if (manipulability >= cfg.dex_manip_min && cond <= cfg.dex_cond_max) {
      dexterous_range.update(p);
      ++dexterous_count;
    }
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n================ Workspace Result (Frame: " << cfg.world_link
            << ") ================\n";

  if (reachable_count == 0 || !reachable_range.valid()) {
    std::cout << "[Reachable Workspace] No valid samples." << std::endl;
  } else {
    std::cout << "[Reachable Workspace] samples=" << reachable_count << "\n";
    std::cout << "  x range: [" << reachable_range.min_x << ", " << reachable_range.max_x << "] m\n";
    std::cout << "  y range: [" << reachable_range.min_y << ", " << reachable_range.max_y << "] m\n";
    std::cout << "  z range: [" << reachable_range.min_z << ", " << reachable_range.max_z << "] m\n";
  }

  if (dexterous_count == 0 || !dexterous_range.valid()) {
    std::cout << "[Dexterous Workspace] No valid samples under current thresholds." << std::endl;
  } else {
    std::cout << "[Dexterous Workspace] samples=" << dexterous_count << "\n";
    std::cout << "  x range: [" << dexterous_range.min_x << ", " << dexterous_range.max_x << "] m\n";
    std::cout << "  y range: [" << dexterous_range.min_y << ", " << dexterous_range.max_y << "] m\n";
    std::cout << "  z range: [" << dexterous_range.min_z << ", " << dexterous_range.max_z << "] m\n";
  }

  std::cout << "===============================================================\n";

  return 0;
}
