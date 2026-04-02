#!/usr/bin/env python3
import argparse
import csv
import math
import random
import statistics
import time
import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node


SUCCESS_CODE = 1


@dataclass
class SampleResult:
    index: int
    success: bool
    ik_time_ms: float
    pos_err_m: float
    rot_err_rad: float
    failure_reason: str


class IKValidator(Node):
    def __init__(self, node_name: str = "moveit_kdl_ik_validator"):
        super().__init__(node_name)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def wait_for_services(self, timeout_sec: float = 10.0) -> None:
        if not self.fk_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("服务 /compute_fk 不可用，请先启动 move_group")
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("服务 /compute_ik 不可用，请先启动 move_group")

    def get_robot_description(self) -> str:
        for param_service in ("/move_group/get_parameters", "/robot_state_publisher/get_parameters"):
            client = self.create_client(GetParameters, param_service)
            if not client.wait_for_service(timeout_sec=2.0):
                continue
            req = GetParameters.Request()
            req.names = ["robot_description"]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() is None:
                continue
            result = future.result()
            if result.values and result.values[0].string_value:
                return result.values[0].string_value

        raise RuntimeError("无法获取 robot_description 参数")

    def compute_fk(self, joint_names: Sequence[str], joint_positions: Sequence[float], tip_link: str):
        req = GetPositionFK.Request()
        req.fk_link_names = [tip_link]
        req.robot_state.joint_state.name = list(joint_names)
        req.robot_state.joint_state.position = list(joint_positions)

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        if resp is None:
            raise RuntimeError("/compute_fk 调用超时")
        return resp

    def compute_ik(
        self,
        group_name: str,
        tip_link: str,
        target_pose_stamped,
        seed_names: Sequence[str],
        seed_positions: Sequence[float],
        timeout_sec: float,
    ):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = tip_link
        req.ik_request.pose_stamped = target_pose_stamped
        req.ik_request.robot_state.joint_state.name = list(seed_names)
        req.ik_request.robot_state.joint_state.position = list(seed_positions)
        req.ik_request.timeout = Duration(seconds=timeout_sec).to_msg()
        req.ik_request.avoid_collisions = False

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec + 1.0)
        resp = future.result()
        if resp is None:
            raise RuntimeError("/compute_ik 调用超时")
        return resp


def parse_joint_limits_from_urdf(urdf_text: str, joint_names: Sequence[str]) -> Dict[str, Tuple[float, float]]:
    root = et.fromstring(urdf_text)
    required = set(joint_names)
    limits: Dict[str, Tuple[float, float]] = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name not in required:
            continue

        joint_type = joint.attrib.get("type", "")
        if joint_type == "continuous":
            limits[name] = (-math.pi, math.pi)
            continue

        limit_el = joint.find("limit")
        if limit_el is None:
            continue

        low = float(limit_el.attrib.get("lower", -math.pi))
        high = float(limit_el.attrib.get("upper", math.pi))
        limits[name] = (low, high)

    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise RuntimeError(f"URDF 缺少关节限位: {missing}")
    return limits


def sample_joint_vector(joint_names: Sequence[str], joint_limits: Dict[str, Tuple[float, float]]) -> List[float]:
    values = []
    for name in joint_names:
        low, high = joint_limits[name]
        width = high - low
        margin = 0.05 * width
        safe_low = low + margin
        safe_high = high - margin
        if safe_low >= safe_high:
            safe_low, safe_high = low, high
        values.append(random.uniform(safe_low, safe_high))
    return values


def clip_to_limits(q: Sequence[float], joint_names: Sequence[str], joint_limits: Dict[str, Tuple[float, float]]) -> List[float]:
    out = []
    for value, name in zip(q, joint_names):
        low, high = joint_limits[name]
        out.append(float(np.clip(value, low, high)))
    return out


def quat_angle_error(a: Pose, b: Pose) -> float:
    qa = np.array([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w], dtype=float)
    qb = np.array([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w], dtype=float)
    qa = qa / np.linalg.norm(qa)
    qb = qb / np.linalg.norm(qb)
    dot = float(np.clip(np.abs(np.dot(qa, qb)), -1.0, 1.0))
    return 2.0 * math.acos(dot)


def pos_error(a: Pose, b: Pose) -> float:
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def extract_solution(solution_names: Sequence[str], solution_positions: Sequence[float], target_joint_names: Sequence[str]) -> Optional[List[float]]:
    mapping = {name: pos for name, pos in zip(solution_names, solution_positions)}
    out = []
    for name in target_joint_names:
        if name not in mapping:
            return None
        out.append(mapping[name])
    return out


def default_joint_names(group_name: str) -> List[str]:
    if group_name == "left_arm":
        prefix = "openarm_left_joint"
    elif group_name == "right_arm":
        prefix = "openarm_right_joint"
    else:
        raise ValueError("请通过 --joint-names 显式指定关节列表")

    return [f"{prefix}{i}" for i in range(1, 8)]


def default_tip_link(group_name: str) -> str:
    if group_name == "left_arm":
        return "openarm_left_hand"
    if group_name == "right_arm":
        return "openarm_right_hand"
    raise ValueError("请通过 --tip-link 显式指定末端链接")


def summarize(results: Sequence[SampleResult]) -> None:
    total = len(results)
    succ = [r for r in results if r.success]
    fail = total - len(succ)
    print("\n===== IK 验证结果 =====")
    print(f"样本数: {total}")
    print(f"成功数: {len(succ)}")
    print(f"失败数: {fail}")
    print(f"成功率: {100.0 * len(succ) / max(total, 1):.2f}%")

    if succ:
        ik_times = [r.ik_time_ms for r in succ]
        pos_errs = [r.pos_err_m for r in succ]
        rot_errs = [r.rot_err_rad for r in succ]
        print(f"IK耗时(ms): 平均={statistics.mean(ik_times):.3f}, P95={np.percentile(ik_times, 95):.3f}, 最大={max(ik_times):.3f}")
        print(f"位置误差(m): 平均={statistics.mean(pos_errs):.6f}, P95={np.percentile(pos_errs, 95):.6f}, 最大={max(pos_errs):.6f}")
        print(f"姿态误差(rad): 平均={statistics.mean(rot_errs):.6f}, P95={np.percentile(rot_errs, 95):.6f}, 最大={max(rot_errs):.6f}")


def main():
    parser = argparse.ArgumentParser(description="MoveIt KDL IK 快速闭环验证 (FK->IK->FK)")
    parser.add_argument("--group", default="left_arm", help="规划组名称，例如 left_arm / right_arm")
    parser.add_argument("--tip-link", default=None, help="末端 link 名称，例如 openarm_left_hand")
    parser.add_argument(
        "--joint-names",
        default=None,
        help="逗号分隔关节名，如 openarm_left_joint1,...,openarm_left_joint7",
    )
    parser.add_argument("--samples", type=int, default=200, help="随机样本数量")
    parser.add_argument("--ik-timeout", type=float, default=0.02, help="单次 IK 超时（秒）")
    parser.add_argument("--seed-noise", type=float, default=0.20, help="种子扰动比例（关节范围百分比）")
    parser.add_argument("--csv", default="ik_validation_results.csv", help="结果 CSV 输出路径")
    parser.add_argument("--random-seed", type=int, default=42, help="随机种子")
    args = parser.parse_args()

    random.seed(args.random_seed)
    np.random.seed(args.random_seed)

    joint_names = (
        [name.strip() for name in args.joint_names.split(",") if name.strip()]
        if args.joint_names
        else default_joint_names(args.group)
    )
    tip_link = args.tip_link if args.tip_link else default_tip_link(args.group)

    rclpy.init()
    node = IKValidator()

    try:
        node.wait_for_services(timeout_sec=10.0)
        urdf_text = node.get_robot_description()
        joint_limits = parse_joint_limits_from_urdf(urdf_text, joint_names)

        results: List[SampleResult] = []

        for i in range(args.samples):
            q_true = sample_joint_vector(joint_names, joint_limits)

            fk_target = node.compute_fk(joint_names, q_true, tip_link)
            if fk_target.error_code.val != SUCCESS_CODE or not fk_target.pose_stamped:
                results.append(SampleResult(i, False, 0.0, float("nan"), float("nan"), "fk_target_failed"))
                continue

            target_pose_stamped = fk_target.pose_stamped[0]

            # 构造 IK 种子（在真实解附近加噪声，更接近实际控制场景）
            q_seed = []
            for name, q_val in zip(joint_names, q_true):
                low, high = joint_limits[name]
                width = high - low
                q_seed.append(q_val + random.uniform(-args.seed_noise, args.seed_noise) * width)
            q_seed = clip_to_limits(q_seed, joint_names, joint_limits)

            start = time.perf_counter()
            ik_resp = node.compute_ik(
                group_name=args.group,
                tip_link=tip_link,
                target_pose_stamped=target_pose_stamped,
                seed_names=joint_names,
                seed_positions=q_seed,
                timeout_sec=args.ik_timeout,
            )
            elapsed_ms = (time.perf_counter() - start) * 1000.0

            if ik_resp.error_code.val != SUCCESS_CODE:
                results.append(SampleResult(i, False, elapsed_ms, float("nan"), float("nan"), f"ik_failed_{ik_resp.error_code.val}"))
                continue

            q_ik = extract_solution(
                ik_resp.solution.joint_state.name,
                ik_resp.solution.joint_state.position,
                joint_names,
            )
            if q_ik is None:
                results.append(SampleResult(i, False, elapsed_ms, float("nan"), float("nan"), "ik_solution_missing_joints"))
                continue

            fk_ik = node.compute_fk(joint_names, q_ik, tip_link)
            if fk_ik.error_code.val != SUCCESS_CODE or not fk_ik.pose_stamped:
                results.append(SampleResult(i, False, elapsed_ms, float("nan"), float("nan"), "fk_ik_failed"))
                continue

            pose_t = target_pose_stamped.pose
            pose_k = fk_ik.pose_stamped[0].pose
            pe = pos_error(pose_t, pose_k)
            re = quat_angle_error(pose_t, pose_k)
            results.append(SampleResult(i, True, elapsed_ms, pe, re, ""))

            if (i + 1) % 20 == 0:
                node.get_logger().info(f"已完成 {i + 1}/{args.samples}")

        summarize(results)

        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "success", "ik_time_ms", "pos_err_m", "rot_err_rad", "failure_reason"])
            for r in results:
                writer.writerow([r.index, int(r.success), f"{r.ik_time_ms:.6f}", r.pos_err_m, r.rot_err_rad, r.failure_reason])

        print(f"\nCSV 已保存: {args.csv}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
