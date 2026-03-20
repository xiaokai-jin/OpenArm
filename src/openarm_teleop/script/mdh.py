import numpy as np


def mdh_transform(theta_i, alpha_i_minus_1, a_i_minus_1, d_i):
    """Craig 改进DH（右手系）: ^{i-1}T_i = Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)"""
    ct = np.cos(theta_i)
    st = np.sin(theta_i)
    ca = np.cos(alpha_i_minus_1)
    sa = np.sin(alpha_i_minus_1)
    return np.array(
        [
            [ct, -st, 0.0, a_i_minus_1],
            [st * ca, ct * ca, -sa, -d_i * sa],
            [st * sa, ct * sa, ca, d_i * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rot_x(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def forward_kinematics_mdh(param_table, q=None):
    """
    param_table 每行: [theta_offset_i, alpha_{i-1}, a_{i-1}, d_i]
    theta_i = q_i + theta_offset_i

    返回:
      T_all: [T0, T1, ..., T7]
      T_end: T0_7
    """
    n = len(param_table)
    if q is None:
        q = np.zeros(n, dtype=float)
    q = np.asarray(q, dtype=float)
    if q.shape[0] != n:
        raise ValueError(f"q 维度错误，期望 {n}，实际 {q.shape[0]}")

    T = np.eye(4)
    T_all = [T.copy()]

    for i in range(n):
        theta_offset, alpha_prev, a_prev, d_i = param_table[i]
        theta_i = q[i] + theta_offset
        T_i = mdh_transform(theta_i, alpha_prev, a_prev, d_i)
        T = T @ T_i
        T_all.append(T.copy())

    return T_all, T


def print_frame_table(T_all):
    print("\n逐步坐标系原点（相对 link0，右手系）")
    print("frame |      x       y       z")
    print("------+---------------------------")
    for i, T in enumerate(T_all):
        p = T[:3, 3]
        print(f"{i:>5} | {p[0]:>7.4f} {p[1]:>7.4f} {p[2]:>7.4f}")


def draw_frames_3d(T_all, axis_len=0.04, save_path="/home/xiaokai/OpenArm/src/openarm_teleop/script/mdh_frames_righthand.png"):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("\n未安装 matplotlib，跳过绘图。")
        return

    labels = [f"F{i}" for i in range(len(T_all))]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    all_p = np.array([T[:3, 3] for T in T_all])
    mins = all_p.min(axis=0) - 0.08
    maxs = all_p.max(axis=0) + 0.08

    for i, (T, label) in enumerate(zip(T_all, labels)):
        p = T[:3, 3]
        R = T[:3, :3]

        x_axis = R[:, 0] * axis_len
        # y_axis = R[:, 1] * axis_len
        z_axis = R[:, 2] * axis_len

        ax.quiver(p[0], p[1], p[2], x_axis[0], x_axis[1], x_axis[2], color="r")
        # ax.quiver(p[0], p[1], p[2], y_axis[0], y_axis[1], y_axis[2], color="g")
        ax.quiver(p[0], p[1], p[2], z_axis[0], z_axis[1], z_axis[2], color="b")

        ax.text(p[0], p[1], p[2], label, fontsize=6, color="k")
        ax.text(*(p + x_axis), f"x_{i}", color="r", fontsize=10)
        # ax.text(*(p + y_axis), f"y_{i}", color="g", fontsize=15)
        ax.text(*(p + z_axis), f"z_{i}", color="b", fontsize=20)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])
    ax.set_title("MDH Frames (Right-Handed) link0 -> link7")
    ax.view_init(elev=24, azim=135)
    plt.tight_layout()
    # plt.savefig(save_path, dpi=180)
    # print(f"\n坐标系图已保存: {save_path}")
    plt.show()


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    # 零位关节角
    q_zero = np.zeros(7, dtype=float)

    # 参数格式: [theta_offset_i, alpha_{i-1}, a_{i-1}, d_i]
    # 这里只保留纯 MDH 右手系，不做额外坐标系变换
    mdh_table = [
        [np.deg2rad(-90), np.deg2rad(0), 0.0, 0.1225],
        [np.deg2rad(-90), np.deg2rad(-90), 0.0, 0.0],
        [np.deg2rad(90), np.deg2rad(90), 0.0, 0.22],
        [np.deg2rad(0), np.deg2rad(90), 0.0, 0.0],
        [np.deg2rad(-90), np.deg2rad(-90), 0.0, 0.216],
        [np.deg2rad(90), np.deg2rad(-90), 0.0, 0.0],
        [np.deg2rad(90), np.deg2rad(90), 0.0, 0.0],
    ]
    #建立的mdh中的第二个关节和第4个关节旋转轴方向与rviz中相反,从而使得它们的旋转方向与rviz中的相反。这是由于MDH参数化方式的定义所导致的，与具体的实现细节有关。
    q=np.array([np.deg2rad(10.0), np.deg2rad(15.0), np.deg2rad(-30.0), np.deg2rad(-20.0), np.deg2rad(10.0), np.deg2rad(-20.0), np.deg2rad(30.0)], dtype=float)  # 可修改为其他关节角度测试
    q_zero=np.zeros(7, dtype=float)  # 零位关节角
    T_all, T_end = forward_kinematics_mdh(mdh_table, q)

    print("\n总齐次变换矩阵 (0^T_7):")
    print(T_end)
    print("\n末端位置 (x, y, z) [m]:")
    print(T_end[:3, 3])
    print("\n末端旋转矩阵 R:")
    print(T_end[:3, :3])

    # link7 旋转矩阵再绕 x7 旋转 -90°
    R_end = T_end[:3, :3]
    R_final = R_end @ rot_x(np.deg2rad(-90.0))
    print("\n最终末端旋转矩阵 R_final = R_end * Rx(-90deg):")
    print(R_final)

    print_frame_table(T_all)
    draw_frames_3d(T_all)
