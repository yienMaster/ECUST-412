def kalman_filter_explicit(x, p, z, F, H, Q, R):
    """
    直接计算卡尔曼滤波，优化计算效率
    x: [x1, x2]  当前状态估计
    p: [[p11, p12], [p21, p22]]  当前误差协方差
    z: [z1, z2]  测量值
    F, H, Q, R:  卡尔曼滤波参数矩阵
    """

    # 提前计算 F 的平方和乘积，减少重复运算
    F00_2 = F[0][0] * F[0][0]
    F01_2 = F[0][1] * F[0][1]
    F10_2 = F[1][0] * F[1][0]
    F11_2 = F[1][1] * F[1][1]
    F00_F01 = 2 * F[0][0] * F[0][1]
    F10_F11 = 2 * F[1][0] * F[1][1]

    # **预测步骤**
    x1_pred = F[0][0] * x[0] + F[0][1] * x[1]
    x2_pred = F[1][0] * x[0] + F[1][1] * x[1]

    # **预测 P_pred**
    p11_pred = F00_2 * p[0][0] + F00_F01 * p[1][0] + F01_2 * p[1][1] + Q[0][0]
    p12_pred = F[0][0] * F[1][0] * p[0][0] + (F[0][0] * F[1][1] + F[0][1] * F[1][0]) * p[1][0] + F[0][1] * F[1][1] * p[1][1] + Q[0][1]
    p21_pred = p12_pred  # 由于 P 是对称矩阵，p12_pred 和 p21_pred 相同
    p22_pred = F10_2 * p[0][0] + F10_F11 * p[1][0] + F11_2 * p[1][1] + Q[1][1]

    # **计算 S = H * P_pred * H^T + R**
    S11 = p11_pred + R[0][0]
    S12 = p12_pred + R[0][1]
    S21 = S12  # 对称矩阵
    S22 = p22_pred + R[1][1]

    # **计算 S 的逆**
    det_S = S11 * S22 - S12 * S21
    inv_det_S = 1 / det_S  # 直接计算倒数，减少除法操作
    S11_inv = S22 * inv_det_S
    S12_inv = -S12 * inv_det_S
    S21_inv = S12_inv  # 对称矩阵
    S22_inv = S11 * inv_det_S

    # **计算卡尔曼增益 K**
    K11 = p11_pred * S11_inv + p12_pred * S21_inv
    K12 = p11_pred * S12_inv + p12_pred * S22_inv
    K21 = p21_pred * S11_inv + p22_pred * S21_inv
    K22 = p21_pred * S12_inv + p22_pred * S22_inv

    # **计算测量残差 y = z - H * x_pred**
    y1 = z[0] - x1_pred
    y2 = z[1] - x2_pred

    # **更新状态估计**
    x1_est = x1_pred + K11 * y1 + K12 * y2
    x2_est = x2_pred + K21 * y1 + K22 * y2

    # **更新协方差 P_est**
    P11_est = (1 - K11) * p11_pred - K12 * p21_pred
    P12_est = (1 - K11) * p12_pred - K12 * p22_pred
    P21_est = -K21 * p11_pred + (1 - K22) * p21_pred
    P22_est = -K21 * p12_pred + (1 - K22) * p22_pred

    return [x1_est, x2_est], [[P11_est, P12_est], [P21_est, P22_est]]


# **初始化参数**
x = [0, 0]
p = [[550, 0], [0, 550]]

# 状态转移矩阵 F
F = [[1, 0.001], [0, 1]]

# 观测矩阵 H
H = [[1, 0], [0, 1]]

# 过程噪声协方差 Q
Q = [[1e-4, 0], [0, 1e-3]]

# 观测噪声协方差 R
R = [[0.01, 0], [0, 0.25]]

# **示例测量值**
theita = 1.5  # 角度
w_y = 0.2  # 角速度
z = [theita, w_y]

# **计算卡尔曼滤波**
x, p = kalman_filter_explicit(x, p, z, F, H, Q, R)
print("状态估计值:", x)
print("误差协方差:", p)
