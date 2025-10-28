#位姿解算和卡尔曼滤波
import math
import matrix_cal as matr

def deal_imu_data(data):
    '''
    解算位姿，并进行单位换算。
    data为imu返回数据
    
    返回：
    theita 俯仰角
    w_y theita方向角速度
    '''
     #位姿解算
    if(data[0] == 0):
        a = 0.001
    else:
        a = data[0]
    theita = math.atan2(data[2] , a)
    
    if(data[7] == 0):
        b = 0.001
    else:
        b = data[7]
    w_y_dps = data[4]
    
    #单位换算
    trans = 250 / 65535 * 2**1 * math.pi / 180
    w_y = w_y_dps * trans + 0.000864
    
    # 返回beita和w_y
    return theita, w_y
    
    
def kalman_filter(x, p, z, F, H, Q, R):
    """
    卡尔曼滤波器的实现
    参数：
    x - 上一次的状态估计值 (1xN 向量)
    p - 上一次的协方差矩阵 (NxN 矩阵)
    z - 本次的测量值 (1xN 向量)
    F - 状态转移矩阵 (NxN 矩阵)
    H - 观测矩阵 (MxN 矩阵)
    Q - 过程噪声协方差矩阵 (NxN 矩阵)
    R - 观测噪声协方差矩阵 (MxM 矩阵)
    
    返回：
    x_est - 更新后的状态估计值
    p_est - 更新后的协方差矩阵
    """
    # 预测步骤
    # 预测新的状态估计
    
    x_pred = matr.matrix_multiply(F, x)  # x_pred = F * x
    # 预测新的协方差
    p_pred = matr.matrix_multiply(F, matr.matrix_multiply(p, matr.matrix_transpose(F)))  # p_pred = F * P * F^T
    p_pred = matr.matrix_add(p_pred, Q)  # p_pred = p_pred + Q

    # 更新步骤
    # 计算卡尔曼增益
    H_p_pred = matr.matrix_multiply(H, p_pred)  # H * p_pred
    H_p_pred_HT = matr.matrix_multiply(H_p_pred, matrix_transpose(H))  # H * p_pred * H^T
    S = matr.matrix_add(H_p_pred_HT, R)  # S = H * p_pred * H^T + R
    S_inv = matr.matrix_inverse(S)  # S_inv = (S)^-1
    K = matr.matrix_multiply(p_pred, matr.matrix_multiply(matrix_transpose(H), S_inv))  # K = p_pred * H^T * S_inv

    # 更新状态估计
    y = matr.matrix_sub(z, matr.matrix_multiply(H, x_pred))  # y = z - H * x_pred
    x_est = matr.matrix_add(x_pred, matr.matrix_multiply(K, y))  # x_est = x_pred + K * y

    # 更新协方差矩阵
    I = matr.identity_matrix()  # 单位矩阵
    KH = matr.matrix_multiply(K, H)  # K * H
    p_est = matr.matrix_sub(p_pred, matr.matrix_multiply(KH, p_pred))  # p_est = p_pred - K * H * p_pred

    return x_est, p_est