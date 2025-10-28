#卡尔曼滤波
    z = [[theita] ,[w_y]]
    x, p = kalman_filter(x, p, z, F, H, Q, R)
    
'''
函数与参数
'''
#矩阵计算函数
def matrix_add(A, B):
    """
    矩阵加法 A + B
    参数：
    A, B: 两个矩阵，使用嵌套列表表示，维度小于等于二维。
    返回：
    返回矩阵加法结果
    """
    # 确保A和B的维度相同
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("两个矩阵的维度必须相同")

    # 进行逐元素加法
    return [
        [A[i][j] + B[i][j] for j in range(len(A[0]))]  # 遍历每一行，每行的每个元素
        for i in range(len(A))  # 遍历每一行
    ]

def matrix_sub(A, B):
    """
    任意尺寸矩阵减法 A - B
    参数：
    A, B: 两个矩阵，使用嵌套列表表示，维度小于等于二维。
    返回：
    返回矩阵减法结果
    """
    # 确保A和B的维度相同
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("两个矩阵的维度必须相同")

    # 进行逐元素减法
    return [
        [A[i][j] - B[i][j] for j in range(len(A[0]))]  # 遍历每一行，每行的每个元素
        for i in range(len(A))  # 遍历每一行
    ]


def matrix_scale(A, scalar):
    """
    2x2矩阵数乘 A * scalar
    参数：
    A: 2x2 矩阵
    scalar: 标量
    返回：
    返回数乘后的矩阵
    """
    return [
        [A[0][0] * scalar, A[0][1] * scalar],
        [A[1][0] * scalar, A[1][1] * scalar]
    ]

def matrix_multiply(A, B):
    """
    执行矩阵乘法 A * B。
    参数：
    A, B: 任意维度的矩阵，假设 A 的列数等于 B 的行数。
    
    返回：
    返回 A * B 的结果，仍然是一个矩阵。
    """
    # 获取 A 和 B 的行列数
    rows_A = len(A)
    cols_A = len(A[0])
    rows_B = len(B)
    cols_B = len(B[0])
    
    # 确保 A 的列数与 B 的行数相同
    if cols_A != rows_B:
        raise ValueError("矩阵 A 的列数必须等于矩阵 B 的行数")
    
    # 创建一个结果矩阵，大小为 A 的行数 x B 的列数
    result = [[0] * cols_B for _ in range(rows_A)]
    
    # 计算矩阵乘法
    for i in range(rows_A):
        for j in range(cols_B):
            for k in range(cols_A):  # 或者 k in range(rows_B)，因为 cols_A == rows_B
                result[i][j] += A[i][k] * B[k][j]
    
    return result


def matrix_transpose(A):
    """
    2x2矩阵转置 A^T
    参数：
    A: 2x2 矩阵
    返回：
    返回矩阵 A 的转置
    """
    return [
        [A[0][0], A[1][0]],
        [A[0][1], A[1][1]]
    ]

def matrix_inverse(A):
    """
    求 2x2 矩阵 A 的逆
    参数：
    A: 2x2 矩阵
    返回：
    返回 A 的逆矩阵
    """
    det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    
    if det == 0:
        raise ValueError("矩阵不可逆，行列式为0")
    
    inv_det = 1.0 / det  # 计算行列式的倒数
    return [
        [A[1][1] * inv_det, -A[0][1] * inv_det],
        [-A[1][0] * inv_det, A[0][0] * inv_det]
    ]

def identity_matrix():
    """
    返回一个 2x2 单位矩阵
    返回：
    返回 2x2 单位矩阵 [[1, 0], [0, 1]]
    """
    return [[1, 0], [0, 1]]

def covariance_matrix(X, Y):
    """
    计算 X 和 Y 之间的协方差矩阵。
    X 和 Y 是一组样本数据。
    参数：
    X: 一个包含 n 个元素的列表，表示 X 变量的样本数据。
    Y: 一个包含 n 个元素的列表，表示 Y 变量的样本数据。
    
    返回：
    返回一个 2x2 协方差矩阵，形式如下：
    [[Var(X), Cov(X,Y)],
     [Cov(X,Y), Var(Y)]]
    """
    # 样本数
    N = len(X)
    
    # 计算均值
    mean_X = sum(X) / N
    mean_Y = sum(Y) / N
    
    # 计算协方差和方差
    cov_XY = sum((X[i] - mean_X) * (Y[i] - mean_Y) for i in range(N)) / N
    var_X = sum((X[i] - mean_X) ** 2 for i in range(N)) / N
    var_Y = sum((Y[i] - mean_Y) ** 2 for i in range(N)) / N
    
    # 返回 2x2 协方差矩阵
    return [
        [var_X, cov_XY],
        [cov_XY, var_Y]
    ]

#卡尔曼滤波

#卡尔曼滤波参数
x = [[0],[0]]
p = [[550,0],[0,550]]
F = [[1,0.001],[0,1]]
H = [[1,0],[0,1]]
Q = [[1e-4,0],[0,1e-3]]
R = [[0.01,0],[0,0.25]]

#卡尔曼滤波函数
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
    
    x_pred = matrix_multiply(F, x)  # x_pred = F * x
    # 预测新的协方差
    p_pred = matrix_multiply(F, matrix_multiply(p, matrix_transpose(F)))  # p_pred = F * P * F^T
    p_pred = matrix_add(p_pred, Q)  # p_pred = p_pred + Q

    # 更新步骤
    # 计算卡尔曼增益
    H_p_pred = matrix_multiply(H, p_pred)  # H * p_pred
    H_p_pred_HT = matrix_multiply(H_p_pred, matrix_transpose(H))  # H * p_pred * H^T
    S = matrix_add(H_p_pred_HT, R)  # S = H * p_pred * H^T + R
    S_inv = matrix_inverse(S)  # S_inv = (S)^-1
    K = matrix_multiply(p_pred, matrix_multiply(matrix_transpose(H), S_inv))  # K = p_pred * H^T * S_inv

    # 更新状态估计
    y = matrix_sub(z, matrix_multiply(H, x_pred))  # y = z - H * x_pred
    x_est = matrix_add(x_pred, matrix_multiply(K, y))  # x_est = x_pred + K * y

    # 更新协方差矩阵
    I = identity_matrix()  # 单位矩阵
    KH = matrix_multiply(K, H)  # K * H
    p_est = matrix_sub(p_pred, matrix_multiply(KH, p_pred))  # p_est = p_pred - K * H * p_pred

    return x_est, p_est