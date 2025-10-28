#pid控制

def v_pid(pur, data, kp, ki, inte, v_limit):
    '''
    PID 控制直立
    
    参数:
    pur : 目标速度
    data : 当前速度
    kp : 比例系数
    ki : 积分系数
    inte :积分累计
    v_limit :积分上限
    
    返回:
    integral: 积分累计
    out : 目标角度
    '''
    in_now = pur - data
    integral = inte + in_now
    
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out 


def theita_pid(pur, data, kp, kd, in_last):
    '''
    PID 控制theita
    
    参数:
    pur : 目标速度
    data : 当前角度
    kp : 比例系数
    kd : 微分系数
    in_last : 上一时刻的控制输入
    
    返回:
    in_now : 当前时刻的控制输入，用于下一次调用
    out : 目标角速度
    '''
    in_now = pur - data
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out


def w_y_pid(pur, data, kp, ki, inte ,w_y_limit):
    '''
    PID 控制w_y
    
    参数:
    pur : 目标角速度
    data : 当前角速度
    kp : 比例系数
    ki : 积分系数
    inte :积分累计
    w_y_limit :积分上限
    
    返回:
    integral : 积分累计
    out : 目标PWM的变化量
    '''
    in_now = pur - data
    integral = inte + in_now
    
    if integral > w_y_limit:
        integral = w_y_limit
    elif integral < -w_y_limit:
        integral = -w_y_limit
        
    out = kp * in_now + ki * integral
    
    if out > 10000:
        out = 10000
    elif out < -10000:
        out = -10000
    
    return integral, out

def dis_pid(pur, data, kp, kd, in_last):
    '''
    PID 控制dis
    
    参数:
    pur : 道路线
    data : CCD中点
    kp : 比例系数
    kd : 微分系数
    in_last : 上一时刻的控制输入
    
    返回:
    in_now : 当前时刻的控制输入，用于下一次调用
    out : 目标角速度
    '''
    in_now = pur - data
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out


def w_z_pid(pur, data, kp, ki, inte ,w_z_limit):
    '''
    PID 控制w_z
    
    参数:
    pur : 目标角速度
    data : 当前角速度
    kp : 比例系数
    ki : 积分系数
    inte :积分累计
    w_y_limit :积分上限
    
    返回:
    integral : 积分累计
    out : 目标PWM的变化量
    '''
    in_now = pur - data
    integral = inte + in_now
    
    if integral > w_y_limit:
        integral = w_y_limit
    elif integral < -w_y_limit:
        integral = -w_y_limit
        
    out = kp * in_now + ki * integral
    
    if out > 10000:
        out = 10000
    elif out < -10000:
        out = -10000
    
    return integral, out