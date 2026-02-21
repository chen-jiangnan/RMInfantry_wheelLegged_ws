KalmanFilterTrace kalman_filter  # 速度加速度 卡尔曼滤波器

float32[2] dot_xw   # 驱动轮相对于世界的平动速度
float32 aver_vel    # 左右驱动轮的平动速度
    
float32 x_filter    # 滤波位移
float32 v_filter    # 滤波速度

float32[2] slip_signal # 打滑检测信号