uint8 joint_fsm_mode    # 0=ZEROTAU 1=CALI 2=DAMPING 3=RESET 4=READY

float32 position        # 线位移 (m)
float32 velocity        # 线速度 (m/s)
float32 yaw_speed       # 机体yaw轴角速度 (rad/s)
float32 roll_euler      # 机体roll轴角度 (rad)
float32[2] leg_lengths       # 腿长 (m)
float32[2] foot_jump_forces  # 跳跃前馈力