# ========== 状态量 ==========
float32 position      # x[0] 位置 (k|k)
float32 velocity      # x[1] 速度 (k|k)

# ========== 协方差对角元素 ==========
float32 p_pos_pos     # P[0,0] 位置方差
float32 p_vel_vel     # P[1,1] 速度方差
float32 p_pos_vel     # P[0,1] 交叉项

# ========== 量测相关 ==========
float32 z_raw_0       # measured_vector[0] 原始量测 (速度项)
float32 z_raw_1       # measured_vector[1] 原始量测 （加速度项）

float32 innovation_0  # 新息 z - H*x（量测残差）
float32 innovation_1

# ========== 卡尔曼增益对角 ==========
float32 k_pos         # K[0,*] 对位置的增益
float32 k_vel         # K[1,*] 对速度的增益

# ========== 标志位 ==========
bool use_auto_adjustment
uint8 valid_measurement_num   # valid_z_num_