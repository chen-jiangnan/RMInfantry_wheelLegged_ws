# 关节模式枚举
uint8 MODE_IDLE = 0              #关节失能
uint8 MODE_POSITION = 1          #位置控制
uint8 MODE_VELOCITY = 2          #速度控制
uint8 MODE_EFFORT = 3            #力矩控制
uint8 MODE_MIT = 4               #MIT控制
uint8 MODE_SET_ZERO = 10         #零点设置

uint8 mode      # 关节模式

# 设定值
float32 q       #位置 
float32 dq      #速度
float32 tau     #力矩
# 参数
float32 kp
float32 kd

