# 关节模式枚举
uint8 MODE_IDLE = 0              #关节失能
uint8 MODE_POSITION = 1          #位置控制
uint8 MODE_VELOCITY = 2          #速度控制
uint8 MODE_EFFORT = 3            #力矩控制
uint8 MODE_MIT = 4               #力矩控制

string name   # 关节名称
uint8 mode      # 关节模式

# 关节状态
float32 q       #位置 
float32 dq      #速度
float32 tau     #力矩