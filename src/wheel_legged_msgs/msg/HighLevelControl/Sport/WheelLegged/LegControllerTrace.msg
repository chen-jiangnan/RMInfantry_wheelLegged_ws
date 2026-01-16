PidControllerTrace[2] spring_damping  #vmc弹簧阻尼控制器
float32[2] feedforward_gravity        #抵消机体自重的力源
float32[2] feedforward_fs             #机体跳跃的额外力源
float32[2] f                          #总输出 F = springDamping.out + compensationRoll.out + feedforward_gravity + feedforward_fs
float32[2] compensation_tp

PidControllerTrace compensation_roll   #机体roll角补偿控制器
PidControllerTrace compensation_phi0   #theta补偿控制器  