/* -------------- GPIO --------------------------- */
USART 1 -> DBUS        遥控器
USART 2 -> DEBUG       打印接口
USART 3 -> NULL        无
USART 6 -> JUDGE       裁判系统
USART 7 -> MinPc       PC
USART 8 -> Capacity    电容
ADC1    -> PF11        电流采集

/* --------------- timer ------------------------------ */
TIM4 1 2-> PD12 PD13   摩擦轮
TIM8 1  -> PI5         舵机
TIM12 1 -> BUZZY       蜂鸣器

/* --------------- 按键 ------------------------ */
W A S D: 前后左右
shift  : 开启超级电容 快走
ctrl   : 慢走 进行补弹用
Q      : 连发安键
E      : 扭腰+小陀螺
R      ： 自瞄
T      ：神符
F      ：舱门控制
C      ：三连发 单发切换