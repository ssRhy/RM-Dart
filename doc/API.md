# 接口标准

version: v1.0.0
> 如代码中的内容与本标准冲突，基于本标准修改代码。

- [接口标准](#接口标准)
  - [用户库（USER\_LIB）](#用户库user_lib)
  - [校准模块（CALIBRATE）](#校准模块calibrate)
  - [裁判系统模块（REFEREE）](#裁判系统模块referee)
  - [遥控器模块（REMOTE\_CONTROL）](#遥控器模块remote_control)
  - [通信模块（COMMUNICATION）](#通信模块communication)
  - [IMU模块（IMU）](#imu模块imu)
  - [音乐模块（MUSIC）](#音乐模块music)
  - [底盘模块（CHASSIS）](#底盘模块chassis)
  - [云台模块（GIMBAL）](#云台模块gimbal)
  - [射击模块（SHOOT）](#射击模块shoot)
  - [机械臂模块（MECHANICAL\_ARM）](#机械臂模块mechanical_arm)
  - [自定义控制器模块（CUSTOM\_CONTROLLER）](#自定义控制器模块custom_controller)

## 用户库（USER_LIB）

```C
#include "user_lib.h"
```

- `invSqrt`
  > 快速开方

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |num|fp32|待开方数据|
  |返回|fp32|开方结果|

- `abs_limit`
  > 浮点数绝对限制

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |num|fp32 * |被限制数据的地址|
  |Limit|fp32 |最大值|

- `sign`
  > 判断符号位

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |value|fp32|被判断数据|
  |返回|fp32|符号位，负数为-1.0f，正数为1.0f|

- `fp32_deadline`
  > 浮点死区

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Value|fp32|被限制数据|
  |minValue|fp32|死区上限|
  |maxValue|fp32|死区下限|
  |返回|fp32|死区限制结果|

- `int16_deadline`
  > int16死区

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Value|int16_t|被限制数据|
  |minValue|int16_t|死区上限|
  |maxValue|int16_t|死区下限|
  |返回|int16_t|死区限制结果|

- `fp32_constrain`
  > 浮点限幅函数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Value|fp32|被限制数据|
  |minValue|fp32|上限|
  |maxValue|fp32|下限|
  |返回|fp32|限幅结果|

- `int16_constrain`
  > int16限幅函数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Value|int16_t|被限制数据|
  |minValue|int16_t|上限|
  |maxValue|int16_t|下限|
  |返回|int16_t|限幅结果|

- `loop_fp32_constrain`
  > 浮点循环限幅函数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Input|fp32|被限制数据|
  |minValue|fp32|上限|
  |maxValue|fp32|下限|
  |返回|fp32|限幅结果|

- `theta_format`
  > 弧度格式化为-PI~PI

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |Ang|fp32|待格式化弧度数据|
  |返回|fp32|弧度格式结果|

- `theta_transform`
  > 角度在极坐标系中的转换

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |angle|fp32|(rad)当前角度|
  |dangle|fp32|(rad)旋转角度|
  |direction|int8_t|方向，1为正，-1为负|
  |duration|uint8_t|周期（1个周期结果范围为$[-\pi,\pi]$, 2个周期$[-2\pi,2\pi]$, ...以此类推）|
  |返回|fp32|转换后的结果|

- `float_to_uint`
  > 浮点数转换为无符号整数函数。\
  将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |x_float|float|待转换的浮点数|
  |x_min|float|范围最小值|
  |x_max|float|范围最大值|
  |bits|int|目标无符号整数的位数|
  |返回|int|无符号整数结果|

- `uint_to_float`
  > 无符号整数转换为浮点数函数。\
  将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |x_int|int|待转换的无符号整数|
  |x_min|float|范围最小值|
  |x_max|float|范围最大值|
  |bits|int|目标无符号整数的位数|
  |返回|int|浮点数结果|

- `LowPassFilterInit`
  > 低通滤波器初始化

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |filter|LowPassFilter_t *|滤波器结构体地址|
  |alpha|float|平滑系数|

- `LowPassFilterCalc`
  > 低通滤波计算

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |filter|LowPassFilter_t *|滤波器结构体地址|
  |input|float|输入|
  |返回|float|滤波结果|

## 校准模块（CALIBRATE）

```C
#include "calibrate.h"
```

## 裁判系统模块（REFEREE）

```C
#include "referee.h"
```

- `GetRefereeStatus`
  > 获取裁判系统状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|裁判系统状态|

- `GetCmdJointPos`
  > （自定义控制器链路）获取关节位置。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |joint|uint8_t|关节索引，从0开始，分别为j0,j1~j6|
  |返回|float|自定义控制器传入的关节位置|

## 遥控器模块（REMOTE_CONTROL）

```C
#include "remote_control.h"
```

- `switch_is_up`
  > 【宏】判断拨杆是否置于上档

  | 参数 | 备注 |
  |------|-----|
  |s|DT7遥控器拨杆值|

- `switch_is_mid`
  > 【宏】判断拨杆是否置于中档

  | 参数 | 备注 |
  |------|-----|
  |s|DT7遥控器拨杆值|

- `switch_is_down`
  > 【宏】判断拨杆是否置于下档

  | 参数 | 备注 |
  |------|-----|
  |s|DT7遥控器拨杆值|

- `GetRcStatus`
  > 获取遥控器状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|遥控器状态|

- `GetDt7RcCh`
  > 获取DT7遥控器通道值。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |channel|uint8_t|通道id，0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮|
  |返回|float|DT7遥控器通道值，范围为 $[-1,1]$|

- `GetDt7RcSw`
  > 获取DT7遥控器拨杆值，可配合switch_is_xxx系列宏函数使用。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |channel|uint8_t|通道id，0-右, 1-左|
  |返回|char|DT7遥控器拨杆值，范围为 $\{1,2,3\}$|

- `GetMouseSpeed`
  > 获取鼠标axis轴的移动速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id, 0-, 1-, 2-|
  |返回|float|鼠标axis轴，范围为 $[,]$|

- `GetMouse`
  > 获取鼠标按键信息

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |key|uint8_t|按键id，配合定义好的按键id宏进行使用|
  |返回|bool|鼠标按键是否被按下|

- `GetKeyboard`
  > 获取键盘按键信息

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |key|uint8_t|按键id，配合定义好的按键id宏进行使用|
  |返回|bool|键盘按键是否被按下|

## 通信模块（COMMUNICATION）

```C
// 导入通信相关的所有库
#include "communication.h"

// 当然，你也可以按需导入

// 导入usb通信相关的库
#include "usb.h"
```

上位机：Supervisory Computer

- `ModifyDebugData`
  > 修改调试数据包。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |index|uint8_t|数据包索引id|
  |data|float|数据值|
  |name|const char *|数据包名字|

- `GetUsbStatus`
  > 获取usb连接状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|usb连接状态|

- `GetScCmdChassisSpeed`
  > 获取上位机控制指令：底盘坐标系下axis方向运动线速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(m/s) 底盘坐标系下axis方向运动线速度|

- `GetScCmdChassisVelocity`
  > 获取上位机控制指令：底盘坐标系下axis方向运动角速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad/s) 底盘坐标系下axis方向运动角速度|

- `GetScCmdChassisAngle`
  > 获取上位机控制指令：底盘姿态，基于欧拉角 $r \times p \times y$

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad) 底盘姿态|

- `GetScCmdChassisHeight`
  > 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|float|(m) 底盘离地高度|

- `GetScCmdGimbalAngle`
  > 获取上位机控制指令：云台姿态，基于欧拉角 $r \times p \times y$

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad) 云台姿态|

- `GetScCmdFire()`
  > 获取上位机控制指令：开火

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|bool|是否开火|

- `GetScCmdFricOn`
  > 获取控制指令：启动摩擦轮

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|bool|是否启动摩擦轮|

- `GetVirtualRcCh`
  > 获取虚拟遥控器通道值。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |channel|uint8_t|通道id，0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮|
  |返回|float|虚拟遥控器通道值，范围为 $[-1,1]$|

- `GetVirtualRcSw`
  > 获取虚拟遥控器拨杆状态，可配合遥控器模块中的switch_is_xxx系列宏函数使用。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |channel|uint8_t|通道id，0-右, 1-左|
  |返回|char|虚拟遥控器拨杆值，范围为 $\{1,2,3\}$|

## IMU模块（IMU）

```C
#include "IMU.h"
```

- `CaliGyro`
  > 校准陀螺仪

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |cali_scale[3]|fp32|...|
  |cali_offset[3]|fp32|...|
  |time_count|uint32_t *|...|

- `CaliSetGyro`
  > 校准陀螺仪设置，传入校准值以修正陀螺仪测量数据

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |cali_scale[3]|fp32|...|
  |cali_offset[3]|fp32|...|

<!-- - `GetStatus()`
  > 获取IMU状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|IMU状态| -->

- `GetImuAngle`
  > 获取欧拉角

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad) axis轴的角度值|

- `GetImuVelocity`
  > (rad/s) 获取角速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad/s) axis轴的角速度|

- `GetImuAccel`
  > (m/s^2) 获取加速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(m/s^2) axis轴上的加速度|

<!-- - ```float GetImuQuaternion(uint8_t axis)```
  > 获取四元数

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(m/s^2) axis轴上的加速度| -->

## 音乐模块（MUSIC）

```C
#include "music.h"
```

- `SwitchMusic`
  > 切换歌曲

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |music_id|uint8_t|音乐id，可配合定义好的音乐id宏使用|

## 底盘模块（CHASSIS）

- `SetChassisCali`
  > 底盘校准设置，传入校准值以修正底盘的相关数据

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |motor_middle[4]|const fp32|...|

- `CmdChassisCali`
  > 底盘校准计算，计算校准数据并导出储存。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |motor_middle[4]|fp32|...|
  |返回|bool|校准是否完成|

- `GetChassisStatus`
  > 获取底盘状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|底盘状态|

- `GetChassisDuration`
  > 获取底盘模块运行周期

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|uint32_t|(ms) 底盘模块运行周期|

- `GetChassisSpeed`
  > 获取底盘坐标系下axis方向运动线速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(m/s) 底盘坐标系下axis方向运动线速度|

- `GetChassisVelocity`
  > 获取底盘坐标系下axis方向运动角速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad/s) 底盘坐标系下axis方向运动角速度|

<!-- - ```float GetRefSpeedVx()```
  > (m/s) 获取底盘坐标系下x方向期望运动速度
- ```float GetRefSpeedVy()```
  > (m/s) 获取底盘坐标系下y方向期望运动速度
- ```float GetRefSpeedWz()```
  > (rad/s) 获取底盘坐标系下期望自转速度 -->

## 云台模块（GIMBAL）

```C
#include "gimbal.h"
```

- `SetGimbalCali`
  > 云台校准设置，传入校准值以修正底盘的相关数据

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |yaw_middle|const fp32|yaw轴电机 - 云台朝向正方向时的位置值|
  |pitch_horizontal|const fp32|pitch轴电机 - 云台水平时的位置值|
  |pitch_max|const fp32|pitch轴电机 - 云台俯仰上限位置值|
  |pitch_min|const fp32|pitch轴电机 - 云台俯仰下限位置值|

- `CmdGimbalCali`
  > 云台校准计算，计算校准数据并导出储存。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |yaw_middle|fp32 *|yaw轴电机 - 云台朝向正方向时的位置值|
  |pitch_horizontal|fp32 *|pitch轴电机 - 云台水平时的位置值|
  |pitch_max|fp32 *|pitch轴电机 - 云台俯仰上限位置值|
  |pitch_min|fp32 *|pitch轴电机 - 云台俯仰下限位置值|

- `GetGimbalStatus`
  > 获取云台状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|云台状态|

- `GetGimbalDuration`
  > 获取云台模块运行周期

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|uint32_t|(ms) 云台模块运行周期|

- `GetGimbalSpeed`
  > 获取云台坐标系下axis方向运动线速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(m/s) 云台坐标系下axis方向运动线速度|

- `GetGimbalVelocity`
  > 获取云台坐标系下axis方向运动角速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |axis|uint8_t|轴id，可配合定义好的轴id宏使用|
  |返回|float|(rad/s) 云台坐标系下axis方向运动角速度|

- `GetGimbalDeltaYawMid`
  > (rad) 获取yaw轴和中值的差值

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|float|(rad) yaw轴和中值的差值|

## 射击模块（SHOOT）

```C
#include "shoot.h"
```

- `GetShootStatus`
  > 获取射击模块状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|射击模块状态|

- `GetShootDuration`
  > 获取射击模块运行周期

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|uint32_t|(ms) 射击模块运行周期|

## 机械臂模块（MECHANICAL_ARM）

```C
#include "mechanical_arm.h"
```

- `SetMechanicalArmCali`
  > 机械臂校准设置

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  | 0 | 0 | 0 |

- `CmdMechanicalArmCali`
  > 机械臂校准计算，计算校准数据并导出储存。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  | 0 | 0 | 0 |

- `GetMechanicalArmStatus`
  > 获取机械臂状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|机械臂状态|

- `GetMechanicalArmDuration`
  > 获取机械臂模块运行周期

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|uint32_t|(ms) 机械臂模块运行周期|

- `GetMechanicalArmVelocity`
  > 获取机械臂关节运动速度

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |joint_id|uint8_t|关节id|
  |返回|float|(rad/s) 机械臂joint_id关节运动速度|

- `GetMechanicalArmPosition`
  > 获取机械臂关节位置

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |joint_id|uint8_t|关节id|
  |返回|float|(rad/s) 机械臂joint_id关节位置|

## 自定义控制器模块（CUSTOM_CONTROLLER）

```C
#include "custom_controller.h"
```

- `GetCustomControllerStatus`
  > 获取自定义控制器状态。

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|ModelStatus_e|自定义控制器状态|

- `GetCustomControllerDuration`
  > 获取自定义控制器模块运行周期

  | 参数 | 类型 | 备注 |
  |------|------|-----|
  |返回|uint32_t|(ms) 自定义控制器模块运行周期|
