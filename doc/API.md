# 模块接口标准

version: v1.0.0
> 如代码中的内容与本标准冲突，基于本标准修改代码。

## 校准模块（CALIBRATE）

## 裁判系统模块（REFEREE）

- ```ModelStatus_e GetStatus()```
  > 获取裁判系统状态。
- ```float GetCmdJointPos(uint8_t joint)```
  > （自定义控制器链路）获取关节位置。

## 遥控器模块（REMOTE_CONTROL）

- ```ModelStatus_e GetRcStatus()```
  > 获取遥控器状态。
- ```float GetRcCh(uint8_t channel)```
  > [-1,1] 获取遥控器通道值。\
  > 输入：通道id
- ```char GetRcSw(uint8_t channel)```
  > 获取遥控器拨杆状态。\
  > 输入：通道id
- ```float GetMouse(uint8_t axis)```
  > 获取鼠标axis轴的移动速度
- ```bool GetMouseKey(uint8_t key)```
  > 获取鼠标按键信息
- ```bool GetKeyboard(uint8_t key)```
  > 获取键盘按键信息

## 通信模块（COMMUNICATION）

- ```void ModifyDebugData(uint8_t index, float data, const char * name)```
  > 修改调试数据包。
- ```ModelStatus_e GetStatus()```
  > 获取usb通信状态。
- ```float GetCmdSpeedVx()```
  > (m/s) 获取控制指令：底盘坐标系下x方向运动速度
- ```float GetCmdSpeedVy()```
  > (m/s) 获取控制指令：底盘坐标系下y方向运动速度
- ```float GetCmdSpeedWz()```
  > (rad/s) 获取控制指令：底盘坐标系下自转速度
- ```float GetCmdChassisAngle(uint8_t axis)```
  > (rad) 获取控制指令：底盘欧拉角
- ```float GetCmdChassisLegLength()```
  > (m) 获取控制指令：底盘腿长
- ```float GetCmdGimbalAngle(uint8_t axis)```
  > (rad) 获取控制指令：云台欧拉角
- ```bool GetCmdFire()```
  > 获取控制指令：开火
- ```bool GetCmdFricOn()```
  > 获取控制指令：启动摩擦轮
- ```float GetVirtualRcCh(uint8_t channel)```
  > [-1,1] 获取虚拟遥控器通道值。\
  > 输入：通道id
- ```char GetVirtualRcSw(uint8_t channel)```
  > 获取虚拟遥控器拨杆状态。\
  > 输入：通道id

## IMU模块（IMU）

- ```void CaliGyro(fp32 cali_scale[3], fp32 cali_offset[3], uint32_t * time_count)```
  > 校准陀螺仪
- ```void CaliSetGyro(fp32 cali_scale[3], fp32 cali_offset[3])```
  > 校准陀螺仪设置，将从flash或者其他地方传入校准值
- ```ModelStatus_e GetStatus()```
  > 获取IMU状态。
- ```float GetAngle(uint8_t axis)```
  > (rad) 获取欧拉角
- ```float GetVelocity(uint8_t axis)```
  > (rad/s) 获取角速度
- ```float GetAccel(uint8_t axis)```
  > (m/s^2) 获取角速度
- ```float GetQuaternion(uint8_t axis)```
  > 获取四元数

## 音乐模块（MUSIC）

- ```void SwitchMusic(uint8_t music_id)```
  > 切换歌曲

## 底盘模块（CHASSIS）

- ```void SetCali()```
  > 底盘校准设置
- ```void CmdCali()```
  > 底盘校准计算
- ```ModelStatus_e GetStatus()```
  > 获取底盘状态。
- ```uint32_t GetDuration()```
  > (ms) 获取底盘模块运行周期
- ```float GetSpeedVx()```
  > (m/s) 获取底盘坐标系下x方向运动速度
- ```float GetSpeedVy()```
  > (m/s) 获取底盘坐标系下y方向运动速度
- ```float GetSpeedWz()```
  > (rad/s) 获取底盘坐标系下自转速度
<!-- - ```float GetRefSpeedVx()```
  > (m/s) 获取底盘坐标系下x方向期望运动速度
- ```float GetRefSpeedVy()```
  > (m/s) 获取底盘坐标系下y方向期望运动速度
- ```float GetRefSpeedWz()```
  > (rad/s) 获取底盘坐标系下期望自转速度 -->

## 云台模块（GIMBAL）

- ```void SetCali()```
  > 云台校准设置
- ```void CmdCali()```
  > 云台校准计算
- ```ModelStatus_e GetStatus()```
  > 获取云台状态。
- ```uint32_t GetDuration()```
  > (ms) 获取云台模块运行周期
- ```float GetDeltaYawMid()```
  > (rad) 获取yaw轴和中值的差值

## 射击模块（SHOOT）

- ```ModelStatus_e GetStatus()```
  > 获取射击模块状态。
- ```uint32_t GetDuration()```
  > (ms) 获取射击模块运行周期

## 机械臂模块（MECHANICAL_ARM）

- ```void SetCali()```
  > 机械臂校准设置
- ```void CmdCali()```
  > 机械臂校准计算
- ```ModelStatus_e GetStatus()```
  > 获取机械臂状态。
- ```uint32_t GetDuration()```
  > (ms) 获取机械臂模块运行周期

## 自定义控制器模块（CUSTOM_CONTROLLER）

- ```ModelStatus_e GetStatus()```
  > 获取自定义控制器状态。
- ```uint32_t GetDuration()```
  > (ms) 获取自定义控制器模块运行周期
