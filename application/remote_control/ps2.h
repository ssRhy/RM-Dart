#ifndef PS2_H__
#define PS2_H__

typedef enum {
    PS2_SELECT = 0,
    PS2_L3,
    PS2_R3,
    PS2_START,
    PS2_UP,
    PS2_RIGHT,
    PS2_DOWN,
    PS2_LEFT,
    PS2_L2,
    PS2_R2,
    PS2_L1,
    PS2_R1,
    PS2_TRIANGLE,
    PS2_CIRCLE,
    PS2_CROSS,
    PS2_SQUARE
} Ps2Key_e;

typedef enum {
    PS2_RX = 0,  // 右正
    PS2_RY,      // 下正
    PS2_LX,
    PS2_LY,

} Ps2Joystick_e;

typedef enum {
    PS2_ERROR = 0,  // 工作异常
    PS2_OK,         // 工作正常
    PS2_CONFIG,     // 配置中
} Ps2Status_e;

extern float GetPs2Joystick(Ps2Joystick_e joystick);
extern bool GetPs2Key(Ps2Key_e key);

#endif  // PS2_H__
/*------------------------------ End of File ------------------------------*/
