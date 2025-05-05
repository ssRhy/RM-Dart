#ifndef PS2_H__
#define PS2_H__

typedef enum
{
    PS2_LX = 0,
    PS2_LY,
    PS2_RX,
    PS2_RY,
    PS2_L1,
    PS2_L2,
    PS2_L3,
    PS2_R1,
    PS2_R2,
    PS2_R3,
    PS2_SELECT,
    PS2_START,
    PS2_UP,
    PS2_DOWN,
    PS2_LEFT,
    PS2_RIGHT,
    PS2_TRIANGLE,
    PS2_CIRCLE,
    PS2_CROSS,
    PS2_SQUARE
} Ps2Key_e;

extern float GetPs2Axis(Ps2Key_e key);
extern bool GetPs2Key(Ps2Key_e key);


#endif  // PS2_H__
/*------------------------------ End of File ------------------------------*/
