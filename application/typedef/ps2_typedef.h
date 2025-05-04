#ifndef PS2_TYPEDEF_H__
#define PS2_TYPEDEF_H__

typedef struct
{
    union 
    {
        struct{
            uint8_t reserved[2];
            uint8_t select:1;
            uint8_t l3:1;
            uint8_t r3:1;
            uint8_t start:1;
            uint8_t up:1;
            uint8_t right:1;
            uint8_t down:1;
            uint8_t left:1;

            uint8_t l2:1;
            uint8_t r2:1;
            uint8_t l1:1;
            uint8_t r1:1;
            uint8_t triangle:1;
            uint8_t circle:1;
            uint8_t cross:1;
            uint8_t square:1;

            uint8_t rx;
            uint8_t ry;
            uint8_t lx;
            uint8_t ly;
        }val;
        struct
        {
            uint8_t data[9];  //数据
        }raw;
    }ps2;
} Ps2Data_t;

#endif  // PS2_TYPEDEF_H__
/*------------------------------ End of File ------------------------------*/
