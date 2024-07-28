#ifndef MUSIC_CALIBRATE_H
#define MUSIC_CALIBRATE_H

#define cali_buzzer_begin() buzzer_on(200, 10000)  // 蜂鸣器的设置频率和强度
#define cali_buzzer_middle() buzzer_on(60, 10000)  // 蜂鸣器的设置频率和强度
#define cali_buzzer_imu() buzzer_on(90, 10000)  // 当imu在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_gimbal() buzzer_on(30, 19999)  // 当云台在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_chassis() buzzer_on(150, 19999)  // 当云台在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_off() buzzer_off()               // buzzer off，关闭蜂鸣器

#endif  // MUSIC_CALIBRATE_H
