
#include "music_motor_offline.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

#define NOTE_ON 350
#define NOTE_OFF 0

#define NOTE_LONE 50
#define SLEEP_LONE 50

#define NOTE_NUM 10
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicMotorOfflineInit(void)
{
    MUSIC_INFO.notes = Notes;

    // 滴-滴-滴-
    WRITE_NOTE(NOTE_ON, NOTE_LONE);
    WRITE_NOTE(NOTE_OFF, SLEEP_LONE);

    WRITE_NOTE(NOTE_ON, NOTE_LONE);
    WRITE_NOTE(NOTE_OFF, SLEEP_LONE);

    WRITE_NOTE(NOTE_ON, NOTE_LONE);
    
    return MUSIC_INFO;
}
