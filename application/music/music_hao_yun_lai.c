#include "music_hao_yun_lai.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define D_do 294
#define D_re 330
#define D_mi 370
#define D_fa 392
#define D_so 440
#define D_la 494
#define D_si 554
// clang-format on

#define ONE_FOURTH 250
#define ONE_FOURTH_HALF 125
#define ONE_FOURTH_HALF_HALF 63

#define NOTE_NUM 50
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicHaoYunLaiInit(void)
{
    MUSIC_INFO.notes = Notes;

    // do re mi fa so la si
    // 1  2  3  4  5  6  7

    // 好运来祝你
    // 6 3`_ 2`_ 2` 1`_ 6_
    WRITE_NOTE(D_la, ONE_FOURTH);           // 6
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF);  // 3`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH);       // 2
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);      // 6_

    // 好运来，
    // 5 1`_ 2`_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH);           // 5
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);       // 6

    // 好运带来了
    // 6 2` 1` 6_ 5_
    WRITE_NOTE(D_la, ONE_FOURTH);       // 6
    WRITE_NOTE(D_re * 2, ONE_FOURTH);   // 2`
    WRITE_NOTE(D_do * 2, ONE_FOURTH);   // 1`
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_

    // 喜和爱,
    // 2 5_ 6_ 3 -
    WRITE_NOTE(D_re, ONE_FOURTH);       // 2
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_mi, ONE_FOURTH * 2);   // 3

    // 好运来我们
    // 3 6_ 5_ 6 6_ 5_
    WRITE_NOTE(D_mi, ONE_FOURTH);       // 3
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(D_la, ONE_FOURTH);       // 6
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_

    // 好运来，
    // 6 2`_ 1`_ 2` -
    WRITE_NOTE(D_la, ONE_FOURTH);           // 6
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH * 2);   // 2`

    //迎着好运兴旺发达
    // 1`-_ 1`__ 1`_ 2`_ 3`_ 3`_ 2`_ 1`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 1`-_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF_HALF);                    // 1`__
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);                         // 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);                         // 2`_
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF);                         // 3`_
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF);                         // 3`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);                         // 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);                         // 1`_

    // 通四海。
    // 5 1`_ 6_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH);           // 5
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);      // 6_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);       // 6

    // 尾奏

    SLEEP_NOTE(500);

    return MUSIC_INFO;
}
/*------------------------------ End of File ------------------------------*/
