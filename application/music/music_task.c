

#include "music_task.h"

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "motor.h"
#include "music.h"
#include "music_calibrate.h"
#include "music_canon.h"
#include "music_castle_in_the_sky.h"
#include "music_deja_vu.h"
#include "music_error.h"
#include "music_gong_xi_fa_cai.h"
#include "music_hao_yun_lai.h"
#include "music_meow.h"
#include "music_motor_offline.h"
#include "music_referee.h"
#include "music_see_you_again.h"
#include "music_start.h"
#include "music_typedef.h"
#include "music_unity.h"
#include "music_you.h"
#include "stm32f4xx_hal.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t music_high_water;
#endif

#define STEP_INIT 1
#define STEP_NORMAL 2

#define is_play_cali()                                                                 \
    (is_play == CALI_BEGIN || is_play == CALI_MIDDLE_TIME || is_play == CALI_GIMBAL || \
     is_play == CALI_IMU || is_play == CALI_CHASSIS)

// Enum Declarations，根据优先级排列，越后面优先级越高
typedef enum {
    PLAY_NONE = 0,
    POWER_UP,
    CALI_BEGIN,
    CALI_MIDDLE_TIME,
    CALI_GIMBAL,
    CALI_IMU,
    CALI_CHASSIS,
    PLAY_MOTOR_OFFLINE,
} Playing_e;

typedef enum {
    canon = 0,
    castle_in_the_sky,
    deja_vu,
    error,
    gong_xi_fa_cai,
    hao_yun_lai,
    meow,
    motor_offline,
    referee,
    see_you_again,
    start,
    unity,
    you,
} MusicIndex_e;

// Variable Declarations
static uint8_t music_step = STEP_INIT;

Playing_e is_play = POWER_UP;

static MusicInfo_s MUSICS[20];
static uint32_t now = 0;
static uint32_t start_time = 0;
static uint32_t play_id = 0;  // 0为保留音符，不使用

static uint32_t task_count = 0;

/**
 * @brief 播放音乐
 * @param  none
 * @return 结束1 未结束0
 */
bool PlayMusic(MusicInfo_s * music_info, float volume)
{
    now = HAL_GetTick();
    bool end = false;

    if (play_id == 0) {
        start_time = now;
        play_id++;
        buzzer_note(music_info->notes[play_id].note, volume);
    }
    if (now - start_time >= music_info->notes[play_id].end) {
        play_id++;
        if (play_id > music_info->last_note_id) {
            end = true;
            play_id = 0;
            buzzer_note(0, 0.1);
        } else {
            buzzer_note(music_info->notes[play_id].note, volume);
        }
    }

    return end;
}

/*******************************************************************************/
/* Main Functions                                                              */
/*     music_task                                                              */
/*     MusicInit                                                               */
/*     MusicPlay                                                               */
/*******************************************************************************/

static void MusicInit(void);
static void MusicPlay(void);

void music_task(void const * pvParameters)
{
    // 初始化音乐
    MusicInit();

    while (1) {
        task_count++;
        MusicPlay();
        // 系统延时
        vTaskDelay(MUSIC_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        music_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void MusicInit(void)
{
    // cali_buzzer_state = Subscribe(CALI_BUZZER_STATE_NAME);

    music_step = STEP_INIT;

    // clang-format off
    MUSICS[start]             = MusicStartInit();
    // MUSICS[referee] = MusicRefereeInit();
    // MUSICS[error] = MusicErrorInit();
    MUSICS[motor_offline]     = MusicMotorOfflineInit();
    MUSICS[you]               = MusicYouInit();
    MUSICS[unity]             = MusicUnityInit();
    MUSICS[canon]             = MusicCanonInit();
    MUSICS[castle_in_the_sky] = MusicCastleInTheSkyInit();
    // MUSICS[see_you_again]     = MusicSeeYouAgainInit();
    // MUSICS[hao_yun_lai]       = MusicHaoYunLaiInit();
    // MUSICS[meow]              = MusicMeowInit();
    // MUSICS[gong_xi_fa_cai]    = MusicGongXiFaCaiInit();
    // MUSICS[deja_vu]           = MusicDejaVuInit();
    // clang-format on
}

static void MusicPlay(void)
{
    if (music_step == STEP_INIT) {  // 开机
        if (PlayMusic(&MUSICS[start], 0.5f)) {
            music_step = STEP_NORMAL;
            is_play = PLAY_NONE;
        }
    } else {                           // 正常状态
        if (task_count % 2000 == 0) {  // 检测是否存在离线电机
            if (ScanOfflineMotor()) {
                // is_play = PLAY_MOTOR_OFFLINE;
            }
        }

        // 根据is_play播放对应音乐
        switch (is_play) {
            case PLAY_MOTOR_OFFLINE: {
                if (PlayMusic(&MUSICS[motor_offline], 0.5f)) is_play = PLAY_NONE;
            } break;

            default: {
                PlayMusic(&MUSICS[castle_in_the_sky], 0.1f);
            } break;
        }
    }
}
