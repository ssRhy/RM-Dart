

#include "music_task.h"

#include "cmsis_os.h"
#include "music.h"
#include "music_canon.h"
#include "music_castle_in_the_sky.h"
#include "music_referee.h"
#include "music_start.h"
#include "music_unity.h"
#include "music_you.h"
#include "music_error.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

#define STEP_INIT 1
#define STEP_NORMAL 2

// Variable Declarations
static uint8_t music_step = STEP_INIT;

static void MusicInit(void);
static void MusicPlay(void);

void music_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 初始化底盘
    MusicInit();

    while (1) {
        MusicPlay();
        // 系统延时
        vTaskDelay(MUSIC_TASK_TIME_MS);
    }
}

static void MusicInit(void)
{
    music_step = STEP_INIT;
    MusicStartInit();
    MusicRefereeInit();
    MusicErrorInit();
    // MusicYouInit();
    // MusicUnityInit();
    // MusicCanonInit();
    // MusicCastleInTheSkyInit();
}

static void MusicPlay(void)
{
    // MusicYouPlay();
    // MusicUnityPlay();
    // MusicCanonPlay();
    // MusicCastleInTheSkyPlay();
    if (music_step == STEP_INIT) {
        if (MusicStartPlay()) {
            music_step = STEP_NORMAL;
        }
    } else {
        // MusicRefereeDisconnectPlay();
        // MusicErrorPlay();
        ;
    }
}
