#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_VERSION "0.0.3"
#define AKD1500_ENABLE_SIMPLE_CONV_V2 0
#define AKD1500_ENABLE_PRESENCE_CLASSIFIER 1

#if AKD1500_PRESENCE_APP_MODE == AKD1500_PRESENCE_APP_MODE_FLASH_MODEL
#define PRESENCE_CLASSIFIER_EMBED_FULL_PROGRAM 1
#define PRESENCE_CLASSIFIER_EMBED_INPUT_IMAGE 0
#elif AKD1500_PRESENCE_APP_MODE == AKD1500_PRESENCE_APP_MODE_RUN_EXTERNAL
#define PRESENCE_CLASSIFIER_EMBED_FULL_PROGRAM 0
#define PRESENCE_CLASSIFIER_EMBED_INPUT_IMAGE 1
#else
#error "Unsupported AKD1500_PRESENCE_APP_MODE"
#endif

void APP_Initialize(void);
void APP_Tasks(void);
void APP_BlinkLed(int n_times);
bool APP_ConfigureSpi5Mode0(void);

#ifdef __cplusplus
}
#endif
