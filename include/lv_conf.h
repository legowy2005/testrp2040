#ifndef LV_CONF_H
#define LV_CONF_H

/* LVGL 9.6 configuration for the OllO ESP32-S3 UI renderer.
 * LVGL renders into small RGB565 strips; the ESP32 converts them to the 8-bit palette
 * framebuffer that is streamed to the RP2040 (see ollo_frame.h). */
#define LV_COLOR_DEPTH 16

#define LV_USE_STDLIB_MALLOC LV_STDLIB_CLIB
#define LV_USE_STDLIB_STRING LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_CLIB
#define LV_USE_OS LV_OS_NONE

#define LV_USE_DRAW_SW 1

/* Anti-aliased Montserrat: 14 = folder name, 16 = image caption, 20 = card text */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_USE_LOG 0
#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0
#define LV_USE_OBSERVER 0
#define LV_DEF_REFR_PERIOD 33
#define LV_DRAW_BUF_STRIDE_ALIGN 1
#define LV_DRAW_BUF_ALIGN 4

#endif /* LV_CONF_H */
