#ifndef OLLO_FONTS_H
#define OLLO_FONTS_H

#include <lvgl.h>

/* Atkinson Hyperlegible Bold, ASCII 0x20-0x7E, 4 bpp (generated with lv_font_conv).
 * A heavy, open-letterform face survives the 1-bit threshold much better than thin Montserrat. */
#ifdef __cplusplus
extern "C" {
#endif
extern const lv_font_t ollo_font_28;
extern const lv_font_t ollo_font_32;
extern const lv_font_t ollo_font_40;
#ifdef __cplusplus
}
#endif

#endif
