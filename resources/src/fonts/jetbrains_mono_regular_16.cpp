/*******************************************************************************
 * Size: 16 px
 * Bpp: 1
 * Opts: --bpp 1 --size 16 --no-compress --font JetBrainsMono-Regular.ttf --range 32-127 --format lvgl -o jetbrains_mono_regular_16.c
 ******************************************************************************/

#include "librsc/fonts.hpp"
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "liblvgl/lvgl.h"
#endif

#ifndef JETBRAINS_MONO_REGULAR_16
#define JETBRAINS_MONO_REGULAR_16 1
#endif

#if JETBRAINS_MONO_REGULAR_16

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0x55, 0x55, 0xf,

    /* U+0022 "\"" */
    0x8c, 0x63, 0x18, 0x80,

    /* U+0023 "#" */
    0x11, 0x8, 0x84, 0x82, 0x47, 0xf9, 0x10, 0x88,
    0x44, 0xff, 0x12, 0x19, 0x8, 0x80,

    /* U+0024 "$" */
    0x10, 0x20, 0x41, 0xe5, 0x6a, 0x54, 0x38, 0x38,
    0x38, 0x4c, 0x99, 0x2f, 0x8e, 0x8, 0x10,

    /* U+0025 "%" */
    0x60, 0xc8, 0xa4, 0x92, 0x49, 0x43, 0x40, 0x20,
    0x26, 0x24, 0x92, 0x51, 0x30, 0x60,

    /* U+0026 "&" */
    0x1c, 0x11, 0x8, 0x84, 0x1, 0x3, 0xc3, 0x37,
    0x8a, 0xc2, 0x63, 0xb1, 0x4f, 0x30,

    /* U+0027 "'" */
    0xf8,

    /* U+0028 "(" */
    0x0, 0xd8, 0x88, 0x42, 0x10, 0x84, 0x21, 0x4,
    0x30, 0xe1,

    /* U+0029 ")" */
    0x87, 0xc, 0x20, 0x84, 0x21, 0x8, 0x42, 0x11,
    0x99, 0x98,

    /* U+002A "*" */
    0x8, 0x4, 0x12, 0x5f, 0xf1, 0xc0, 0xa0, 0xd8,
    0x44,

    /* U+002B "+" */
    0x10, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10,

    /* U+002C "," */
    0x6d, 0x2c,

    /* U+002D "-" */
    0xf8,

    /* U+002E "." */
    0xfc,

    /* U+002F "/" */
    0x2, 0xc, 0x10, 0x20, 0xc1, 0x6, 0xc, 0x10,
    0x60, 0x81, 0x6, 0x8, 0x30, 0x0,

    /* U+0030 "0" */
    0x7d, 0x8e, 0xc, 0x18, 0x33, 0x66, 0xc1, 0x83,
    0x7, 0x13, 0xc0,

    /* U+0031 "1" */
    0x30, 0xe3, 0x40, 0x81, 0x2, 0x4, 0x8, 0x10,
    0x20, 0x47, 0xf0,

    /* U+0032 "2" */
    0x79, 0x8a, 0x8, 0x10, 0x20, 0x83, 0xc, 0x30,
    0xc3, 0x7, 0xf0,

    /* U+0033 "3" */
    0x7e, 0xc, 0x30, 0xc3, 0x87, 0x80, 0x81, 0x3,
    0x7, 0x1b, 0xc0,

    /* U+0034 "4" */
    0xc, 0x10, 0x61, 0x82, 0x8, 0x70, 0xc1, 0xfe,
    0x4, 0x8, 0x10,

    /* U+0035 "5" */
    0x7e, 0x81, 0x2, 0x4, 0xf, 0x81, 0x81, 0x3,
    0x7, 0x1b, 0xe0,

    /* U+0036 "6" */
    0x18, 0x20, 0xc1, 0x6, 0xf, 0xb1, 0x41, 0x83,
    0x5, 0x11, 0xc0,

    /* U+0037 "7" */
    0xff, 0x82, 0x86, 0x6, 0xc, 0xc, 0x8, 0x18,
    0x10, 0x30, 0x30, 0x20,

    /* U+0038 "8" */
    0x3c, 0xcd, 0xa, 0x16, 0x67, 0x91, 0xc1, 0x83,
    0x5, 0x11, 0xc0,

    /* U+0039 "9" */
    0x38, 0x8a, 0xc, 0x18, 0x38, 0xdf, 0x6, 0x8,
    0x30, 0x41, 0x80,

    /* U+003A ":" */
    0xfc, 0x0, 0x7, 0xe0,

    /* U+003B ";" */
    0xfc, 0x0, 0x3, 0xdb, 0x60,

    /* U+003C "<" */
    0x0, 0xc, 0x73, 0x88, 0x1c, 0xe, 0x7, 0x2,

    /* U+003D "=" */
    0xfe, 0x0, 0x0, 0xf, 0xe0,

    /* U+003E ">" */
    0x1, 0x81, 0xc0, 0xe0, 0x61, 0xce, 0x70, 0x80,

    /* U+003F "?" */
    0xf0, 0x20, 0x41, 0x4, 0x33, 0x88, 0x0, 0x6,
    0x18,

    /* U+0040 "@" */
    0x3c, 0x42, 0xc1, 0x81, 0x8f, 0x91, 0x91, 0x91,
    0x91, 0x91, 0x8e, 0x80, 0x80, 0x40, 0x3c,

    /* U+0041 "A" */
    0x18, 0x38, 0x38, 0x2c, 0x2c, 0x64, 0x64, 0x46,
    0x7e, 0xc2, 0xc2, 0x83,

    /* U+0042 "B" */
    0xfd, 0xe, 0xc, 0x18, 0x7f, 0x21, 0xc1, 0x83,
    0x6, 0x1f, 0xe0,

    /* U+0043 "C" */
    0x7d, 0x8e, 0xc, 0x8, 0x10, 0x20, 0x40, 0x81,
    0x7, 0x1b, 0xe0,

    /* U+0044 "D" */
    0xf9, 0xa, 0xc, 0x18, 0x30, 0x60, 0xc1, 0x83,
    0x6, 0x17, 0xc0,

    /* U+0045 "E" */
    0xfe, 0x8, 0x20, 0x83, 0xf8, 0x20, 0x82, 0x8,
    0x3f,

    /* U+0046 "F" */
    0xfe, 0x8, 0x20, 0x82, 0xf, 0xe0, 0x82, 0x8,
    0x20,

    /* U+0047 "G" */
    0x7d, 0x8e, 0xc, 0x8, 0x10, 0x27, 0xc1, 0x83,
    0x7, 0x1b, 0xe0,

    /* U+0048 "H" */
    0x86, 0x18, 0x61, 0x87, 0xf8, 0x61, 0x86, 0x18,
    0x61,

    /* U+0049 "I" */
    0xfc, 0x41, 0x4, 0x10, 0x41, 0x4, 0x10, 0x41,
    0x3f,

    /* U+004A "J" */
    0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x81, 0x3,
    0x7, 0x13, 0xc0,

    /* U+004B "K" */
    0x87, 0xa, 0x34, 0x49, 0x9e, 0x24, 0x4c, 0x89,
    0x1a, 0x14, 0x30,

    /* U+004C "L" */
    0x82, 0x8, 0x20, 0x82, 0x8, 0x20, 0x82, 0x8,
    0x3f,

    /* U+004D "M" */
    0xc7, 0x8f, 0xad, 0x5a, 0xb6, 0x64, 0xc1, 0x83,
    0x6, 0xc, 0x10,

    /* U+004E "N" */
    0xc3, 0x87, 0x8d, 0x1b, 0x32, 0x64, 0xcd, 0x8b,
    0x1e, 0x1c, 0x30,

    /* U+004F "O" */
    0x7d, 0x8e, 0xc, 0x18, 0x30, 0x60, 0xc1, 0x83,
    0x7, 0x1b, 0xe0,

    /* U+0050 "P" */
    0xfd, 0xa, 0xc, 0x18, 0x30, 0xbe, 0x40, 0x81,
    0x2, 0x4, 0x0,

    /* U+0051 "Q" */
    0x79, 0x8a, 0xc, 0x18, 0x30, 0x60, 0xc1, 0x83,
    0x5, 0x19, 0xe0, 0xc0, 0x80, 0x80,

    /* U+0052 "R" */
    0xf9, 0xa, 0xc, 0x18, 0x30, 0xff, 0x44, 0x8d,
    0xa, 0x1c, 0x10,

    /* U+0053 "S" */
    0x79, 0x8e, 0xc, 0xc, 0xf, 0x7, 0x3, 0x3,
    0x7, 0x1b, 0xc0,

    /* U+0054 "T" */
    0xff, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10,

    /* U+0055 "U" */
    0x83, 0x6, 0xc, 0x18, 0x30, 0x60, 0xc1, 0x83,
    0x7, 0x1b, 0xe0,

    /* U+0056 "V" */
    0x83, 0xc2, 0xc2, 0x46, 0x46, 0x64, 0x64, 0x24,
    0x2c, 0x38, 0x38, 0x18,

    /* U+0057 "W" */
    0x99, 0xcc, 0xe6, 0x73, 0x29, 0xd6, 0xeb, 0xd4,
    0xca, 0x65, 0x33, 0x99, 0xcc, 0xc0,

    /* U+0058 "X" */
    0xc3, 0x46, 0x64, 0x2c, 0x38, 0x18, 0x18, 0x3c,
    0x2c, 0x66, 0xc2, 0xc3,

    /* U+0059 "Y" */
    0x41, 0x31, 0x88, 0x86, 0x41, 0x40, 0xa0, 0x20,
    0x10, 0x8, 0x4, 0x2, 0x1, 0x0,

    /* U+005A "Z" */
    0xfe, 0xc, 0x10, 0x60, 0x83, 0xc, 0x10, 0x60,
    0x83, 0x7, 0xf0,

    /* U+005B "[" */
    0xf2, 0x49, 0x24, 0x92, 0x49, 0x38,

    /* U+005C "\\" */
    0xc0, 0x81, 0x81, 0x2, 0x6, 0x4, 0xc, 0x18,
    0x10, 0x30, 0x20, 0x40, 0xc0, 0x80,

    /* U+005D "]" */
    0xe4, 0x92, 0x49, 0x24, 0x92, 0x78,

    /* U+005E "^" */
    0x10, 0x30, 0xe1, 0x26, 0x48, 0xf0, 0x80,

    /* U+005F "_" */
    0xff,

    /* U+0060 "`" */
    0x99, 0x80,

    /* U+0061 "a" */
    0x3c, 0x8c, 0x8, 0x17, 0xf0, 0x60, 0xc3, 0x7a,

    /* U+0062 "b" */
    0x81, 0x2, 0x5, 0xec, 0x70, 0x60, 0xc1, 0x83,
    0x7, 0x1d, 0xe0,

    /* U+0063 "c" */
    0x3d, 0x8e, 0xc, 0x8, 0x10, 0x20, 0xe3, 0x7c,

    /* U+0064 "d" */
    0x2, 0x4, 0xb, 0xdc, 0x70, 0x60, 0xc1, 0x83,
    0x7, 0x1b, 0xd0,

    /* U+0065 "e" */
    0x39, 0x8a, 0xc, 0x1f, 0xf0, 0x20, 0x63, 0x7c,

    /* U+0066 "f" */
    0xf, 0x10, 0x10, 0x10, 0x10, 0xff, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10,

    /* U+0067 "g" */
    0x77, 0x38, 0x61, 0x86, 0x1c, 0xdd, 0x4, 0x10,
    0x5e,

    /* U+0068 "h" */
    0x82, 0x8, 0x2e, 0xce, 0x18, 0x61, 0x86, 0x18,
    0x61,

    /* U+0069 "i" */
    0x18, 0x18, 0x0, 0x0, 0xf0, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10, 0xff,

    /* U+006A "j" */
    0xc, 0x30, 0x0, 0xfc, 0x10, 0x41, 0x4, 0x10,
    0x41, 0x4, 0x10, 0xfe,

    /* U+006B "k" */
    0x81, 0x2, 0x4, 0x28, 0xd1, 0x26, 0x78, 0x99,
    0x12, 0x34, 0x30,

    /* U+006C "l" */
    0xf0, 0x8, 0x4, 0x2, 0x1, 0x0, 0x80, 0x40,
    0x20, 0x10, 0x8, 0x4, 0x1, 0xf0,

    /* U+006D "m" */
    0xfe, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91,
    0x91,

    /* U+006E "n" */
    0xbb, 0x38, 0x61, 0x86, 0x18, 0x61, 0x84,

    /* U+006F "o" */
    0x7d, 0x8e, 0xc, 0x18, 0x30, 0x60, 0xe3, 0x78,

    /* U+0070 "p" */
    0xbd, 0x8e, 0xc, 0x18, 0x30, 0x60, 0xe3, 0xbd,
    0x2, 0x4, 0x0,

    /* U+0071 "q" */
    0x7b, 0x8e, 0xc, 0x18, 0x30, 0x60, 0xe3, 0x7a,
    0x4, 0x8, 0x10,

    /* U+0072 "r" */
    0xbb, 0x38, 0x61, 0x82, 0x8, 0x20, 0x80,

    /* U+0073 "s" */
    0x7d, 0xe, 0x7, 0x7, 0xc0, 0xc0, 0xc1, 0x7c,

    /* U+0074 "t" */
    0x10, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0xf,

    /* U+0075 "u" */
    0x83, 0x6, 0xc, 0x18, 0x30, 0x60, 0xe3, 0x7c,

    /* U+0076 "v" */
    0xc3, 0xc2, 0x46, 0x66, 0x64, 0x2c, 0x2c, 0x38,
    0x18,

    /* U+0077 "w" */
    0x4c, 0xa6, 0x53, 0x2a, 0xb5, 0x53, 0xa8, 0xcc,
    0x66, 0x33, 0x0,

    /* U+0078 "x" */
    0xc2, 0x66, 0x2c, 0x38, 0x18, 0x38, 0x6c, 0x46,
    0xc2,

    /* U+0079 "y" */
    0xc3, 0xc2, 0x46, 0x64, 0x24, 0x2c, 0x38, 0x18,
    0x18, 0x10, 0x30, 0x30,

    /* U+007A "z" */
    0xfe, 0xc, 0x30, 0xc1, 0x6, 0x18, 0x60, 0xfe,

    /* U+007B "{" */
    0xe, 0x30, 0x60, 0xc1, 0x83, 0x6, 0x70, 0x18,
    0x30, 0x60, 0xc1, 0x83, 0x3, 0x80,

    /* U+007C "|" */
    0xff, 0xfe,

    /* U+007D "}" */
    0xe0, 0x60, 0xc1, 0x82, 0x4, 0x8, 0xf, 0x30,
    0x40, 0x81, 0x83, 0x6, 0x38, 0x0,

    /* U+007E "~" */
    0x73, 0x26, 0x70
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 154, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 154, .box_w = 2, .box_h = 12, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 4, .adv_w = 154, .box_w = 5, .box_h = 5, .ofs_x = 2, .ofs_y = 7},
    {.bitmap_index = 8, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 22, .adv_w = 154, .box_w = 7, .box_h = 17, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 37, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 51, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 65, .adv_w = 154, .box_w = 1, .box_h = 5, .ofs_x = 4, .ofs_y = 7},
    {.bitmap_index = 66, .adv_w = 154, .box_w = 5, .box_h = 16, .ofs_x = 3, .ofs_y = -2},
    {.bitmap_index = 76, .adv_w = 154, .box_w = 5, .box_h = 16, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 86, .adv_w = 154, .box_w = 9, .box_h = 8, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 95, .adv_w = 154, .box_w = 8, .box_h = 7, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 102, .adv_w = 154, .box_w = 3, .box_h = 5, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 104, .adv_w = 154, .box_w = 5, .box_h = 1, .ofs_x = 2, .ofs_y = 5},
    {.bitmap_index = 105, .adv_w = 154, .box_w = 3, .box_h = 2, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 106, .adv_w = 154, .box_w = 7, .box_h = 15, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 120, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 131, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 142, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 153, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 164, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 175, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 186, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 197, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 209, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 220, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 231, .adv_w = 154, .box_w = 3, .box_h = 9, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 235, .adv_w = 154, .box_w = 3, .box_h = 12, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 240, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 248, .adv_w = 154, .box_w = 7, .box_h = 5, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 253, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 261, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 270, .adv_w = 154, .box_w = 8, .box_h = 15, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 285, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 297, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 308, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 319, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 330, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 339, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 348, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 359, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 368, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 377, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 388, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 399, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 408, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 419, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 430, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 441, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 452, .adv_w = 154, .box_w = 7, .box_h = 15, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 466, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 477, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 488, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 500, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 511, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 523, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 537, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 549, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 563, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 574, .adv_w = 154, .box_w = 3, .box_h = 15, .ofs_x = 3, .ofs_y = -2},
    {.bitmap_index = 580, .adv_w = 154, .box_w = 7, .box_h = 15, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 594, .adv_w = 154, .box_w = 3, .box_h = 15, .ofs_x = 3, .ofs_y = -2},
    {.bitmap_index = 600, .adv_w = 154, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 607, .adv_w = 154, .box_w = 8, .box_h = 1, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 608, .adv_w = 154, .box_w = 3, .box_h = 3, .ofs_x = 3, .ofs_y = 10},
    {.bitmap_index = 610, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 629, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 637, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 648, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 656, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 668, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 677, .adv_w = 154, .box_w = 6, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 686, .adv_w = 154, .box_w = 8, .box_h = 13, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 699, .adv_w = 154, .box_w = 6, .box_h = 16, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 711, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 722, .adv_w = 154, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 736, .adv_w = 154, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 745, .adv_w = 154, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 752, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 760, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 771, .adv_w = 154, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 782, .adv_w = 154, .box_w = 6, .box_h = 9, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 789, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 797, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 809, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 817, .adv_w = 154, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 826, .adv_w = 154, .box_w = 9, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 837, .adv_w = 154, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 846, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 858, .adv_w = 154, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 866, .adv_w = 154, .box_w = 7, .box_h = 15, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 880, .adv_w = 154, .box_w = 1, .box_h = 15, .ofs_x = 4, .ofs_y = -2},
    {.bitmap_index = 882, .adv_w = 154, .box_w = 7, .box_h = 15, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 896, .adv_w = 154, .box_w = 7, .box_h = 3, .ofs_x = 1, .ofs_y = 4}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};

extern const lv_font_t lv_font_montserrat_16;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t fonts::jetbrains_mono_regular_16 = {
#else
lv_font_t jetbrains_mono_regular_16 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 18,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -2,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = &lv_font_montserrat_16,
#endif
    .user_data = NULL,
};



#endif /*#if JETBRAINS_MONO_REGULAR_16*/

