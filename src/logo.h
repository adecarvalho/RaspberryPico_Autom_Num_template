#ifndef LOGO_IUT_H
#define LOGO_IUT_H

#include <Arduino.h>

const unsigned char bitmap_logo_iut[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3f, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x70, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc0, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1c, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x78, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xf8, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0xf0, 0x1f, 0xf0, 0xfc, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0xf0, 0x7f, 0xf8, 0xfe, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xe1, 0xff, 0xf8, 0xfe, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xe3, 0xff, 0xf8, 0x7e, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xe7, 0xff, 0xfc, 0x7e, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xef, 0xff, 0xfc, 0x7f, 0x0f, 0x83, 0x0c, 0x01, 0xcf, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xef, 0xff, 0xfc, 0x7f, 0x0f, 0x83, 0x0c, 0x01, 0xcf, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xff, 0xf0, 0x00, 0x3f, 0x1f, 0x83, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xff, 0xe0, 0x00, 0x3f, 0xbf, 0x83, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xc0, 0x00, 0x3f, 0xbf, 0x83, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x00, 0x00, 0x1f, 0xfe, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x00, 0x00, 0x1f, 0xfe, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x00, 0x00, 0x0f, 0xfc, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x80, 0x00, 0x0f, 0xf8, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x80, 0x00, 0x0f, 0xf8, 0x03, 0x0c, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0x80, 0x00, 0x0f, 0xf0, 0x03, 0x0c, 0x01, 0x80, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xc0, 0x00, 0x1f, 0xe0, 0x03, 0x0e, 0x01, 0x80, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xe0, 0x00, 0x1f, 0xc0, 0x03, 0x07, 0x03, 0x80, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xff, 0xfe, 0x00, 0x3f, 0xc0, 0x03, 0x07, 0xcf, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1f, 0xff, 0xff, 0xfc, 0x7f, 0x80, 0x03, 0x03, 0xfe, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xff, 0xff, 0xf8, 0x7f, 0x00, 0x03, 0x00, 0xf8, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xff, 0xff, 0xf0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xff, 0xff, 0xf1, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0xff, 0xff, 0xe1, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0xff, 0xbf, 0xc3, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7f, 0xf8, 0x00, 0x00, 0x01, 0x01, 0xef, 0xbc, 0x7a, 0x23, 0x8f, 0x22, 0xfb, 0xc9, 0xcf,
    0x00, 0x7f, 0xfe, 0x00, 0x00, 0x07, 0x01, 0x0c, 0x22, 0x8b, 0x46, 0x59, 0xb2, 0x26, 0x6b, 0x08,
    0x00, 0x3f, 0xff, 0x80, 0x00, 0x1f, 0x02, 0x0c, 0x22, 0x81, 0xc6, 0x50, 0xba, 0x24, 0x2b, 0x08,
    0x00, 0x1f, 0xff, 0xf0, 0x00, 0xff, 0x02, 0x0f, 0xbc, 0xb9, 0xbf, 0xd0, 0xaa, 0x24, 0x29, 0xcf,
    0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x03, 0x0c, 0x2c, 0x89, 0x06, 0x10, 0xa6, 0x24, 0x28, 0x28,
    0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xa6, 0xda, 0x06, 0x1f, 0x26, 0x23, 0xcb, 0xef,
    0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0x00, 0xc7, 0xa2, 0x72, 0x06, 0x06, 0x22, 0x21, 0x89, 0xcf,
    0x00, 0x01, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//
//
#endif