#ifndef __LCDFONT_CLEAN_H
#define __LCDFONT_CLEAN_H

typedef struct
{
    const char *Index;
    unsigned char Msk[24];
} typFNT_GB12;

typedef struct
{
    const char *Index;
    unsigned char Msk[32];
} typFNT_GB16;

typedef struct
{
    const char *Index;
    unsigned char Msk[72];
} typFNT_GB24;

typedef struct
{
    const char *Index;
    unsigned char Msk[128];
} typFNT_GB32;

static const unsigned char ascii_1206[95][12] = {0};
static const unsigned char ascii_1608[95][16] = {0};
static const unsigned char ascii_2412[95][48] = {0};
static const unsigned char ascii_3216[95][64] = {0};

static const typFNT_GB12 tfont12[1] = {{"", {0}}};
static const typFNT_GB16 tfont16[1] = {{"", {0}}};
static const typFNT_GB24 tfont24[1] = {{"", {0}}};
static const typFNT_GB32 tfont32[1] = {{"", {0}}};

#endif
