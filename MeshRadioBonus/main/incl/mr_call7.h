/******************************************************************************
 *  FILE: mr_call7.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Utility helpers for handling fixed-length 7 character callsign identifiers
 *  used within the MeshRadio protocol.
 *
 *  Functions include:
 *    - setting and padding callsigns
 *    - converting callsigns to printable strings
 *    - comparing callsigns
 *    - detecting wildcard addresses
 *
 *  These helpers guarantee consistent formatting of callsigns inside radio
 *  packets and routing tables.
 ******************************************************************************/

#pragma once
#include <string.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void call7_set(char o[8], const char *s)
{
    memset(o,' ',8);
    size_t n = s ? strlen(s) : 0;
    if(n > 8) n = 8;
    if(n) memcpy(o, s, n);
}

static inline void call7_to_str(char o[9], const char i[8])
{
    memcpy(o, i, 8);
    o[8] = 0;
    for(int k = 7; k >= 0; k--){
        if(o[k] == ' ') o[k] = 0;
        else break;
    }
}

static inline bool call7_eq(const char a[8], const char b[8])
{
    return memcmp(a, b, 8) == 0;
}

static inline bool call7_is_wild(const char a[8])
{
    return (a[0]=='*' && a[1]==' ' && a[2]==' ' && a[3]==' ' &&
            a[4]==' ' && a[5]==' ' && a[6]==' ' && a[7]==' ');
}

#ifdef __cplusplus
}
#endif