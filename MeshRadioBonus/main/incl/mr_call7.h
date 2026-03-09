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

static inline void call7_set(char o[7], const char *s)
{
    memset(o,' ',7);
    size_t n = s ? strlen(s) : 0;
    if(n > 7) n = 7;
    if(n) memcpy(o, s, n);
}

static inline void call7_to_str(char o[8], const char i[7])
{
    memcpy(o, i, 7);
    o[7] = 0;
    for(int k = 6; k >= 0; k--){
        if(o[k] == ' ') o[k] = 0;
        else break;
    }
}

static inline bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a, b, 7) == 0;
}

static inline bool call7_is_wild(const char a[7])
{
    return (a[0]=='*' && a[1]==' ' && a[2]==' ' && a[3]==' ' &&
            a[4]==' ' && a[5]==' ' && a[6]==' ');
}

#ifdef __cplusplus
}
#endif