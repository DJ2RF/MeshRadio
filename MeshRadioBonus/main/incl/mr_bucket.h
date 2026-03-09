/******************************************************************************
 *  FILE: mr_bucket.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Lightweight token bucket rate limiter used for controlling packet
 *  transmission rates in the MeshRadio firmware.
 *
 *  The token bucket algorithm prevents excessive transmissions by allowing
 *  packets only when tokens are available, while continuously refilling tokens
 *  based on a configured rate.
 *
 *  Used for limiting:
 *      - beacon transmissions
 *      - routing advertisements
 *      - acknowledgements
 *      - user data packets
 *
 *  This mechanism helps maintain fair channel usage and reduces network
 *  congestion in dense mesh deployments.
 ******************************************************************************/

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float tokens;
    float tps;
    float burst;
    uint32_t last_ms;
} bucket_t;

// now_ms() bleibt bei dir (FreeRTOS Tick), wir deklarieren es nur:
uint32_t now_ms(void);

static inline void bucket_init(bucket_t *b, float tps, float burst)
{
    b->tps = tps;
    b->burst = burst;
    b->tokens = burst;
    b->last_ms = now_ms();
}

static inline void bucket_refill(bucket_t *b)
{
    uint32_t t = now_ms();
    uint32_t dt = t - b->last_ms;
    b->last_ms = t;

    float add = (dt / 1000.0f) * b->tps;
    b->tokens += add;
    if(b->tokens > b->burst) b->tokens = b->burst;
}

static inline bool bucket_take(bucket_t *b)
{
    bucket_refill(b);
    if(b->tokens >= 1.0f){
        b->tokens -= 1.0f;
        return true;
    }
    return false;
}

#ifdef __cplusplus
}
#endif