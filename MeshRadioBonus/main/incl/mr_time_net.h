#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void mr_time_net_init(void);
void mr_time_net_poll(void);

bool mr_time_net_is_valid(void);
bool mr_time_net_get_datetime(char *out, size_t out_sz);
bool mr_time_net_get_time(char *out, size_t out_sz);
bool mr_time_net_get_date(char *out, size_t out_sz);

#ifdef __cplusplus
}
#endif