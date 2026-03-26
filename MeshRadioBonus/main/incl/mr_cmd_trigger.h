#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mr_cmd_trigger_init(void);
void mr_cmd_trigger_apply_cfg(void);
void mr_cmd_trigger_poll(void);

void mr_cmd_trigger_mark_sent(void);
void mr_cmd_trigger_mark_confirmed(const char *from_callsign);

bool mr_cmd_trigger_is_enabled(void);

#ifdef __cplusplus
}
#endif