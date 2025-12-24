// bt_audio_sink.h - Bluetooth A2DP sink
#ifndef BT_AUDIO_SINK_H
#define BT_AUDIO_SINK_H

#include <stdint.h>

// Initialize A2DP sink
void bt_audio_sink_init(void);

// Process BTstack events
void bt_audio_sink_process(void);

// Get connection status
int bt_audio_sink_is_connected(void);

// Auto-reconnect to last connected device
void bt_audio_sink_schedule_reconnect(void);
void bt_audio_sink_reconnect_last(void);

#endif // BT_AUDIO_SINK_H
