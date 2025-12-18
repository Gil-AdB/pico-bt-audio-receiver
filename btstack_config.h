// btstack_config.h - A2DP Sink configuration
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Enable Classic Bluetooth (A2DP requires this)
#ifndef ENABLE_CLASSIC
#define ENABLE_CLASSIC
#endif

// Enable A2DP Sink
#define ENABLE_A2DP_SINK

// Enable AVRCP
#define ENABLE_AVRCP

// Enable SBC decoder for A2DP
#define ENABLE_SBC_DECODER

// Logging
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// ATT DB not needed for Classic-only A2DP
// #define MAX_ATT_DB_SIZE 256

// HCI / BTstack buffer configuration
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE 1021
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// NVM for link key storage
#define NVM_NUM_LINK_KEYS 2
#define MAX_NR_HCI_CONNECTIONS 2
#define MAX_NR_L2CAP_SERVICES 6
#define MAX_NR_L2CAP_CHANNELS 6
#define MAX_NR_RFCOMM_MULTIPLEXERS 1
#define MAX_NR_RFCOMM_CHANNELS 1
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_BNEP_SERVICES 0
#define MAX_NR_BNEP_CHANNELS 0
#define MAX_NR_HFP_CONNECTIONS 0
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_SERVICE_RECORD_ITEMS 4
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 1
#define MAX_NR_AVDTP_CONNECTIONS 1
#define MAX_NR_AVRCP_CONNECTIONS 1

// Controller buffers
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 0

// A2DP audio configuration
#define A2DP_SINK_SIGNED_16_BIT_PCM 1

// HAL configuration
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define ENABLE_SOFTWARE_AES128

#endif // BTSTACK_CONFIG_H
