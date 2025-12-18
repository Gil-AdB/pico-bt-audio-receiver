// bt_audio_sink.c - Bluetooth A2DP Sink implementation
#include "bt_audio_sink.h"
#include "btstack.h"
#include "i2s_audio.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

// A2DP sink state
static uint8_t a2dp_local_seid = 0;
static uint16_t a2dp_cid = 0;
static uint8_t avrcp_connected = 0;
static uint8_t media_initialized = 0;

// SBC configuration
static btstack_sbc_decoder_state_t sbc_decoder_state;
static int16_t pcm_buffer[256 * 2]; // Max SBC frame = 256 samples stereo

// Packet handler forward declaration
static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size);

// Service record for A2DP Sink
static uint8_t a2dp_sink_service_buffer[150];
static uint8_t avrcp_target_service_buffer[150];

// SBC capabilities (non-const for BTstack API)
static uint8_t a2dp_sink_capabilities[] = {
    // Media type audio
    0x00,
    // SBC codec
    0x00,
    // SBC capabilities:
    // Sampling: 44100 Hz, Channels: Stereo
    // Block length: 16, Subbands: 8, Allocation: Loudness
    0x21, // 44100, stereo
    0x15, // block 16, subbands 8
    0x02, // min bitpool
    0x35  // max bitpool (53)
};

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;

  if (packet_type != HCI_EVENT_PACKET)
    return;

  uint8_t event = hci_event_packet_get_type(packet);

  switch (event) {
  case HCI_EVENT_A2DP_META:
    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
      a2dp_cid =
          a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
      printf("[A2DP] Connection established, cid 0x%04x\n", a2dp_cid);
      break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
      printf("[A2DP] Connection released\n");
      a2dp_cid = 0;
      i2s_audio_stop();
      break;

    case A2DP_SUBEVENT_STREAM_STARTED:
      printf("[A2DP] Stream started\n");
      i2s_audio_start();
      break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
    case A2DP_SUBEVENT_STREAM_STOPPED:
      printf("[A2DP] Stream stopped\n");
      i2s_audio_stop();
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }
}

static void sbc_decode_callback(int16_t *pcm_data, int num_samples,
                                int num_channels, int sample_rate,
                                void *context) {
  (void)context;
  (void)sample_rate;

  // Write decoded PCM to I2S buffer
  uint32_t total_samples = num_samples * num_channels;
  i2s_audio_write(pcm_data, total_samples);
}

static void media_packet_handler(uint8_t seid, uint8_t *packet, uint16_t size) {
  (void)seid;

  // Decode SBC frames
  btstack_sbc_decoder_process_data(&sbc_decoder_state, 0, packet, size);
}

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel,
                                 uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;

  if (packet_type != HCI_EVENT_PACKET)
    return;

  uint8_t event = hci_event_packet_get_type(packet);

  switch (event) {
  case HCI_EVENT_AVRCP_META:
    switch (hci_event_avrcp_meta_get_subevent_code(packet)) {
    case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED:
      avrcp_connected = 1;
      printf("[AVRCP] Connected\n");
      break;

    case AVRCP_SUBEVENT_CONNECTION_RELEASED:
      avrcp_connected = 0;
      printf("[AVRCP] Disconnected\n");
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }
}

void bt_audio_sink_init(void) {
  printf("Initializing Bluetooth A2DP Sink...\n");

  // Initialize I2S audio
  i2s_audio_init();

  // Initialize SBC decoder
  btstack_sbc_decoder_init(&sbc_decoder_state, SBC_MODE_STANDARD,
                           sbc_decode_callback, NULL);

  // Initialize L2CAP
  l2cap_init();

  // Initialize RFCOMM (needed for some services)
  rfcomm_init();

  // Initialize SDP
  sdp_init();

  // Initialize A2DP Sink
  a2dp_sink_init();
  a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
  a2dp_sink_register_media_handler(&media_packet_handler);

  // Create A2DP service record
  a2dp_sink_create_sdp_record(a2dp_sink_service_buffer, 0x10001, AVDTP_SINK,
                              NULL, 0);
  sdp_register_service(a2dp_sink_service_buffer);

  // Register SBC endpoint
  avdtp_stream_endpoint_t *endpoint = a2dp_sink_create_stream_endpoint(
      AVDTP_AUDIO, AVDTP_CODEC_SBC, a2dp_sink_capabilities,
      sizeof(a2dp_sink_capabilities), a2dp_sink_capabilities,
      sizeof(a2dp_sink_capabilities));
  a2dp_local_seid = avdtp_local_seid(endpoint);

  // Initialize AVRCP Target
  avrcp_target_init();
  avrcp_target_register_packet_handler(&avrcp_packet_handler);

  // Create AVRCP service record
  // API: avrcp_target_create_sdp_record(buffer, handle, features, name,
  // provider)
  avrcp_target_create_sdp_record(avrcp_target_service_buffer, 0x10002,
                                 AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER,
                                 "Pico Audio", NULL);
  sdp_register_service(avrcp_target_service_buffer);

  // Set device name and class
  gap_set_local_name("Pico Audio Receiver");
  gap_discoverable_control(1);
  gap_set_class_of_device(0x200414); // Audio/AV, Headphones
  gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE |
                                       LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

  // Make connectable
  gap_connectable_control(1);

  printf("Bluetooth A2DP Sink ready - discoverable as 'Pico Audio Receiver'\n");
}

void bt_audio_sink_process(void) {
  // BTstack handles events via callbacks
}

int bt_audio_sink_is_connected(void) { return (a2dp_cid != 0); }
