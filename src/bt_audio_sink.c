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

// Volume control (0-127, default 100 = ~79%)
static uint8_t audio_volume = 100;

// Debug counters
static uint32_t total_pcm_samples = 0;
static uint32_t total_media_packets = 0;
static bool sample_rate_set = false;

// SBC configuration
static btstack_sbc_decoder_state_t sbc_decoder_state;
static int16_t pcm_buffer[256 * 2]; // Max SBC frame = 256 samples stereo

// Packet handler forward declaration
static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size);

// Service record for A2DP Sink
static uint8_t a2dp_sink_service_buffer[160];
static uint8_t avrcp_target_service_buffer[200];

// SBC capabilities - accept all configurations (4 bytes)
// Byte 0: Sampling Freq (0xFF = all) | Channel Mode (0xFF = all)
// Byte 1: Block Length | Subbands | Allocation Method (0xFF = all)
// Byte 2-3: Min/Max Bitpool
static uint8_t media_sbc_codec_capabilities[] = {
    0xFF, // All sampling frequencies & channel modes
    0xFF, // All block lengths, subbands, allocation methods
    2,    // Min bitpool
    53    // Max bitpool
};

// Storage for configured codec settings (filled by source)
static uint8_t media_sbc_codec_configuration[4];

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;

  if (packet_type != HCI_EVENT_PACKET)
    return;

  uint8_t event = hci_event_packet_get_type(packet);
  uint8_t status;

  switch (event) {
  case HCI_EVENT_A2DP_META:
    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
      status =
          a2dp_subevent_signaling_connection_established_get_status(packet);
      if (status != ERROR_CODE_SUCCESS) {
        printf("[A2DP] Connection FAILED, status 0x%02x\n", status);
        break;
      }
      a2dp_cid =
          a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
      printf("[A2DP] Connection established, cid 0x%04x\n", a2dp_cid);
      break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
      // Extract sample rate from SBC configuration
      uint8_t sampling_freq =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(
              packet);
      uint32_t sample_rate = 44100; // Default
      if (sampling_freq & 0x20)
        sample_rate = 44100;
      else if (sampling_freq & 0x10)
        sample_rate = 48000;
      else if (sampling_freq & 0x40)
        sample_rate = 32000;
      else if (sampling_freq & 0x80)
        sample_rate = 16000;

      i2s_audio_set_sample_rate(sample_rate);
      printf("[A2DP] SBC configured: %u Hz\n", sample_rate);
      break;
    }

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
      status = a2dp_subevent_stream_established_get_status(packet);
      if (status != ERROR_CODE_SUCCESS) {
        printf("[A2DP] Stream establish FAILED, status 0x%02x\n", status);
        break;
      }
      a2dp_local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
      printf("[A2DP] Stream established, seid %d\n", a2dp_local_seid);
      break;

    case A2DP_SUBEVENT_STREAM_STARTED:
      printf("[A2DP] Stream started\n");
      sample_rate_set = false; // Reset to detect rate on this stream
      i2s_audio_start();
      break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
      printf("[A2DP] Stream suspended\n");
      i2s_audio_stop();
      break;

    case A2DP_SUBEVENT_STREAM_STOPPED:
      printf("[A2DP] Stream stopped\n");
      i2s_audio_stop();
      break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
      printf("[A2DP] Stream released\n");
      i2s_audio_stop();
      break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
      printf("[A2DP] Connection released\n");
      a2dp_cid = 0;
      // Reset state for clean reconnection
      total_pcm_samples = 0;
      total_media_packets = 0;
      sample_rate_set = false; // Reset sample rate flag
      i2s_audio_stop();
      break;

    default:
      printf("[A2DP] Unhandled subevent: 0x%02x\n",
             hci_event_a2dp_meta_get_subevent_code(packet));
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

  // Set sample rate once on first decoded frame
  if (!sample_rate_set && sample_rate > 0) {
    printf("[AUDIO] Detected sample rate: %d Hz\n", sample_rate);
    i2s_audio_set_sample_rate(sample_rate);
    sample_rate_set = true;
  }

  // Write decoded PCM to I2S buffer
  uint32_t total_samples = num_samples * num_channels;
  i2s_audio_write(pcm_data, total_samples);

  total_pcm_samples += total_samples;
}

static void media_packet_handler(uint8_t seid, uint8_t *packet, uint16_t size) {
  (void)seid;

  total_media_packets++;

  // Decode SBC frames
  btstack_sbc_decoder_process_data(&sbc_decoder_state, 0, packet, size);
}

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel,
                                 uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;

  // uint8_t event = hci_event_packet_get_type(packet);
  // printf("[AVRCP_DBG] Packet type 0x%02x, Event 0x%02x\n", packet_type,
  // event);

  if (packet_type != HCI_EVENT_PACKET)
    return;

  uint8_t event = hci_event_packet_get_type(packet);
  // Log every event getting routed to AVRCP handler
  // printf("[AVRCP_DBG] Event 0x%02x\n", event);

  if (event == HCI_EVENT_AVRCP_META) {
    uint8_t subevent = hci_event_avrcp_meta_get_subevent_code(packet);
    printf("[AVRCP] Meta Event 0x%02x\n", subevent);
  }

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

    case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED: {
      uint8_t vol =
          avrcp_subevent_notification_volume_changed_get_absolute_volume(
              packet);
      audio_volume = vol;
      printf("[AVRCP] Volume changed: %d%%\n", (vol * 100) / 127);
      break;
    }

    case AVRCP_SUBEVENT_OPERATION: {
      uint8_t op_id = avrcp_subevent_operation_get_operation_id(packet);
      uint8_t pressed = avrcp_subevent_operation_get_button_pressed(packet);
      if (op_id == AVRCP_OPERATION_ID_VOLUME_UP && pressed) {
        if (audio_volume < 127)
          audio_volume += 10;
        if (audio_volume > 127)
          audio_volume = 127;
        printf("[AVRCP] Volume UP: %d%%\n", (audio_volume * 100) / 127);
      } else if (op_id == AVRCP_OPERATION_ID_VOLUME_DOWN && pressed) {
        if (audio_volume > 10)
          audio_volume -= 10;
        else
          audio_volume = 0;
        printf("[AVRCP] Volume DOWN: %d%%\n", (audio_volume * 100) / 127);
      }
      break;
    }

    default:
      printf("[AVRCP] Subevent: 0x%02x\n",
             hci_event_avrcp_meta_get_subevent_code(packet));
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
      AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities,
      sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration,
      sizeof(media_sbc_codec_configuration));
  a2dp_local_seid = avdtp_local_seid(endpoint);
  printf("[A2DP] Stream endpoint created, seid %d\n", a2dp_local_seid);

  // Initialize AVRCP Target and Controller (both needed for volume)
  avrcp_init();
  avrcp_target_init();
  avrcp_controller_init();
  avrcp_target_register_packet_handler(&avrcp_packet_handler);
  avrcp_controller_register_packet_handler(&avrcp_packet_handler);

  // Create AVRCP service record
  // API: avrcp_target_create_sdp_record(buffer, handle, features, name,
  // provider)
  // Create AVRCP service record
  // API: avrcp_target_create_sdp_record(buffer, handle, features, name,
  // provider) Enable CATEGORY_PLAYER_OR_RECORDER and
  // CATEGORY_MONITOR_OR_AMPLIFIER to support Absolute Volume
  avrcp_target_create_sdp_record(
      avrcp_target_service_buffer, 0x10002,
      AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER |
          AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER,
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
