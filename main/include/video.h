#include "esp_h264_dec.h"

struct video_packet {
    uint8_t * data;
    uint32_t data_len;
    uint32_t data_written;
};

int video_packet_finished(struct video_packet *pkt);

// returns number of processed bytes
uint32_t video_packet_process(struct video_packet *pkt, const uint8_t *buffer, uint32_t buffer_len);

void video_init(void);

esp_h264_err_t video_decode(uint8_t *buffer, uint32_t buffer_len);
