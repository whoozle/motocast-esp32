#include "video.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_h264_dec_sw.h"

static const char * TAG = "video";

extern esp_lcd_panel_handle_t panel_handle;

esp_h264_dec_cfg_sw_t h264_config = {
    .pic_type = ESP_H264_RAW_FMT_I420
};
static esp_h264_dec_handle_t h264_handle = NULL;

void video_init() {
    ESP_LOGI(TAG, "initialising video decoder...");
    ESP_ERROR_CHECK(esp_h264_dec_sw_new(&h264_config, &h264_handle));
    ESP_ERROR_CHECK(esp_h264_dec_open(h264_handle));
    ESP_LOGI(TAG, "initialised video decoder.");
}

struct video_packet pkt = {};

esp_h264_dec_out_frame_t out_frame = {};

void video_packet_free(struct video_packet *pkt) {
    free(pkt->data);
    pkt->data = 0;
    pkt->data_len = pkt->data_written = 0;
    pkt->header_read = 0;
}

int video_packet_finished(struct video_packet *pkt) {
    return pkt->data_written >= pkt->data_len;
}

#define MIN(a, b) ((a) < (b)? (a): (b))

esp_h264_err_t video_decode(uint8_t *buffer, uint32_t buffer_len) {
    ESP_LOGI(TAG, "processing %u bytes of decoded data", buffer_len);
    uint32_t src_offset = 0;
    while(src_offset < buffer_len) {
        ESP_LOGI(TAG, "video_packet_process %u/%u", src_offset, buffer_len);
        src_offset += video_packet_process(&pkt, buffer + src_offset, buffer_len - src_offset);
        if (!video_packet_finished(&pkt))
            return ESP_H264_ERR_OK;

        ESP_LOGI(TAG, "video_packet_process finished %u/%u", src_offset, buffer_len);
        esp_h264_dec_in_frame_t in_frame = {.raw_data = { pkt.data, pkt.data_len }};
        while (in_frame.raw_data.len)  {
            int ret = esp_h264_dec_process(h264_handle, &in_frame, &out_frame);
            if (ret != ESP_H264_ERR_OK) {
                ESP_LOGI(TAG, "esp_h264_dec_process error: %d", ret);
            } else {
                ESP_LOGI(TAG, "out frame: %p %u", out_frame.outbuf, out_frame.out_size);
            }
            in_frame.raw_data.buffer += in_frame.consume;
            in_frame.raw_data.len -= in_frame.consume;
        }
        video_packet_free(&pkt);
    }
    ESP_LOGI(TAG, "video_decode finished %u/%u", src_offset, buffer_len);
    return ESP_H264_ERR_OK;
}

uint32_t video_packet_process(struct video_packet *pkt, const uint8_t *buffer, uint32_t buffer_len) {
    uint32_t src_offset = 0;
    if (!pkt->data) {
        while(pkt->header_read < 4 && src_offset < buffer_len) {
            pkt->data_len  |=  (uint32_t)buffer[pkt->header_read] << (pkt->header_read * 8);
            ++pkt->header_read;
            ++src_offset;
        }
        if (pkt->header_read < 4)
            return src_offset;
        pkt->data_written = 0;
        ESP_LOGI(TAG, "video_packet_process, new packet, data len: %u", pkt->data_len);
        pkt->data = (uint8_t*)malloc(pkt->data_len);
        if (!pkt->data) {
            ESP_LOGE(TAG, "malloc failed");
            abort();
        }
    }
    uint32_t to_write = pkt->data_len - pkt->data_written;
    uint32_t to_read = buffer_len - src_offset;
    uint8_t *dst = pkt->data + pkt->data_written;
    const uint8_t *src = buffer + src_offset;
    if (to_read > to_write)
        to_read = to_write;
    memcpy(dst, src, to_read);
    pkt->data_written += to_read;
    return to_read + src_offset;
}
