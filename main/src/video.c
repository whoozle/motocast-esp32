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

static uint16_t *fb_rgb;

static const unsigned W = 320, H = 240;

void video_init() {
    ESP_LOGI(TAG, "initialising video decoder...");
    ESP_ERROR_CHECK(esp_h264_dec_sw_new(&h264_config, &h264_handle));
    ESP_ERROR_CHECK(esp_h264_dec_open(h264_handle));
    ESP_LOGI(TAG, "initialised video decoder.");
    fb_rgb = (uint16_t*)malloc(W * H * 2);
    if (!fb_rgb) {
        ESP_LOGE(TAG, "no memory for RGB framebuffer");
        abort();
    }
}

struct video_packet pkt = {};

esp_h264_dec_out_frame_t out_frame = {};

void video_present_frame(const uint8_t *yuv420) {
    static const unsigned W2 = W / 2, H2 = H / 2;
    static const unsigned LSIZE = W * H;
    static const unsigned CSIZE = W2 * H2;
    const uint8_t * Y = yuv420;
    const uint8_t * U = Y + LSIZE;
    const uint8_t * V = U + CSIZE;
    uint16_t *dst = fb_rgb;
    for(unsigned i = 0; i != H; ++i) {
        for(unsigned j = 0; j != W; ++j) {
            int16_t t_y = Y[((i * W) + j)];
            int16_t t_u = U[(((i / 2) * W2) + (j / 2))] - 128;
            int16_t t_v = V[(((i / 2) * W2) + (j / 2))] - 128;
            t_y = t_y < 16 ? 0 : t_y - 16;

            int16_t r = (298 * t_y + 409 * t_v + 128) >> 8;
            int16_t g = (298 * t_y - 100 * t_u - 208 * t_v + 128) >> 8;
            int16_t b = (298 * t_y + 516 * t_u + 128) >> 8;
            r >>= 3;
            g >>= 2;
            b >>= 3;
            *dst++ = (r << 11) | (g << 5) | b;
        }
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, W, H, fb_rgb));
}

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
    uint32_t src_offset = 0;
    while(src_offset < buffer_len) {
        src_offset += video_packet_process(&pkt, buffer + src_offset, buffer_len - src_offset);
        if (!video_packet_finished(&pkt))
            return ESP_H264_ERR_OK;

        esp_h264_dec_in_frame_t in_frame = {.raw_data = { pkt.data, pkt.data_len }};
        while (in_frame.raw_data.len)  {
            int ret = esp_h264_dec_process(h264_handle, &in_frame, &out_frame);
            if (ret != ESP_H264_ERR_OK) {
                ESP_LOGI(TAG, "esp_h264_dec_process error: %d", ret);
            } else {
                if (out_frame.outbuf)
                    video_present_frame(out_frame.outbuf);
            }
            in_frame.raw_data.buffer += in_frame.consume;
            in_frame.raw_data.len -= in_frame.consume;
        }
        video_packet_free(&pkt);
    }
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
