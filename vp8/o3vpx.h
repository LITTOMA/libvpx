/*
 * Old3DS VP8-derived O3VPX experiment.
 *
 * O3VPX is not a VP8-compatible bitstream. It lives under vp8/ because the
 * encoder/decoder path is intended to fork and reuse VP8 internals while
 * replacing syntax and raw-key handling for the Old3DS profile.
 */

#ifndef VPX_VP8_O3VPX_H_
#define VPX_VP8_O3VPX_H_

#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define O3VPX_WIDTH 800
#define O3VPX_HEIGHT 240
#define O3VPX_FPS 24
#define O3VPX_FRAME_SIZE (O3VPX_WIDTH * O3VPX_HEIGHT * 3 / 2)
#define O3VPX_EYE_WIDTH (O3VPX_WIDTH / 2)
#define O3VPX_EYE_HEIGHT O3VPX_HEIGHT
#define O3VPX_EYE_FRAME_SIZE \
  (O3VPX_EYE_WIDTH * O3VPX_EYE_HEIGHT * 3 / 2)

typedef struct O3vpxFrameInfo {
  unsigned int frame_no;
  unsigned int frame_type;
  unsigned int frame_size_bytes;
} O3vpxFrameInfo;

size_t vp8_o3vpx_decoder_size(void);
size_t vp8_o3vpx_decoder_align(void);
size_t vp8_o3vpx_decoder_internal_bytes(void);
size_t vp8_o3vpx_eye_frame_bytes(void);
int vp8_o3vpx_decoder_init(void *decoder, size_t decoder_size,
                           const unsigned char *stream, size_t stream_len);
int vp8_o3vpx_decoder_init_file(void *decoder, size_t decoder_size,
                                FILE *stream);
int vp8_o3vpx_decoder_reset(void *decoder);
int vp8_o3vpx_decoder_next_frame(void *decoder, O3vpxFrameInfo *info);
int vp8_o3vpx_decoder_write_current_yuv420p(void *decoder,
                                            unsigned char *left,
                                            size_t left_len,
                                            unsigned char *right,
                                            size_t right_len);
void vp8_o3vpx_decoder_drop(void *decoder);

int vp8_o3vpx_encode_file(const char *in_path, const char *out_path, int frames,
                          int keyint, double target_mbps, int search_radius,
                          double scene_mse_threshold, int res_q,
                          double min_gain_per_byte, double p_burst_mult,
                          const char *baseline_psnr_path,
                          const char *previous_psnr_path);
int vp8_o3vpx_decode_file(const char *in_path, const char *out_path);
int vp8_o3vpx_decode_file_limit(const char *in_path, const char *out_path,
                                int max_frames);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VPX_VP8_O3VPX_H_
