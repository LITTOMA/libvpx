/*
 * CLI wrapper for the VP8-derived O3VPX experiment.
 *
 * The codec implementation lives under vp8/o3vpx.c so this tool does not
 * become a standalone side codec.
 */

#include "vp8/o3vpx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *prog) {
  fprintf(stderr,
          "usage:\n"
          "  %s enc <in-800x240-yuv420.yuv> <out.o3vx> [frames] [keyint] "
          "[target_mbps] [search_radius] [scene_mse_threshold] [res_q] "
          "[min_gain_per_byte] [p_burst_mult] [baseline_psnr.log] "
          "[previous_o3vpx_psnr.log]\n"
          "  %s dec <in.o3vx> <out-800x240-yuv420.yuv> [frames]\n"
          "  %s decmem <in.o3vx> <out-800x240-yuv420.yuv> [frames]\n"
          "  %s decnull <in.o3vx> [frames]\n"
          "encoder env:\n"
          "  O3VPX_DECODE_COST_BUDGET\n"
          "  O3VPX_MAX_P_FRAME_BYTES\n"
          "  O3VPX_MAX_FULL_IDCT_BLOCKS\n"
          "  O3VPX_MAX_HALFPEL_MB\n"
          "  O3VPX_MAX_RESIDUAL_BLOCKS\n",
          prog, prog, prog, prog);
}

static long local_file_size(FILE *file) {
  long pos;
  long end;
  if (fseek(file, 0, SEEK_CUR) != 0) return -1;
  pos = ftell(file);
  if (pos < 0) return -1;
  if (fseek(file, 0, SEEK_END) != 0) return -1;
  end = ftell(file);
  if (end < 0) return -1;
  if (fseek(file, pos, SEEK_SET) != 0) return -1;
  return end;
}

static int decode_memory_to_file(const char *in_path, const char *out_path,
                                 int max_frames) {
  FILE *in;
  FILE *out;
  long input_size;
  unsigned char *stream;
  void *decoder;
  unsigned char *left;
  unsigned char *right;
  unsigned char *sbs;
  size_t decoder_size = vp8_o3vpx_decoder_size();
  size_t eye_size = vp8_o3vpx_eye_frame_bytes();
  int frame_count = 0;
  int rc = EXIT_FAILURE;
  in = fopen(in_path, "rb");
  if (!in) {
    perror("open input failed");
    return EXIT_FAILURE;
  }
  input_size = local_file_size(in);
  if (input_size <= 0) {
    fprintf(stderr, "bad input size\n");
    fclose(in);
    return EXIT_FAILURE;
  }
  stream = (unsigned char *)malloc((size_t)input_size);
  decoder = malloc(decoder_size);
  left = (unsigned char *)malloc(eye_size);
  right = (unsigned char *)malloc(eye_size);
  sbs = (unsigned char *)malloc(O3VPX_FRAME_SIZE);
  if (!stream || !decoder || !left || !right || !sbs) {
    fprintf(stderr, "allocation failed\n");
    goto done;
  }
  if (fread(stream, 1, (size_t)input_size, in) != (size_t)input_size) {
    fprintf(stderr, "read failed\n");
    goto done;
  }
  out = fopen(out_path, "wb");
  if (!out) {
    perror("open output failed");
    goto done;
  }
  if (vp8_o3vpx_decoder_init(decoder, decoder_size, stream,
                             (size_t)input_size) != 0) {
    fprintf(stderr, "decoder init failed\n");
    fclose(out);
    goto done;
  }
  for (;;) {
    O3vpxFrameInfo info;
    int row;
    int dec_rc;
    if (max_frames > 0 && frame_count >= max_frames) break;
    dec_rc = vp8_o3vpx_decoder_next_frame(decoder, &info);
    if (dec_rc == 0) break;
    if (dec_rc < 0) {
      fprintf(stderr, "decode failed: %d\n", dec_rc);
      fclose(out);
      goto done_drop;
    }
    if (vp8_o3vpx_decoder_write_current_yuv420p(
            decoder, left, eye_size, right, eye_size) != 0) {
      fprintf(stderr, "split output failed\n");
      fclose(out);
      goto done_drop;
    }
    for (row = 0; row < O3VPX_HEIGHT; ++row) {
      memcpy(sbs + row * O3VPX_WIDTH, left + row * O3VPX_EYE_WIDTH,
             O3VPX_EYE_WIDTH);
      memcpy(sbs + row * O3VPX_WIDTH + O3VPX_EYE_WIDTH,
             right + row * O3VPX_EYE_WIDTH, O3VPX_EYE_WIDTH);
    }
    for (row = 0; row < O3VPX_HEIGHT / 2; ++row) {
      const size_t src_row = (size_t)row * (O3VPX_EYE_WIDTH / 2);
      const size_t dst_row = (size_t)row * (O3VPX_WIDTH / 2);
      const unsigned char *left_u =
          left + O3VPX_EYE_WIDTH * O3VPX_EYE_HEIGHT;
      const unsigned char *left_v =
          left_u + (O3VPX_EYE_WIDTH / 2) * (O3VPX_EYE_HEIGHT / 2);
      const unsigned char *right_u =
          right + O3VPX_EYE_WIDTH * O3VPX_EYE_HEIGHT;
      const unsigned char *right_v =
          right_u + (O3VPX_EYE_WIDTH / 2) * (O3VPX_EYE_HEIGHT / 2);
      unsigned char *sbs_u = sbs + O3VPX_WIDTH * O3VPX_HEIGHT;
      unsigned char *sbs_v =
          sbs_u + (O3VPX_WIDTH / 2) * (O3VPX_HEIGHT / 2);
      memcpy(sbs_u + dst_row, left_u + src_row, O3VPX_EYE_WIDTH / 2);
      memcpy(sbs_u + dst_row + O3VPX_EYE_WIDTH / 2, right_u + src_row,
             O3VPX_EYE_WIDTH / 2);
      memcpy(sbs_v + dst_row, left_v + src_row, O3VPX_EYE_WIDTH / 2);
      memcpy(sbs_v + dst_row + O3VPX_EYE_WIDTH / 2, right_v + src_row,
             O3VPX_EYE_WIDTH / 2);
    }
    if (fwrite(sbs, 1, O3VPX_FRAME_SIZE, out) != O3VPX_FRAME_SIZE) {
      fprintf(stderr, "write failed\n");
      fclose(out);
      goto done_drop;
    }
    ++frame_count;
  }
  fprintf(stderr, "o3vpx_decmem_summary frames=%d bytes=%ld\n", frame_count,
          input_size);
  rc = EXIT_SUCCESS;
  fclose(out);
done_drop:
  vp8_o3vpx_decoder_drop(decoder);
done:
  free(sbs);
  free(right);
  free(left);
  free(decoder);
  free(stream);
  fclose(in);
  return rc;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    usage(argv[0]);
    return EXIT_FAILURE;
  }
  if (strcmp(argv[1], "enc") == 0) {
    int frames = 0;
    int keyint = 48;
    double target_mbps = 8.0;
    int radius = 8;
    double scene_mse_threshold = 2000.0;
    int res_q = 32;
    double min_gain_per_byte = 0.0;
    double p_burst_mult = 1.0;
    const char *baseline_psnr_path = NULL;
    const char *previous_psnr_path = NULL;
    if (argc < 4 || argc > 14) {
      usage(argv[0]);
      return EXIT_FAILURE;
    }
    if (argc >= 5) frames = atoi(argv[4]);
    if (argc >= 6) keyint = atoi(argv[5]);
    if (argc >= 7) target_mbps = atof(argv[6]);
    if (argc >= 8) radius = atoi(argv[7]);
    if (argc >= 9) scene_mse_threshold = atof(argv[8]);
    if (argc >= 10) res_q = atoi(argv[9]);
    if (argc >= 11) min_gain_per_byte = atof(argv[10]);
    if (argc >= 12) p_burst_mult = atof(argv[11]);
    if (argc >= 13) baseline_psnr_path = argv[12];
    if (argc >= 14) previous_psnr_path = argv[13];
    return vp8_o3vpx_encode_file(argv[2], argv[3], frames, keyint, target_mbps,
                                 radius, scene_mse_threshold, res_q,
                                 min_gain_per_byte, p_burst_mult,
                                 baseline_psnr_path, previous_psnr_path);
  }
  if (strcmp(argv[1], "dec") == 0) {
    int frames = 0;
    if (argc < 4 || argc > 5) {
      usage(argv[0]);
      return EXIT_FAILURE;
    }
    if (argc >= 5) frames = atoi(argv[4]);
    return vp8_o3vpx_decode_file_limit(argv[2], argv[3], frames);
  }
  if (strcmp(argv[1], "decmem") == 0) {
    int frames = 0;
    if (argc < 4 || argc > 5) {
      usage(argv[0]);
      return EXIT_FAILURE;
    }
    if (argc >= 5) frames = atoi(argv[4]);
    return decode_memory_to_file(argv[2], argv[3], frames);
  }
  if (strcmp(argv[1], "decnull") == 0) {
    int frames = 0;
    if (argc < 3 || argc > 4) {
      usage(argv[0]);
      return EXIT_FAILURE;
    }
    if (argc >= 4) frames = atoi(argv[3]);
    return vp8_o3vpx_decode_file_limit(argv[2], NULL, frames);
  }
  usage(argv[0]);
  return EXIT_FAILURE;
}
