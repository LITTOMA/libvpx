/*
 * Old3DS VP8-derived codec experiment.
 *
 * This is intentionally not a VP8-compatible bitstream. It is the first
 * VP8-side O3VPX profile for the GUIDE_v2 route: raw YUV keyframes, one
 * reference frame, half-pel luma 16x16 motion, bounded raw macroblock and 4x4
 * repair, and a byte-aligned VP8 4x4 transform residual mode.
 */

#define _POSIX_C_SOURCE 200809L

#include "vp8/o3vpx.h"

#include "./vpx_dsp_rtcd.h"
#include "vp8/common/mv.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef __3DS__
#include <3ds.h>
#endif

#define O3VPX_MAGIC "O3VX"
#define O3VPX_VERSION 13
#define WIDTH O3VPX_WIDTH
#define HEIGHT O3VPX_HEIGHT
#define FPS O3VPX_FPS
#define MB_SIZE 16
#define MB_W (WIDTH / MB_SIZE)
#define MB_H (HEIGHT / MB_SIZE)
#define MB_COUNT (MB_W * MB_H)
#define Y_SIZE (WIDTH * HEIGHT)
#define UV_W (WIDTH / 2)
#define UV_H (HEIGHT / 2)
#define UV_SIZE (UV_W * UV_H)
#define FRAME_SIZE O3VPX_FRAME_SIZE
#define READER_BUFFER_SIZE (32 * 1024)
#define RAW_MB_BYTES (16 * 16 + 8 * 8 + 8 * 8)
#define RAW_Y_MB_BYTES (16 * 16)
#define RAW_UV_MB_BYTES (8 * 8 + 8 * 8)
#define RAW_4X4_BYTES (4 * 4)
#define COPY16_BYTES 3
#define MODE_COPY16 0
#define MODE_RAW_MB 1
#define MODE_COPY16_PATCH4 2
#define MODE_COPY16_RES4 3
#define MODE_INTRA_DC 4
#define MODE_INTRA_V 5
#define MODE_INTRA_H 6
#define MODE_RAW_Y_MB 7
#define MODE_RAW_UV_MB 8
#define MODE_COPY16_DC 9
#define MODE_COPY16_QDC 10
#define MODE_COPY16_RES4_RAWUV 11
#define FRAME_RAW_KEY 0
#define FRAME_P 1
#define PATCH_PLANE_Y 0
#define PATCH_PLANE_U 1
#define PATCH_PLANE_V 2
#define PATCH_CANDIDATES_PER_MB (16 + 4 + 4)
#define PATCH_CANDIDATE_COUNT (MB_COUNT * PATCH_CANDIDATES_PER_MB)
#define RES_BLOCKS_PER_MB PATCH_CANDIDATES_PER_MB
#define RES_CANDIDATE_COUNT (PATCH_CANDIDATE_COUNT * 2)
#define REPAIR_CANDIDATE_COUNT (RES_CANDIDATE_COUNT + MB_COUNT * 4)
#define RES_MAX_Q 255
#define RAW_MB_EXTRA_COST (1 + RAW_MB_BYTES - 3)
#define RAW_Y_MB_EXTRA_COST (1 + 2 + RAW_Y_MB_BYTES - 3)
#define RAW_UV_MB_EXTRA_COST (1 + 2 + RAW_UV_MB_BYTES - 3)
#define COPY16_DC_EXTRA_COST 3
#define COPY16_QDC_EXTRA_COST 6
#define REPAIR_CAND_RES4 0
#define REPAIR_CAND_RAW_MB 1
#define REPAIR_CAND_RAW4 2
#define REPAIR_CAND_RAW_Y_MB 3
#define REPAIR_CAND_RAW_UV_MB 4
#define REPAIR_CAND_COPY16_DC 5
#define REPAIR_CAND_COPY16_QDC 6
#define REPAIR_RAW4_FLAG 0x80
#define REPAIR_NIBBLE_FLAG 0x40
#define REPAIR_DC4_FLAG 0x20
#define REPAIR_TABLE_FLAG (REPAIR_DC4_FLAG | REPAIR_NIBBLE_FLAG)
#define REPAIR_SLOT_MASK 0x1f
#define RES4_MASK_TABLE_COUNT 16
#define CHROMA_REPAIR_BIAS_U 2.35
#define CHROMA_REPAIR_BIAS_V 2.70
#define KEY_SCENE_LOOKAHEAD 4
#define BUDGET_WEIGHT_DEN 256.0
#define BUDGET_WEIGHT_CAP 3.0
#define BUDGET_AGE_WEIGHT 0.0
#define QUALITY_BUDGET_BIAS_MIN 0.75
#define QUALITY_BUDGET_BIAS_MAX 1.85
#define QUALITY_GAIN_SCALE_MIN 0.75
#define QUALITY_GAIN_SCALE_MAX 2.00
#define QUALITY_PLANE_GAIN_MIN 0.80
#define QUALITY_PLANE_GAIN_MAX 1.75
#define IDCT_COSPI8SQRT2MINUS1 20091
#define IDCT_SINPI8SQRT2 35468

void vp8_short_fdct4x4_c(short *input, short *output, int pitch);
void vp8_short_idct4x4llm_c(short *input, unsigned char *pred_ptr,
                            int pred_stride, unsigned char *dst_ptr,
                            int dst_stride);
void vp8_dc_only_idct_add_c(short input_dc, unsigned char *pred_ptr,
                            int pred_stride, unsigned char *dst_ptr,
                            int dst_stride);

typedef struct MbAnalysis {
  int index;
  // VP8 normally stores subpel units in MV. O3VPX stores half-pel luma units.
  MV mv;
  unsigned int sad;
  int raw_mode;
  int8_t dc_y;
  int8_t dc_u;
  int8_t dc_v;
  int8_t dc_y4[4];
} MbAnalysis;

typedef struct ResBlock {
  uint8_t type;
  uint8_t plane;
  uint8_t block;
  uint8_t nz;
  uint8_t packed_coeffs;
  uint16_t cost;
  uint16_t coeff_mask;
  uint64_t gain;
  int8_t qcoeff[16];
  uint8_t raw[RAW_4X4_BYTES];
} ResBlock;

typedef struct MbResState {
  uint32_t block_mask;
  uint8_t count;
  size_t cost;
  uint64_t gain;
  ResBlock blocks[RES_BLOCKS_PER_MB];
} MbResState;

typedef struct ResCandidate {
  ResBlock block;
  int mb_index;
  uint64_t gain;
  uint64_t pred_sse;
  uint64_t recon_sse;
  uint16_t cost;
  uint8_t type;
  int8_t dc_y;
  int8_t dc_u;
  int8_t dc_v;
  int8_t dc_y4[4];
} ResCandidate;

typedef struct QualityMetrics {
  double mse_avg;
  double mse_y;
  double mse_u;
  double mse_v;
  int valid;
} QualityMetrics;

typedef struct Buffer {
  uint8_t *data;
  size_t len;
  size_t cap;
} Buffer;

typedef struct O3vpxReader {
  FILE *file;
  const uint8_t *data;
  size_t len;
  size_t pos;
  uint8_t buffer[READER_BUFFER_SIZE];
  size_t buffer_pos;
  size_t buffer_len;
} O3vpxReader;

typedef struct O3vpxMbCommand {
  uint8_t mode;
  uint8_t block_count;
  MV mv;
  int8_t dc_y;
  int8_t dc_u;
  int8_t dc_v;
  int8_t dc_y4[4];
  union {
    uint8_t raw[RAW_MB_BYTES];
    ResBlock blocks[RES_BLOCKS_PER_MB];
    struct {
      uint8_t raw[RAW_UV_MB_BYTES];
      ResBlock blocks[16];
    } rawuv;
  } data;
} O3vpxMbCommand;

typedef struct O3vpxReconstructJob {
  uint8_t *recon;
  const uint8_t *ref;
  const O3vpxMbCommand *commands;
  int res_q;
  int eye;
} O3vpxReconstructJob;

typedef struct O3vpxDecoderState {
  const uint8_t *stream;
  size_t stream_len;
  FILE *stream_file;
  O3vpxReader reader;
  int frames;
  int res_q;
  int frame_no;
  uint8_t *ref;
  uint8_t *recon;
  O3vpxMbCommand *commands;
#ifdef __3DS__
  Thread worker_thread;
  volatile int worker_running;
  volatile int worker_has_job;
  volatile int worker_done;
  O3vpxReconstructJob worker_job;
#endif
} O3vpxDecoderState;

static uint64_t scaled_gain(uint64_t gain, double scale);
static uint64_t repair_score_gain(uint8_t plane, uint64_t gain,
                                  const double *plane_gain_scale);

static const uint8_t vp8_zigzag[16] = {
  0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};

static const uint16_t res4_mask_table[RES4_MASK_TABLE_COUNT] = {
  0x0005, 0x0003, 0x0007, 0x000d, 0x0009, 0x0011, 0x0021, 0x0004,
  0x0017, 0x020d, 0x0023, 0x0013, 0x0015, 0x0002, 0x000f, 0x0027,
};

static int res4_mask_table_index(uint16_t mask) {
  int i;
  for (i = 0; i < RES4_MASK_TABLE_COUNT; ++i) {
    if (res4_mask_table[i] == mask) return i;
  }
  return -1;
}

static uint8_t qcoeff_to_nibble(int8_t q) {
  return (uint8_t)q & 0x0f;
}

static int8_t qcoeff_from_nibble(uint8_t nibble) {
  return (int8_t)((nibble & 0x08) ? (int)nibble - 16 : (int)nibble);
}

static uint16_t res4_encoded_cost(uint16_t mask, uint8_t packed_coeffs,
                                  uint8_t nz) {
  if (mask == 1) return 2;
  if (packed_coeffs && res4_mask_table_index(mask) >= 0) {
    return (uint16_t)(2 + (nz >> 1));
  }
  return (uint16_t)(3 + (packed_coeffs ? ((nz + 1) >> 1) : nz));
}

static double now_seconds(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static void die(const char *msg) {
  fprintf(stderr, "o3vpx: %s\n", msg);
  exit(EXIT_FAILURE);
}

static void die_errno(const char *msg) {
  fprintf(stderr, "o3vpx: %s: %s\n", msg, strerror(errno));
  exit(EXIT_FAILURE);
}

static void *xmalloc(size_t size) {
  void *ptr = malloc(size);
  if (!ptr) die_errno("malloc failed");
  return ptr;
}

static void buffer_init(Buffer *buf) {
  buf->data = NULL;
  buf->len = 0;
  buf->cap = 0;
}

static void buffer_free(Buffer *buf) {
  free(buf->data);
  buf->data = NULL;
  buf->len = 0;
  buf->cap = 0;
}

static void buffer_reserve(Buffer *buf, size_t extra) {
  size_t need = buf->len + extra;
  if (need <= buf->cap) return;
  size_t cap = buf->cap ? buf->cap : 4096;
  while (cap < need) cap *= 2;
  uint8_t *data = (uint8_t *)realloc(buf->data, cap);
  if (!data) die_errno("realloc failed");
  buf->data = data;
  buf->cap = cap;
}

static void buffer_u8(Buffer *buf, uint8_t v) {
  buffer_reserve(buf, 1);
  buf->data[buf->len++] = v;
}

static void buffer_le16(Buffer *buf, uint16_t v) {
  buffer_reserve(buf, 2);
  buf->data[buf->len++] = (uint8_t)(v & 0xff);
  buf->data[buf->len++] = (uint8_t)((v >> 8) & 0xff);
}

static void buffer_bytes(Buffer *buf, const uint8_t *data, size_t len) {
  buffer_reserve(buf, len);
  memcpy(buf->data + buf->len, data, len);
  buf->len += len;
}

static void put_le16(FILE *f, uint16_t v) {
  uint8_t b[2];
  b[0] = (uint8_t)(v & 0xff);
  b[1] = (uint8_t)((v >> 8) & 0xff);
  if (fwrite(b, 1, sizeof(b), f) != sizeof(b)) die_errno("write failed");
}

static void put_le32(FILE *f, uint32_t v) {
  uint8_t b[4];
  b[0] = (uint8_t)(v & 0xff);
  b[1] = (uint8_t)((v >> 8) & 0xff);
  b[2] = (uint8_t)((v >> 16) & 0xff);
  b[3] = (uint8_t)((v >> 24) & 0xff);
  if (fwrite(b, 1, sizeof(b), f) != sizeof(b)) die_errno("write failed");
}

static void reader_init_file(O3vpxReader *reader, FILE *file) {
  memset(reader, 0, sizeof(*reader));
  reader->file = file;
}

static void reader_init_mem(O3vpxReader *reader, const uint8_t *data,
                            size_t len) {
  memset(reader, 0, sizeof(*reader));
  reader->data = data;
  reader->len = len;
}

static void reader_exact(O3vpxReader *reader, void *dst, size_t len) {
  if (reader->file) {
    uint8_t *out = (uint8_t *)dst;
    while (len > 0) {
      size_t available = reader->buffer_len - reader->buffer_pos;
      size_t take;
      if (available == 0) {
        reader->buffer_pos = 0;
        reader->buffer_len = 0;
        if (len >= sizeof(reader->buffer)) {
          if (fread(out, 1, len, reader->file) != len) die("unexpected EOF");
          return;
        }
        reader->buffer_len =
            fread(reader->buffer, 1, sizeof(reader->buffer), reader->file);
        if (reader->buffer_len == 0) die("unexpected EOF");
        available = reader->buffer_len;
      }
      take = len < available ? len : available;
      memcpy(out, reader->buffer + reader->buffer_pos, take);
      reader->buffer_pos += take;
      out += take;
      len -= take;
    }
    return;
  }
  if (reader->pos > reader->len || len > reader->len - reader->pos) {
    die("unexpected EOF");
  }
  memcpy(dst, reader->data + reader->pos, len);
  reader->pos += len;
}

static uint8_t get_u8(O3vpxReader *reader) {
  if (reader->file) {
    if (reader->buffer_pos >= reader->buffer_len) {
      reader->buffer_pos = 0;
      reader->buffer_len =
          fread(reader->buffer, 1, sizeof(reader->buffer), reader->file);
      if (reader->buffer_len == 0) die("unexpected EOF");
    }
    return reader->buffer[reader->buffer_pos++];
  }
  if (reader->pos >= reader->len) die("unexpected EOF");
  return reader->data[reader->pos++];
}

static uint16_t get_le16(O3vpxReader *reader) {
  uint16_t v = get_u8(reader);
  v |= (uint16_t)get_u8(reader) << 8;
  return v;
}

static uint32_t get_le32(O3vpxReader *reader) {
  uint32_t v = get_u8(reader);
  v |= (uint32_t)get_u8(reader) << 8;
  v |= (uint32_t)get_u8(reader) << 16;
  v |= (uint32_t)get_u8(reader) << 24;
  return v;
}

static size_t file_size(FILE *f) {
  long pos;
  long end;
  if (fseek(f, 0, SEEK_CUR) != 0) die_errno("seek failed");
  pos = ftell(f);
  if (pos < 0) die_errno("tell failed");
  if (fseek(f, 0, SEEK_END) != 0) die_errno("seek failed");
  end = ftell(f);
  if (end < 0) die_errno("tell failed");
  if (fseek(f, pos, SEEK_SET) != 0) die_errno("seek failed");
  return (size_t)end;
}

static int mb_eye_min_x(int mb_x) { return mb_x < MB_W / 2 ? 0 : MB_W / 2; }

static int round_div4(int v) {
  return v >= 0 ? (v + 2) / 4 : -((-v + 2) / 4);
}

static uint8_t avg2_u8(uint8_t a, uint8_t b) {
  return (uint8_t)(((int)a + (int)b + 1) >> 1);
}

static uint8_t sample_halfpel_u8(const uint8_t *plane, int stride, int x2,
                                 int y2) {
  const int x = x2 >> 1;
  const int y = y2 >> 1;
  const int fx = x2 & 1;
  const int fy = y2 & 1;
  const uint8_t a = plane[y * stride + x];
  if (!fx && !fy) return a;
  if (fx && !fy) return avg2_u8(a, plane[y * stride + x + 1]);
  if (!fx && fy) return avg2_u8(a, plane[(y + 1) * stride + x]);
  return (uint8_t)(((int)a + (int)plane[y * stride + x + 1] +
                    (int)plane[(y + 1) * stride + x] +
                    (int)plane[(y + 1) * stride + x + 1] + 2) >>
                   2);
}

static unsigned int mb_sad(const uint8_t *src, const uint8_t *ref, int mb_index,
                           MV mv) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  const uint8_t *ref_y = ref;
  const uint8_t *ref_u = ref + Y_SIZE;
  const uint8_t *ref_v = ref + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int rx2 = x * 2 + mv.col;
  int ry2 = y * 2 + mv.row;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  int ruvx = uvx + round_div4(mv.col);
  int ruvy = uvy + round_div4(mv.row);
  unsigned int sad = 0;
  int row;
  int col;
  if (((mv.col | mv.row) & 1) == 0) {
    sad += vpx_sad16x16_c(src_y + y * WIDTH + x, WIDTH,
                          ref_y + (ry2 / 2) * WIDTH + (rx2 / 2), WIDTH);
  } else {
    for (row = 0; row < 16; ++row) {
      const uint8_t *s = src_y + (y + row) * WIDTH + x;
      for (col = 0; col < 16; ++col) {
        const int pred =
            sample_halfpel_u8(ref_y, WIDTH, rx2 + col * 2, ry2 + row * 2);
        sad += (unsigned int)abs((int)s[col] - pred);
      }
    }
  }
  sad += 2 * vpx_sad8x8_c(src_u + uvy * UV_W + uvx, UV_W,
                          ref_u + ruvy * UV_W + ruvx, UV_W);
  sad += 2 * vpx_sad8x8_c(src_v + uvy * UV_W + uvx, UV_W,
                          ref_v + ruvy * UV_W + ruvx, UV_W);
  return sad;
}

static void mb_prediction_sse_planes(const uint8_t *src, const uint8_t *ref,
                                     int mb_index, MV mv, uint64_t *sse_y,
                                     uint64_t *sse_u, uint64_t *sse_v) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  const uint8_t *ref_y = ref;
  const uint8_t *ref_u = ref + Y_SIZE;
  const uint8_t *ref_v = ref + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int rx2 = x * 2 + mv.col;
  int ry2 = y * 2 + mv.row;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  int ruvx = uvx + round_div4(mv.col);
  int ruvy = uvy + round_div4(mv.row);
  int row;
  int col;
  *sse_y = 0;
  *sse_u = 0;
  *sse_v = 0;
  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src_y + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred =
          sample_halfpel_u8(ref_y, WIDTH, rx2 + col * 2, ry2 + row * 2);
      const int d = (int)s[col] - pred;
      *sse_y += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src_u + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src_v + (uvy + row) * UV_W + uvx;
    const uint8_t *ru = ref_u + (ruvy + row) * UV_W + ruvx;
    const uint8_t *rv = ref_v + (ruvy + row) * UV_W + ruvx;
    for (col = 0; col < 8; ++col) {
      int d = (int)su[col] - (int)ru[col];
      *sse_u += (uint64_t)(d * d);
      d = (int)sv[col] - (int)rv[col];
      *sse_v += (uint64_t)(d * d);
    }
  }
}

static uint64_t mb_prediction_sse(const uint8_t *src, const uint8_t *ref,
                                  int mb_index, MV mv) {
  uint64_t sse_y;
  uint64_t sse_u;
  uint64_t sse_v;
  mb_prediction_sse_planes(src, ref, mb_index, mv, &sse_y, &sse_u, &sse_v);
  return sse_y + sse_u + sse_v;
}

static int mv_in_bounds_same_eye(int mb_index, MV mv) {
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int x2 = mb_x * 32 + mv.col;
  int y2 = mb_y * 32 + mv.row;
  int min_x2 = mb_x < MB_W / 2 ? 0 : WIDTH;
  int max_x2 = mb_x < MB_W / 2 ? WIDTH - 32 : WIDTH * 2 - 32;
  return x2 >= min_x2 && x2 <= max_x2 && y2 >= 0 &&
         y2 <= HEIGHT * 2 - 32;
}

static MV find_best_mv(const uint8_t *src, const uint8_t *ref, int mb_index,
                       int radius, unsigned int *best_sad) {
  MV best;
  int dy;
  int dx;
  int refine_y;
  int refine_x;
  best.col = 0;
  best.row = 0;
  *best_sad = UINT32_MAX;
  for (dy = -radius; dy <= radius; ++dy) {
    for (dx = -radius; dx <= radius; ++dx) {
      MV mv;
      unsigned int sad;
      mv.col = (int16_t)(dx * 2);
      mv.row = (int16_t)(dy * 2);
      if (!mv_in_bounds_same_eye(mb_index, mv)) continue;
      sad = mb_sad(src, ref, mb_index, mv);
      if (sad < *best_sad) {
        *best_sad = sad;
        best = mv;
      }
    }
  }
  for (refine_y = -1; refine_y <= 1; ++refine_y) {
    for (refine_x = -1; refine_x <= 1; ++refine_x) {
      MV mv;
      unsigned int sad;
      if (refine_x == 0 && refine_y == 0) continue;
      mv.col = (int16_t)(best.col + refine_x);
      mv.row = (int16_t)(best.row + refine_y);
      if (!mv_in_bounds_same_eye(mb_index, mv)) continue;
      sad = mb_sad(src, ref, mb_index, mv);
      if (sad < *best_sad) {
        *best_sad = sad;
        best = mv;
      }
    }
  }
  return best;
}

static void copy_mb_y_from_ref(uint8_t *dst, const uint8_t *ref, int mb_index,
                               MV mv) {
  const uint8_t *ref_y = ref;
  uint8_t *dst_y = dst;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int row;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int rx2 = x * 2 + mv.col;
  int ry2 = y * 2 + mv.row;
  const int rx = rx2 >> 1;
  const int ry = ry2 >> 1;
  const int fx = rx2 & 1;
  const int fy = ry2 & 1;
  if (!fx && !fy) {
    for (row = 0; row < 16; ++row) {
      memcpy(dst_y + (y + row) * WIDTH + x,
             ref_y + (ry + row) * WIDTH + rx, 16);
    }
  } else {
    int col;
    if (fx && !fy) {
      for (row = 0; row < 16; ++row) {
        const uint8_t *s = ref_y + (ry + row) * WIDTH + rx;
        uint8_t *d = dst_y + (y + row) * WIDTH + x;
        for (col = 0; col < 16; ++col) {
          d[col] = avg2_u8(s[col], s[col + 1]);
        }
      }
    } else if (!fx && fy) {
      for (row = 0; row < 16; ++row) {
        const uint8_t *s0 = ref_y + (ry + row) * WIDTH + rx;
        const uint8_t *s1 = s0 + WIDTH;
        uint8_t *d = dst_y + (y + row) * WIDTH + x;
        for (col = 0; col < 16; ++col) {
          d[col] = avg2_u8(s0[col], s1[col]);
        }
      }
    } else {
      for (row = 0; row < 16; ++row) {
        const uint8_t *s0 = ref_y + (ry + row) * WIDTH + rx;
        const uint8_t *s1 = s0 + WIDTH;
        uint8_t *d = dst_y + (y + row) * WIDTH + x;
        for (col = 0; col < 16; ++col) {
          d[col] = (uint8_t)(((int)s0[col] + (int)s0[col + 1] +
                              (int)s1[col] + (int)s1[col + 1] + 2) >>
                             2);
        }
      }
    }
  }
}

static void copy_mb_uv_from_ref(uint8_t *dst, const uint8_t *ref, int mb_index,
                                MV mv) {
  const uint8_t *ref_u = ref + Y_SIZE;
  const uint8_t *ref_v = ref + Y_SIZE + UV_SIZE;
  uint8_t *dst_u = dst + Y_SIZE;
  uint8_t *dst_v = dst + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int row;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  int ruvx = uvx + round_div4(mv.col);
  int ruvy = uvy + round_div4(mv.row);
  for (row = 0; row < 8; ++row) {
    memcpy(dst_u + (uvy + row) * UV_W + uvx,
           ref_u + (ruvy + row) * UV_W + ruvx, 8);
    memcpy(dst_v + (uvy + row) * UV_W + uvx,
           ref_v + (ruvy + row) * UV_W + ruvx, 8);
  }
}

static void copy_mb_from_ref(uint8_t *dst, const uint8_t *ref, int mb_index,
                             MV mv) {
  copy_mb_y_from_ref(dst, ref, mb_index, mv);
  copy_mb_uv_from_ref(dst, ref, mb_index, mv);
}

static uint8_t clip_u8_int(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

static int8_t dc_offset_from_sum(int64_t sum, int count) {
  int offset;
  if (count <= 0) return 0;
  if (sum >= 0) {
    offset = (int)((sum + count / 2) / count);
  } else {
    offset = -(int)((-sum + count / 2) / count);
  }
  if (offset < -128) offset = -128;
  if (offset > 127) offset = 127;
  return (int8_t)offset;
}

static void add_plane_offset(uint8_t *plane, int stride, int x, int y, int w,
                             int h, int offset) {
  int row;
  int col;
  if (offset == 0) return;
  for (row = 0; row < h; ++row) {
    uint8_t *d = plane + (y + row) * stride + x;
    for (col = 0; col < w; ++col) {
      d[col] = clip_u8_int((int)d[col] + offset);
    }
  }
}

static void apply_copy16_dc_to_frame(uint8_t *dst, const uint8_t *ref,
                                     int mb_index, MV mv, int dc_y,
                                     int dc_u, int dc_v) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  copy_mb_from_ref(dst, ref, mb_index, mv);
  add_plane_offset(dst, WIDTH, x, y, 16, 16, dc_y);
  add_plane_offset(dst + Y_SIZE, UV_W, uvx, uvy, 8, 8, dc_u);
  add_plane_offset(dst + Y_SIZE + UV_SIZE, UV_W, uvx, uvy, 8, 8, dc_v);
}

static void apply_copy16_qdc_to_frame(uint8_t *dst, const uint8_t *ref,
                                      int mb_index, MV mv,
                                      const int8_t dc_y4[4], int dc_u,
                                      int dc_v) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  copy_mb_from_ref(dst, ref, mb_index, mv);
  add_plane_offset(dst, WIDTH, x, y, 8, 8, dc_y4[0]);
  add_plane_offset(dst, WIDTH, x + 8, y, 8, 8, dc_y4[1]);
  add_plane_offset(dst, WIDTH, x, y + 8, 8, 8, dc_y4[2]);
  add_plane_offset(dst, WIDTH, x + 8, y + 8, 8, 8, dc_y4[3]);
  add_plane_offset(dst + Y_SIZE, UV_W, uvx, uvy, 8, 8, dc_u);
  add_plane_offset(dst + Y_SIZE + UV_SIZE, UV_W, uvx, uvy, 8, 8, dc_v);
}

static int eval_copy16_dc_candidate(const uint8_t *src, const uint8_t *ref,
                                    int mb_index, MV mv,
                                    const double *plane_gain_scale,
                                    ResCandidate *candidate) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  const uint8_t *ref_u = ref + Y_SIZE;
  const uint8_t *ref_v = ref + Y_SIZE + UV_SIZE;
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int rx2 = x * 2 + mv.col;
  const int ry2 = y * 2 + mv.row;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  const int ruvx = uvx + round_div4(mv.col);
  const int ruvy = uvy + round_div4(mv.row);
  int64_t sum_y = 0;
  int64_t sum_u = 0;
  int64_t sum_v = 0;
  uint64_t pred_sse_y = 0;
  uint64_t pred_sse_u = 0;
  uint64_t pred_sse_v = 0;
  uint64_t recon_sse_y = 0;
  uint64_t recon_sse_u = 0;
  uint64_t recon_sse_v = 0;
  int8_t off_y;
  int8_t off_u;
  int8_t off_v;
  int row;
  int col;

  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src_y + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred =
          sample_halfpel_u8(ref, WIDTH, rx2 + col * 2, ry2 + row * 2);
      const int d = (int)s[col] - pred;
      sum_y += d;
      pred_sse_y += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src_u + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src_v + (uvy + row) * UV_W + uvx;
    const uint8_t *ru = ref_u + (ruvy + row) * UV_W + ruvx;
    const uint8_t *rv = ref_v + (ruvy + row) * UV_W + ruvx;
    for (col = 0; col < 8; ++col) {
      int d = (int)su[col] - (int)ru[col];
      sum_u += d;
      pred_sse_u += (uint64_t)(d * d);
      d = (int)sv[col] - (int)rv[col];
      sum_v += d;
      pred_sse_v += (uint64_t)(d * d);
    }
  }

  off_y = dc_offset_from_sum(sum_y, 16 * 16);
  off_u = dc_offset_from_sum(sum_u, 8 * 8);
  off_v = dc_offset_from_sum(sum_v, 8 * 8);
  if (off_y == 0 && off_u == 0 && off_v == 0) return 0;

  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src_y + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred =
          sample_halfpel_u8(ref, WIDTH, rx2 + col * 2, ry2 + row * 2);
      const int r = clip_u8_int(pred + off_y);
      const int d = (int)s[col] - r;
      recon_sse_y += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src_u + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src_v + (uvy + row) * UV_W + uvx;
    const uint8_t *ru = ref_u + (ruvy + row) * UV_W + ruvx;
    const uint8_t *rv = ref_v + (ruvy + row) * UV_W + ruvx;
    for (col = 0; col < 8; ++col) {
      int d = (int)su[col] - (int)clip_u8_int((int)ru[col] + off_u);
      recon_sse_u += (uint64_t)(d * d);
      d = (int)sv[col] - (int)clip_u8_int((int)rv[col] + off_v);
      recon_sse_v += (uint64_t)(d * d);
    }
  }
  if (recon_sse_y + recon_sse_u + recon_sse_v >=
      pred_sse_y + pred_sse_u + pred_sse_v) {
    return 0;
  }

  memset(candidate, 0, sizeof(*candidate));
  candidate->mb_index = mb_index;
  candidate->gain =
      scaled_gain(pred_sse_y - recon_sse_y,
                  plane_gain_scale ? plane_gain_scale[PATCH_PLANE_Y] : 1.0) +
      repair_score_gain(PATCH_PLANE_U, pred_sse_u - recon_sse_u,
                        plane_gain_scale) +
      repair_score_gain(PATCH_PLANE_V, pred_sse_v - recon_sse_v,
                        plane_gain_scale);
  if (candidate->gain == 0) return 0;
  candidate->pred_sse = pred_sse_y + pred_sse_u + pred_sse_v;
  candidate->recon_sse = recon_sse_y + recon_sse_u + recon_sse_v;
  candidate->cost = COPY16_DC_EXTRA_COST;
  candidate->type = REPAIR_CAND_COPY16_DC;
  candidate->dc_y = off_y;
  candidate->dc_u = off_u;
  candidate->dc_v = off_v;
  return 1;
}

static int __attribute__((unused)) eval_copy16_qdc_candidate(
    const uint8_t *src, const uint8_t *ref, int mb_index, MV mv,
    const double *plane_gain_scale, ResCandidate *candidate) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  const uint8_t *ref_u = ref + Y_SIZE;
  const uint8_t *ref_v = ref + Y_SIZE + UV_SIZE;
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int rx2 = x * 2 + mv.col;
  const int ry2 = y * 2 + mv.row;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  const int ruvx = uvx + round_div4(mv.col);
  const int ruvy = uvy + round_div4(mv.row);
  int64_t sum_y[4] = {0, 0, 0, 0};
  int64_t sum_u = 0;
  int64_t sum_v = 0;
  uint64_t pred_sse_y = 0;
  uint64_t pred_sse_u = 0;
  uint64_t pred_sse_v = 0;
  uint64_t recon_sse_y = 0;
  uint64_t recon_sse_u = 0;
  uint64_t recon_sse_v = 0;
  int8_t off_y[4];
  int8_t off_u;
  int8_t off_v;
  int row;
  int col;
  int q;

  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src_y + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred =
          sample_halfpel_u8(ref, WIDTH, rx2 + col * 2, ry2 + row * 2);
      const int d = (int)s[col] - pred;
      const int slot = (row >= 8 ? 2 : 0) + (col >= 8 ? 1 : 0);
      sum_y[slot] += d;
      pred_sse_y += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src_u + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src_v + (uvy + row) * UV_W + uvx;
    const uint8_t *ru = ref_u + (ruvy + row) * UV_W + ruvx;
    const uint8_t *rv = ref_v + (ruvy + row) * UV_W + ruvx;
    for (col = 0; col < 8; ++col) {
      int d = (int)su[col] - (int)ru[col];
      sum_u += d;
      pred_sse_u += (uint64_t)(d * d);
      d = (int)sv[col] - (int)rv[col];
      sum_v += d;
      pred_sse_v += (uint64_t)(d * d);
    }
  }

  for (q = 0; q < 4; ++q) off_y[q] = dc_offset_from_sum(sum_y[q], 8 * 8);
  off_u = dc_offset_from_sum(sum_u, 8 * 8);
  off_v = dc_offset_from_sum(sum_v, 8 * 8);
  if (off_y[0] == 0 && off_y[1] == 0 && off_y[2] == 0 && off_y[3] == 0 &&
      off_u == 0 && off_v == 0) {
    return 0;
  }

  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src_y + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred =
          sample_halfpel_u8(ref, WIDTH, rx2 + col * 2, ry2 + row * 2);
      const int slot = (row >= 8 ? 2 : 0) + (col >= 8 ? 1 : 0);
      const int r = clip_u8_int(pred + off_y[slot]);
      const int d = (int)s[col] - r;
      recon_sse_y += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src_u + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src_v + (uvy + row) * UV_W + uvx;
    const uint8_t *ru = ref_u + (ruvy + row) * UV_W + ruvx;
    const uint8_t *rv = ref_v + (ruvy + row) * UV_W + ruvx;
    for (col = 0; col < 8; ++col) {
      int d = (int)su[col] - (int)clip_u8_int((int)ru[col] + off_u);
      recon_sse_u += (uint64_t)(d * d);
      d = (int)sv[col] - (int)clip_u8_int((int)rv[col] + off_v);
      recon_sse_v += (uint64_t)(d * d);
    }
  }
  if (recon_sse_y + recon_sse_u + recon_sse_v >=
      pred_sse_y + pred_sse_u + pred_sse_v) {
    return 0;
  }

  memset(candidate, 0, sizeof(*candidate));
  candidate->mb_index = mb_index;
  candidate->gain =
      scaled_gain(pred_sse_y - recon_sse_y,
                  plane_gain_scale ? plane_gain_scale[PATCH_PLANE_Y] : 1.0) +
      repair_score_gain(PATCH_PLANE_U, pred_sse_u - recon_sse_u,
                        plane_gain_scale) +
      repair_score_gain(PATCH_PLANE_V, pred_sse_v - recon_sse_v,
                        plane_gain_scale);
  if (candidate->gain == 0) return 0;
  candidate->pred_sse = pred_sse_y + pred_sse_u + pred_sse_v;
  candidate->recon_sse = recon_sse_y + recon_sse_u + recon_sse_v;
  candidate->cost = COPY16_QDC_EXTRA_COST;
  candidate->type = REPAIR_CAND_COPY16_QDC;
  for (q = 0; q < 4; ++q) candidate->dc_y4[q] = off_y[q];
  candidate->dc_u = off_u;
  candidate->dc_v = off_v;
  return 1;
}

static int intra_dc_value(const uint8_t *frame, int stride, int x, int y, int w,
                          int h, int have_left, int have_top) {
  int sum = 0;
  int count = 0;
  int i;
  if (have_top) {
    const uint8_t *top = frame + (y - 1) * stride + x;
    for (i = 0; i < w; ++i) sum += top[i];
    count += w;
  }
  if (have_left) {
    const uint8_t *left = frame + y * stride + x - 1;
    for (i = 0; i < h; ++i) sum += left[i * stride];
    count += h;
  }
  return count ? (sum + count / 2) / count : 128;
}

static void intra_dc_values(const uint8_t *frame, int mb_index, int *dc_y,
                            int *dc_u, int *dc_v) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int have_left = mb_x > mb_eye_min_x(mb_x);
  const int have_top = mb_y > 0;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  *dc_y = intra_dc_value(frame, WIDTH, x, y, 16, 16, have_left, have_top);
  *dc_u = intra_dc_value(frame + Y_SIZE, UV_W, uvx, uvy, 8, 8, have_left,
                         have_top);
  *dc_v = intra_dc_value(frame + Y_SIZE + UV_SIZE, UV_W, uvx, uvy, 8, 8,
                         have_left, have_top);
}

static int intra_predict_sample(const uint8_t *frame, int stride, int x, int y,
                                int row, int col, int have_left,
                                int have_top, int dc, int mode) {
  if (mode == MODE_INTRA_V && have_top) {
    return frame[(y - 1) * stride + x + col];
  }
  if (mode == MODE_INTRA_H && have_left) {
    return frame[(y + row) * stride + x - 1];
  }
  return dc;
}

static uint64_t intra_mb_sse(const uint8_t *src, const uint8_t *recon,
                             int mb_index, int mode) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int have_left = mb_x > mb_eye_min_x(mb_x);
  const int have_top = mb_y > 0;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  int dc_y;
  int dc_u;
  int dc_v;
  int row;
  int col;
  uint64_t sse = 0;
  intra_dc_values(recon, mb_index, &dc_y, &dc_u, &dc_v);
  for (row = 0; row < 16; ++row) {
    const uint8_t *s = src + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      const int pred = intra_predict_sample(recon, WIDTH, x, y, row, col,
                                            have_left, have_top, dc_y, mode);
      const int d = (int)s[col] - pred;
      sse += (uint64_t)(d * d);
    }
  }
  for (row = 0; row < 8; ++row) {
    const uint8_t *su = src + Y_SIZE + (uvy + row) * UV_W + uvx;
    const uint8_t *sv = src + Y_SIZE + UV_SIZE + (uvy + row) * UV_W + uvx;
    for (col = 0; col < 8; ++col) {
      int pred = intra_predict_sample(recon + Y_SIZE, UV_W, uvx, uvy, row,
                                      col, have_left, have_top, dc_u, mode);
      int d = (int)su[col] - pred;
      sse += (uint64_t)(d * d);
      pred = intra_predict_sample(recon + Y_SIZE + UV_SIZE, UV_W, uvx, uvy,
                                  row, col, have_left, have_top, dc_v, mode);
      d = (int)sv[col] - pred;
      sse += (uint64_t)(d * d);
    }
  }
  return sse;
}

static void predict_intra_mb(uint8_t *dst, int mb_index, int mode) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int have_left = mb_x > mb_eye_min_x(mb_x);
  const int have_top = mb_y > 0;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  int dc_y;
  int dc_u;
  int dc_v;
  int row;
  int col;
  intra_dc_values(dst, mb_index, &dc_y, &dc_u, &dc_v);
  for (row = 0; row < 16; ++row) {
    uint8_t *d = dst + (y + row) * WIDTH + x;
    for (col = 0; col < 16; ++col) {
      d[col] = (uint8_t)intra_predict_sample(dst, WIDTH, x, y, row, col,
                                             have_left, have_top, dc_y, mode);
    }
  }
  for (row = 0; row < 8; ++row) {
    uint8_t *du = dst + Y_SIZE + (uvy + row) * UV_W + uvx;
    uint8_t *dv = dst + Y_SIZE + UV_SIZE + (uvy + row) * UV_W + uvx;
    for (col = 0; col < 8; ++col) {
      du[col] = (uint8_t)intra_predict_sample(dst + Y_SIZE, UV_W, uvx, uvy,
                                              row, col, have_left, have_top,
                                              dc_u, mode);
      dv[col] =
          (uint8_t)intra_predict_sample(dst + Y_SIZE + UV_SIZE, UV_W, uvx,
                                        uvy, row, col, have_left, have_top,
                                        dc_v, mode);
    }
  }
}

static void copy_raw_mb_to_frame(uint8_t *dst, const uint8_t *src,
                                 int mb_index) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  uint8_t *dst_y = dst;
  uint8_t *dst_u = dst + Y_SIZE;
  uint8_t *dst_v = dst + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int row;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  for (row = 0; row < 16; ++row) {
    memcpy(dst_y + (y + row) * WIDTH + x, src_y + (y + row) * WIDTH + x, 16);
  }
  for (row = 0; row < 8; ++row) {
    memcpy(dst_u + (uvy + row) * UV_W + uvx,
           src_u + (uvy + row) * UV_W + uvx, 8);
    memcpy(dst_v + (uvy + row) * UV_W + uvx,
           src_v + (uvy + row) * UV_W + uvx, 8);
  }
}

static void write_raw_mb(Buffer *payload, const uint8_t *src, int mb_index) {
  const uint8_t *src_y = src;
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int row;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  for (row = 0; row < 16; ++row) {
    buffer_bytes(payload, src_y + (y + row) * WIDTH + x, 16);
  }
  for (row = 0; row < 8; ++row) {
    buffer_bytes(payload, src_u + (uvy + row) * UV_W + uvx, 8);
  }
  for (row = 0; row < 8; ++row) {
    buffer_bytes(payload, src_v + (uvy + row) * UV_W + uvx, 8);
  }
}

static void apply_raw_mb_bytes_to_frame(uint8_t *dst, int mb_index,
                                        const uint8_t *p) {
  uint8_t *dst_y = dst;
  uint8_t *dst_u = dst + Y_SIZE;
  uint8_t *dst_v = dst + Y_SIZE + UV_SIZE;
  int mb_x = mb_index % MB_W;
  int mb_y = mb_index / MB_W;
  int row;
  int x = mb_x * 16;
  int y = mb_y * 16;
  int uvx = mb_x * 8;
  int uvy = mb_y * 8;
  for (row = 0; row < 16; ++row) {
    memcpy(dst_y + (y + row) * WIDTH + x, p, 16);
    p += 16;
  }
  for (row = 0; row < 8; ++row) {
    memcpy(dst_u + (uvy + row) * UV_W + uvx, p, 8);
    p += 8;
  }
  for (row = 0; row < 8; ++row) {
    memcpy(dst_v + (uvy + row) * UV_W + uvx, p, 8);
    p += 8;
  }
}

static void copy_raw_y_mb_to_frame(uint8_t *dst, const uint8_t *src,
                                   int mb_index) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  int row;
  for (row = 0; row < 16; ++row) {
    memcpy(dst + (y + row) * WIDTH + x, src + (y + row) * WIDTH + x, 16);
  }
}

static void write_raw_y_mb(Buffer *payload, const uint8_t *src, int mb_index) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  int row;
  for (row = 0; row < 16; ++row) {
    buffer_bytes(payload, src + (y + row) * WIDTH + x, 16);
  }
}

static void apply_raw_y_mb_bytes_to_frame(uint8_t *dst, int mb_index,
                                          const uint8_t *p) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int x = mb_x * 16;
  const int y = mb_y * 16;
  int row;
  for (row = 0; row < 16; ++row) {
    memcpy(dst + (y + row) * WIDTH + x, p, 16);
    p += 16;
  }
}

static void copy_raw_uv_mb_to_frame(uint8_t *dst, const uint8_t *src,
                                    int mb_index) {
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  uint8_t *dst_u = dst + Y_SIZE;
  uint8_t *dst_v = dst + Y_SIZE + UV_SIZE;
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  int row;
  for (row = 0; row < 8; ++row) {
    memcpy(dst_u + (uvy + row) * UV_W + uvx,
           src_u + (uvy + row) * UV_W + uvx, 8);
    memcpy(dst_v + (uvy + row) * UV_W + uvx,
           src_v + (uvy + row) * UV_W + uvx, 8);
  }
}

static void write_raw_uv_mb(Buffer *payload, const uint8_t *src,
                            int mb_index) {
  const uint8_t *src_u = src + Y_SIZE;
  const uint8_t *src_v = src + Y_SIZE + UV_SIZE;
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  int row;
  for (row = 0; row < 8; ++row) {
    buffer_bytes(payload, src_u + (uvy + row) * UV_W + uvx, 8);
  }
  for (row = 0; row < 8; ++row) {
    buffer_bytes(payload, src_v + (uvy + row) * UV_W + uvx, 8);
  }
}

static void apply_raw_uv_mb_bytes_to_frame(uint8_t *dst, int mb_index,
                                           const uint8_t *p) {
  uint8_t *dst_u = dst + Y_SIZE;
  uint8_t *dst_v = dst + Y_SIZE + UV_SIZE;
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  const int uvx = mb_x * 8;
  const int uvy = mb_y * 8;
  int row;
  for (row = 0; row < 8; ++row) {
    memcpy(dst_u + (uvy + row) * UV_W + uvx, p, 8);
    p += 8;
  }
  for (row = 0; row < 8; ++row) {
    memcpy(dst_v + (uvy + row) * UV_W + uvx, p, 8);
    p += 8;
  }
}

static void patch4_geometry(int mb_index, uint8_t plane, uint8_t block,
                            int *base, int *stride, int *x, int *y) {
  const int mb_x = mb_index % MB_W;
  const int mb_y = mb_index / MB_W;
  if (plane == PATCH_PLANE_Y) {
    if (block >= 16) die("bad Y patch block");
    *base = 0;
    *stride = WIDTH;
    *x = mb_x * 16 + (block & 3) * 4;
    *y = mb_y * 16 + (block >> 2) * 4;
  } else if (plane == PATCH_PLANE_U || plane == PATCH_PLANE_V) {
    if (block >= 4) die("bad chroma patch block");
    *base = plane == PATCH_PLANE_U ? Y_SIZE : Y_SIZE + UV_SIZE;
    *stride = UV_W;
    *x = mb_x * 8 + (block & 1) * 4;
    *y = mb_y * 8 + (block >> 1) * 4;
  } else {
    die("bad patch plane");
  }
}

static uint64_t patch4_sse(const uint8_t *src, const uint8_t *ref,
                           int mb_index, MV mv, uint8_t plane,
                           uint8_t block) {
  int src_base;
  int src_stride;
  int src_x;
  int src_y;
  int row;
  int col;
  uint64_t sse = 0;
  patch4_geometry(mb_index, plane, block, &src_base, &src_stride, &src_x,
                  &src_y);
  if (plane == PATCH_PLANE_Y) {
    const int ref_x2 = src_x * 2 + mv.col;
    const int ref_y2 = src_y * 2 + mv.row;
    for (row = 0; row < 4; ++row) {
      const uint8_t *s = src + src_base + (src_y + row) * src_stride + src_x;
      for (col = 0; col < 4; ++col) {
        const int pred =
            sample_halfpel_u8(ref, WIDTH, ref_x2 + col * 2,
                              ref_y2 + row * 2);
        const int d = (int)s[col] - pred;
        sse += (uint64_t)(d * d);
      }
    }
  } else {
    const int ref_base = plane == PATCH_PLANE_U ? Y_SIZE : Y_SIZE + UV_SIZE;
    const int ref_x = src_x + round_div4(mv.col);
    const int ref_y = src_y + round_div4(mv.row);
    for (row = 0; row < 4; ++row) {
      const uint8_t *s = src + src_base + (src_y + row) * src_stride + src_x;
      const uint8_t *r = ref + ref_base + (ref_y + row) * UV_W + ref_x;
      for (col = 0; col < 4; ++col) {
        const int d = (int)s[col] - (int)r[col];
        sse += (uint64_t)(d * d);
      }
    }
  }
  return sse;
}

static void copy_raw4_bytes_to_frame(uint8_t *dst, int mb_index,
                                     const ResBlock *block) {
  const uint8_t *p = block->raw;
  int base;
  int stride;
  int x;
  int y;
  int row;
  patch4_geometry(mb_index, block->plane, block->block, &base, &stride, &x,
                  &y);
  for (row = 0; row < 4; ++row) {
    memcpy(dst + base + (y + row) * stride + x, p, 4);
    p += 4;
  }
}

static void load_raw4_bytes(const uint8_t *src, int mb_index, uint8_t plane,
                            uint8_t block, uint8_t *raw) {
  int base;
  int stride;
  int x;
  int y;
  int row;
  patch4_geometry(mb_index, plane, block, &base, &stride, &x, &y);
  for (row = 0; row < 4; ++row) {
    memcpy(raw + row * 4, src + base + (y + row) * stride + x, 4);
  }
}

static void write_raw4(Buffer *payload, const uint8_t *src, int mb_index,
                       uint8_t plane, uint8_t block) {
  int base;
  int stride;
  int x;
  int y;
  int row;
  patch4_geometry(mb_index, plane, block, &base, &stride, &x, &y);
  for (row = 0; row < 4; ++row) {
    buffer_bytes(payload, src + base + (y + row) * stride + x, 4);
  }
}

static int valid_4x4_block(uint8_t plane, uint8_t block) {
  if (plane == PATCH_PLANE_Y) return block < 16;
  if (plane == PATCH_PLANE_U || plane == PATCH_PLANE_V) return block < 4;
  return 0;
}

static int res_block_slot(uint8_t plane, uint8_t block) {
  if (plane == PATCH_PLANE_Y) return block;
  if (plane == PATCH_PLANE_U) return 16 + block;
  if (plane == PATCH_PLANE_V) return 20 + block;
  die("bad residual plane");
  return 0;
}

static void res_slot_to_plane_block(uint8_t slot, uint8_t *plane,
                                    uint8_t *block) {
  if (slot < 16) {
    *plane = PATCH_PLANE_Y;
    *block = slot;
  } else if (slot < 20) {
    *plane = PATCH_PLANE_U;
    *block = (uint8_t)(slot - 16);
  } else if (slot < RES_BLOCKS_PER_MB) {
    *plane = PATCH_PLANE_V;
    *block = (uint8_t)(slot - 20);
  } else {
    die("bad residual slot");
  }
}

static uint64_t scaled_gain(uint64_t gain, double scale) {
  const double v = (double)gain * scale;
  if (v >= (double)UINT64_MAX) return UINT64_MAX;
  return (uint64_t)(v + 0.5);
}

static uint64_t repair_score_gain(uint8_t plane, uint64_t gain,
                                  const double *plane_gain_scale) {
  double scale = 1.0;
  if (plane == PATCH_PLANE_U) {
    scale = (double)CHROMA_REPAIR_BIAS_U;
  } else if (plane == PATCH_PLANE_V) {
    scale = (double)CHROMA_REPAIR_BIAS_V;
  }
  if (plane_gain_scale) scale *= plane_gain_scale[plane];
  return scaled_gain(gain, scale);
}

static int coeff_q_step(int res_q, int coeff) {
  int q = coeff == 0 ? (res_q + 1) / 2 : res_q;
  return q < 1 ? 1 : q;
}

static int plane_res_q(int res_q, uint8_t plane) {
  if (plane == PATCH_PLANE_U || plane == PATCH_PLANE_V) {
    const int chroma_q = res_q - 2;
    return chroma_q < 1 ? 1 : chroma_q;
  }
  return res_q;
}

static int coeff_q_step_plane(int res_q, int coeff, uint8_t plane) {
  return coeff_q_step(plane_res_q(res_q, plane), coeff);
}

static int quantize_i8(short coeff, int q_step) {
  int q;
  if (coeff >= 0) {
    q = ((int)coeff + q_step / 2) / q_step;
  } else {
    q = -((-(int)coeff + q_step / 2) / q_step);
  }
  if (q > 127) return 127;
  if (q < -127) return -127;
  return q;
}

static int eval_res4_candidate(const uint8_t *src, const uint8_t *ref,
                               int mb_index, MV mv, uint8_t plane,
                               uint8_t block, int res_q,
                               const double *plane_gain_scale,
                               ResCandidate *candidate) {
  int src_base;
  int src_stride;
  int src_x;
  int src_y;
  int ref_base = 0;
  int ref_stride = WIDTH;
  int ref_x = 0;
  int ref_y = 0;
  int ref_x2 = 0;
  int ref_y2 = 0;
  int row;
  int col;
  int z;
  short diff[16];
  short coeff[16];
  short deq[16];
  uint8_t pred[16];
  uint8_t recon[16];
  uint64_t pred_sse = 0;
  uint64_t recon_sse = 0;
  uint16_t mask = 0;
  uint8_t nz = 0;
  int packed_coeffs = 1;

  patch4_geometry(mb_index, plane, block, &src_base, &src_stride, &src_x,
                  &src_y);
  if (plane == PATCH_PLANE_Y) {
    ref_x2 = src_x * 2 + mv.col;
    ref_y2 = src_y * 2 + mv.row;
  } else {
    ref_base = plane == PATCH_PLANE_U ? Y_SIZE : Y_SIZE + UV_SIZE;
    ref_stride = UV_W;
    ref_x = src_x + round_div4(mv.col);
    ref_y = src_y + round_div4(mv.row);
  }
  for (row = 0; row < 4; ++row) {
    const uint8_t *s = src + src_base + (src_y + row) * src_stride + src_x;
    for (col = 0; col < 4; ++col) {
      const int idx = row * 4 + col;
      const int p = plane == PATCH_PLANE_Y
                        ? sample_halfpel_u8(ref, WIDTH, ref_x2 + col * 2,
                                            ref_y2 + row * 2)
                        : ref[ref_base + (ref_y + row) * ref_stride + ref_x +
                              col];
      const int d = (int)s[col] - p;
      pred[idx] = (uint8_t)p;
      diff[idx] = (short)d;
      pred_sse += (uint64_t)(d * d);
    }
  }

  vp8_short_fdct4x4_c(diff, coeff, 8);
  memset(deq, 0, sizeof(deq));
  memset(candidate->block.qcoeff, 0, sizeof(candidate->block.qcoeff));
  for (z = 0; z < 16; ++z) {
    const int rc = vp8_zigzag[z];
    const int q =
        quantize_i8(coeff[rc], coeff_q_step_plane(res_q, rc, plane));
    if (q != 0) {
      mask |= (uint16_t)(1u << z);
      candidate->block.qcoeff[rc] = (int8_t)q;
      deq[rc] = (short)(q * coeff_q_step_plane(res_q, rc, plane));
      ++nz;
      if (q < -8 || q > 7) packed_coeffs = 0;
    }
  }
  if (nz == 0) return 0;

  if (mask == 1) {
    vp8_dc_only_idct_add_c(deq[0], pred, 4, recon, 4);
  } else {
    vp8_short_idct4x4llm_c(deq, pred, 4, recon, 4);
  }

  for (row = 0; row < 4; ++row) {
    const uint8_t *s = src + src_base + (src_y + row) * src_stride + src_x;
    for (col = 0; col < 4; ++col) {
      const int d = (int)s[col] - (int)recon[row * 4 + col];
      recon_sse += (uint64_t)(d * d);
    }
  }
  if (recon_sse >= pred_sse) return 0;

  candidate->mb_index = mb_index;
  candidate->block.type = REPAIR_CAND_RES4;
  candidate->block.plane = plane;
  candidate->block.block = block;
  candidate->block.nz = nz;
  candidate->block.packed_coeffs = (uint8_t)packed_coeffs;
  candidate->block.coeff_mask = mask;
  candidate->gain =
      repair_score_gain(plane, pred_sse - recon_sse, plane_gain_scale);
  candidate->pred_sse = pred_sse;
  candidate->recon_sse = recon_sse;
  candidate->cost = res4_encoded_cost(mask, (uint8_t)packed_coeffs, nz);
  candidate->block.cost = candidate->cost;
  candidate->block.gain = candidate->gain;
  candidate->type = REPAIR_CAND_RES4;
  return 1;
}

static int eval_raw4_candidate(const uint8_t *src, const uint8_t *ref,
                               int mb_index, MV mv, uint8_t plane,
                               uint8_t block,
                               const double *plane_gain_scale,
                               ResCandidate *candidate) {
  const uint64_t sse = patch4_sse(src, ref, mb_index, mv, plane, block);
  if (sse == 0) return 0;
  memset(candidate, 0, sizeof(*candidate));
  candidate->mb_index = mb_index;
  candidate->block.type = REPAIR_CAND_RAW4;
  candidate->block.plane = plane;
  candidate->block.block = block;
  load_raw4_bytes(src, mb_index, plane, block, candidate->block.raw);
  candidate->gain = repair_score_gain(plane, sse, plane_gain_scale);
  candidate->pred_sse = sse;
  candidate->recon_sse = 0;
  candidate->cost = 1 + RAW_4X4_BYTES;
  candidate->block.cost = candidate->cost;
  candidate->block.gain = candidate->gain;
  candidate->type = REPAIR_CAND_RAW4;
  return 1;
}

static int collect_res_candidates(const uint8_t *src, const uint8_t *ref,
                                  int mb_index, MV mv, int res_q,
                                  const double *plane_gain_scale,
                                  ResCandidate *candidates) {
  int count = 0;
  uint8_t block;
  for (block = 0; block < 16; ++block) {
    count += eval_raw4_candidate(src, ref, mb_index, mv, PATCH_PLANE_Y, block,
                                 plane_gain_scale, candidates + count);
    count += eval_res4_candidate(src, ref, mb_index, mv, PATCH_PLANE_Y, block,
                                 res_q, plane_gain_scale, candidates + count);
  }
  for (block = 0; block < 4; ++block) {
    count += eval_raw4_candidate(src, ref, mb_index, mv, PATCH_PLANE_U, block,
                                 plane_gain_scale, candidates + count);
    count += eval_res4_candidate(src, ref, mb_index, mv, PATCH_PLANE_U, block,
                                 res_q, plane_gain_scale, candidates + count);
    count += eval_raw4_candidate(src, ref, mb_index, mv, PATCH_PLANE_V, block,
                                 plane_gain_scale, candidates + count);
    count += eval_res4_candidate(src, ref, mb_index, mv, PATCH_PLANE_V, block,
                                 res_q, plane_gain_scale, candidates + count);
  }
  return count;
}

static void apply_res4_col0_to_frame(uint8_t *dst, int base, int stride, int x,
                                     int y, const ResBlock *block,
                                     int block_res_q) {
  const int qdc = coeff_q_step(block_res_q, 0);
  const int qac = coeff_q_step(block_res_q, 4);
  const int in0 = (block->coeff_mask & 0x0001)
                      ? block->qcoeff[0] * qdc
                      : 0;
  const int in4 = (block->coeff_mask & 0x0004)
                      ? block->qcoeff[4] * qac
                      : 0;
  const int in8 = (block->coeff_mask & 0x0008)
                      ? block->qcoeff[8] * qac
                      : 0;
  const int in12 = (block->coeff_mask & 0x0200)
                       ? block->qcoeff[12] * qac
                       : 0;
  const int a1 = in0 + in8;
  const int b1 = in0 - in8;
  int temp1 = (in4 * IDCT_SINPI8SQRT2) >> 16;
  int temp2 = in12 + ((in12 * IDCT_COSPI8SQRT2MINUS1) >> 16);
  const int c1 = temp1 - temp2;
  short row_out[4];
  int row;
  int col;
  temp1 = in4 + ((in4 * IDCT_COSPI8SQRT2MINUS1) >> 16);
  temp2 = (in12 * IDCT_SINPI8SQRT2) >> 16;
  {
    const int d1 = temp1 + temp2;
    row_out[0] = (short)(a1 + d1);
    row_out[3] = (short)(a1 - d1);
  }
  row_out[1] = (short)(b1 + c1);
  row_out[2] = (short)(b1 - c1);
  for (row = 0; row < 4; ++row) {
    const int add = (row_out[row] + 4) >> 3;
    uint8_t *d = dst + base + (y + row) * stride + x;
    for (col = 0; col < 4; ++col) {
      d[col] = clip_u8_int((int)d[col] + add);
    }
  }
}

static void apply_res4_row0_to_frame(uint8_t *dst, int base, int stride, int x,
                                     int y, const ResBlock *block,
                                     int block_res_q) {
  const int qdc = coeff_q_step(block_res_q, 0);
  const int qac = coeff_q_step(block_res_q, 1);
  const int in0 = (block->coeff_mask & 0x0001)
                      ? block->qcoeff[0] * qdc
                      : 0;
  const int in1 = (block->coeff_mask & 0x0002)
                      ? block->qcoeff[1] * qac
                      : 0;
  const int in2 = (block->coeff_mask & 0x0020)
                      ? block->qcoeff[2] * qac
                      : 0;
  const int in3 = (block->coeff_mask & 0x0040)
                      ? block->qcoeff[3] * qac
                      : 0;
  const int a1 = in0 + in2;
  const int b1 = in0 - in2;
  int temp1 = (in1 * IDCT_SINPI8SQRT2) >> 16;
  int temp2 = in3 + ((in3 * IDCT_COSPI8SQRT2MINUS1) >> 16);
  const int c1 = temp1 - temp2;
  short col_out[4];
  int row;
  int col;
  temp1 = in1 + ((in1 * IDCT_COSPI8SQRT2MINUS1) >> 16);
  temp2 = (in3 * IDCT_SINPI8SQRT2) >> 16;
  {
    const int d1 = temp1 + temp2;
    col_out[0] = (short)((a1 + d1 + 4) >> 3);
    col_out[3] = (short)((a1 - d1 + 4) >> 3);
  }
  col_out[1] = (short)((b1 + c1 + 4) >> 3);
  col_out[2] = (short)((b1 - c1 + 4) >> 3);
  for (row = 0; row < 4; ++row) {
    uint8_t *d = dst + base + (y + row) * stride + x;
    for (col = 0; col < 4; ++col) {
      d[col] = clip_u8_int((int)d[col] + col_out[col]);
    }
  }
}

static void apply_res4_top_left_2x2_to_frame(uint8_t *dst, int base,
                                             int stride, int x, int y,
                                             const ResBlock *block,
                                             int block_res_q) {
  const int qdc = coeff_q_step(block_res_q, 0);
  const int qac = coeff_q_step(block_res_q, 1);
  const int in0 = (block->coeff_mask & 0x0001)
                      ? block->qcoeff[0] * qdc
                      : 0;
  const int in1 = (block->coeff_mask & 0x0002)
                      ? block->qcoeff[1] * qac
                      : 0;
  const int in4 = (block->coeff_mask & 0x0004)
                      ? block->qcoeff[4] * qac
                      : 0;
  const int in5 = (block->coeff_mask & 0x0010)
                      ? block->qcoeff[5] * qac
                      : 0;
  short col0[4];
  short col1[4];
  int row;

  {
    const int a1 = in0;
    const int b1 = in0;
    const int c1 = (in4 * IDCT_SINPI8SQRT2) >> 16;
    const int d1 = in4 + ((in4 * IDCT_COSPI8SQRT2MINUS1) >> 16);
    col0[0] = (short)(a1 + d1);
    col0[3] = (short)(a1 - d1);
    col0[1] = (short)(b1 + c1);
    col0[2] = (short)(b1 - c1);
  }
  {
    const int a1 = in1;
    const int b1 = in1;
    const int c1 = (in5 * IDCT_SINPI8SQRT2) >> 16;
    const int d1 = in5 + ((in5 * IDCT_COSPI8SQRT2MINUS1) >> 16);
    col1[0] = (short)(a1 + d1);
    col1[3] = (short)(a1 - d1);
    col1[1] = (short)(b1 + c1);
    col1[2] = (short)(b1 - c1);
  }

  for (row = 0; row < 4; ++row) {
    const int a1 = col0[row];
    const int b1 = col0[row];
    const int c1 = (col1[row] * IDCT_SINPI8SQRT2) >> 16;
    const int d1 =
        col1[row] + ((col1[row] * IDCT_COSPI8SQRT2MINUS1) >> 16);
    uint8_t *d = dst + base + (y + row) * stride + x;
    d[0] = clip_u8_int((int)d[0] + ((a1 + d1 + 4) >> 3));
    d[3] = clip_u8_int((int)d[3] + ((a1 - d1 + 4) >> 3));
    d[1] = clip_u8_int((int)d[1] + ((b1 + c1 + 4) >> 3));
    d[2] = clip_u8_int((int)d[2] + ((b1 - c1 + 4) >> 3));
  }
}

static void apply_res4_to_frame(uint8_t *dst, int mb_index,
                                const ResBlock *block, int res_q) {
  int base;
  int stride;
  int x;
  int y;
  int z;
  int block_res_q;
  short deq[16];
  if (block->type == REPAIR_CAND_RAW4) {
    copy_raw4_bytes_to_frame(dst, mb_index, block);
    return;
  }
  patch4_geometry(mb_index, block->plane, block->block, &base, &stride, &x,
                  &y);
  block_res_q = plane_res_q(res_q, block->plane);
  if (block->coeff_mask == 1) {
    const short dc = (short)(block->qcoeff[0] * coeff_q_step(block_res_q, 0));
    vp8_dc_only_idct_add_c(dc, dst + base + y * stride + x, stride,
                           dst + base + y * stride + x, stride);
    return;
  }
  if ((block->coeff_mask & (uint16_t)~0x020d) == 0) {
    apply_res4_col0_to_frame(dst, base, stride, x, y, block, block_res_q);
    return;
  }
  if ((block->coeff_mask & (uint16_t)~0x0063) == 0) {
    apply_res4_row0_to_frame(dst, base, stride, x, y, block, block_res_q);
    return;
  }
  if ((block->coeff_mask & (uint16_t)~0x0017) == 0) {
    apply_res4_top_left_2x2_to_frame(dst, base, stride, x, y, block,
                                     block_res_q);
    return;
  }
  memset(deq, 0, sizeof(deq));
  for (z = 0; z < 16; ++z) {
    if (block->coeff_mask & (uint16_t)(1u << z)) {
      const int rc = vp8_zigzag[z];
      deq[rc] = (short)(block->qcoeff[rc] * coeff_q_step(block_res_q, rc));
    }
  }
  vp8_short_idct4x4llm_c(deq, dst + base + y * stride + x, stride,
                         dst + base + y * stride + x, stride);
}

static void write_repair4(Buffer *payload, const uint8_t *src, int mb_index,
                          const ResBlock *block) {
  const uint8_t slot = (uint8_t)res_block_slot(block->plane, block->block);
  int z;
  if (block->type == REPAIR_CAND_RAW4) {
    buffer_u8(payload, (uint8_t)(REPAIR_RAW4_FLAG | slot));
    write_raw4(payload, src, mb_index, block->plane, block->block);
    return;
  }
  if (block->coeff_mask == 1) {
    buffer_u8(payload, (uint8_t)(REPAIR_DC4_FLAG | slot));
    buffer_u8(payload, (uint8_t)block->qcoeff[0]);
    return;
  }
  if (block->packed_coeffs) {
    const int table_index = res4_mask_table_index(block->coeff_mask);
    if (table_index >= 0) {
      uint8_t packed = 0;
      int have_low = 0;
      int wrote_table_byte = 0;
      buffer_u8(payload, (uint8_t)(REPAIR_TABLE_FLAG | slot));
      for (z = 0; z < 16; ++z) {
        if (block->coeff_mask & (uint16_t)(1u << z)) {
          const int rc = vp8_zigzag[z];
          const uint8_t nibble = qcoeff_to_nibble(block->qcoeff[rc]);
          if (!wrote_table_byte) {
            buffer_u8(payload,
                      (uint8_t)(((uint8_t)table_index << 4) | nibble));
            wrote_table_byte = 1;
          } else if (!have_low) {
            packed = nibble;
            have_low = 1;
          } else {
            buffer_u8(payload, (uint8_t)(packed | (nibble << 4)));
            have_low = 0;
          }
        }
      }
      if (have_low) buffer_u8(payload, packed);
      return;
    }
  }
  buffer_u8(payload, (uint8_t)(slot |
                               (block->packed_coeffs ? REPAIR_NIBBLE_FLAG : 0)));
  buffer_le16(payload, block->coeff_mask);
  if (block->packed_coeffs) {
    uint8_t packed = 0;
    int have_low = 0;
    for (z = 0; z < 16; ++z) {
      if (block->coeff_mask & (uint16_t)(1u << z)) {
        const int rc = vp8_zigzag[z];
        const uint8_t nibble = qcoeff_to_nibble(block->qcoeff[rc]);
        if (!have_low) {
          packed = nibble;
          have_low = 1;
        } else {
          buffer_u8(payload, (uint8_t)(packed | (nibble << 4)));
          have_low = 0;
        }
      }
    }
    if (have_low) buffer_u8(payload, packed);
  } else {
    for (z = 0; z < 16; ++z) {
      if (block->coeff_mask & (uint16_t)(1u << z)) {
        const int rc = vp8_zigzag[z];
        buffer_u8(payload, (uint8_t)block->qcoeff[rc]);
      }
    }
  }
}

static int read_res4(O3vpxReader *reader, ResBlock *block) {
  int z;
  int nz = 0;
  uint8_t plane_type;
  uint8_t slot;
  int table_coded;
  plane_type = get_u8(reader);
  table_coded = ((plane_type & REPAIR_TABLE_FLAG) == REPAIR_TABLE_FLAG);
  block->type =
      (plane_type & REPAIR_RAW4_FLAG) ? REPAIR_CAND_RAW4 : REPAIR_CAND_RES4;
  block->packed_coeffs =
      (uint8_t)((plane_type & REPAIR_NIBBLE_FLAG) ? 1 : 0);
  if ((plane_type & REPAIR_DC4_FLAG) && block->type == REPAIR_CAND_RAW4) {
    die("bad raw4 residual flags");
  }
  if ((plane_type & REPAIR_DC4_FLAG) && block->packed_coeffs &&
      !table_coded) {
    die("bad dc4 residual flags");
  }
  if (block->type == REPAIR_CAND_RAW4 && block->packed_coeffs) {
    die("bad raw4 residual flags");
  }
  slot = (uint8_t)(plane_type & REPAIR_SLOT_MASK);
  res_slot_to_plane_block(slot, &block->plane, &block->block);
  if (!valid_4x4_block(block->plane, block->block)) die("bad residual block");
  if (block->type == REPAIR_CAND_RAW4) {
    reader_exact(reader, block->raw, RAW_4X4_BYTES);
    return 1 + RAW_4X4_BYTES;
  }
  if (table_coded) {
    uint8_t table_byte = get_u8(reader);
    int table_index = (int)(table_byte >> 4);
    uint8_t first_nibble = (uint8_t)(table_byte & 0x0f);
    uint8_t packed = 0;
    int packed_nz = 0;
    block->coeff_mask = res4_mask_table[table_index];
    for (z = 0; z < 16; ++z) {
      if (block->coeff_mask & (uint16_t)(1u << z)) {
        const int rc = vp8_zigzag[z];
        uint8_t nibble;
        if (nz == 0) {
          nibble = first_nibble;
        } else {
          if ((packed_nz & 1) == 0) packed = get_u8(reader);
          nibble = (packed_nz & 1) ? (uint8_t)(packed >> 4)
                                   : (uint8_t)(packed & 0x0f);
          ++packed_nz;
        }
        block->qcoeff[rc] = qcoeff_from_nibble(nibble);
        ++nz;
      }
    }
    block->nz = (uint8_t)nz;
    return 2 + (nz >> 1);
  }
  if (plane_type & REPAIR_DC4_FLAG) {
    block->coeff_mask = 1;
    block->qcoeff[0] = (int8_t)get_u8(reader);
    block->nz = 1;
    return 2;
  }
  block->coeff_mask = get_le16(reader);
  if (block->coeff_mask == 0) die("empty residual block");
  if (block->packed_coeffs) {
    uint8_t packed = 0;
    for (z = 0; z < 16; ++z) {
      if (block->coeff_mask & (uint16_t)(1u << z)) {
        const int rc = vp8_zigzag[z];
        uint8_t nibble;
        if ((nz & 1) == 0) packed = get_u8(reader);
        nibble = (nz & 1) ? (uint8_t)(packed >> 4) : (uint8_t)(packed & 0x0f);
        block->qcoeff[rc] = qcoeff_from_nibble(nibble);
        ++nz;
      }
    }
  } else {
    for (z = 0; z < 16; ++z) {
      if (block->coeff_mask & (uint16_t)(1u << z)) {
        const int rc = vp8_zigzag[z];
        block->qcoeff[rc] = (int8_t)get_u8(reader);
        ++nz;
      }
    }
  }
  block->nz = (uint8_t)nz;
  return 3 + (block->packed_coeffs ? ((nz + 1) >> 1) : nz);
}

static void count_res4_block(O3vpxFrameInfo *info, const ResBlock *block) {
  if (!info || !block) return;
  if (block->type == REPAIR_CAND_RAW4) {
    info->raw4_blocks++;
    return;
  }
  info->residual_blocks++;
  if (block->coeff_mask == 1) {
    info->dc_only_blocks++;
  } else if ((block->coeff_mask & (uint16_t)~0x020d) != 0 &&
             (block->coeff_mask & (uint16_t)~0x0063) != 0 &&
             (block->coeff_mask & (uint16_t)~0x0017) != 0) {
    info->full_idct_blocks++;
  }
}

static void count_mv_mb(O3vpxFrameInfo *info, MV mv) {
  if (!info) return;
  if ((mv.col & 1) || (mv.row & 1)) {
    info->halfpel_mb++;
  }
}

static uint64_t frame_sse(const uint8_t *src, const uint8_t *recon) {
  uint64_t sse = 0;
  size_t i;
  for (i = 0; i < FRAME_SIZE; ++i) {
    int d = (int)src[i] - (int)recon[i];
    sse += (uint64_t)(d * d);
  }
  return sse;
}

static double frame_psnr(uint64_t sse) {
  if (sse == 0) return INFINITY;
  return 10.0 * log10((255.0 * 255.0 * FRAME_SIZE) / (double)sse);
}

static double sequence_psnr(uint64_t sse, int frames) {
  if (sse == 0) return INFINITY;
  return 10.0 *
         log10((255.0 * 255.0 * FRAME_SIZE * frames) / (double)sse);
}

static double luma_mse(const uint8_t *a, const uint8_t *b) {
  uint64_t sse = 0;
  int i;
  for (i = 0; i < Y_SIZE; ++i) {
    int d = (int)a[i] - (int)b[i];
    sse += (uint64_t)(d * d);
  }
  return (double)sse / (double)Y_SIZE;
}

static int build_key_plan(FILE *in, int frames, int keyint,
                          double scene_mse_threshold, uint8_t *key_flags,
                          double *mse_by_frame) {
  uint8_t *prev = (uint8_t *)xmalloc(FRAME_SIZE);
  uint8_t *cur = (uint8_t *)xmalloc(FRAME_SIZE);
  int frame_no;
  int key_count = 0;
  int last_key = 0;
  memset(key_flags, 0, (size_t)frames);
  memset(mse_by_frame, 0, sizeof(*mse_by_frame) * (size_t)frames);
  if (fseek(in, 0, SEEK_SET) != 0) die_errno("seek failed");
  for (frame_no = 0; frame_no < frames; ++frame_no) {
    if (fread(cur, 1, FRAME_SIZE, in) != FRAME_SIZE) die("short input frame");
    if (frame_no > 0) {
      mse_by_frame[frame_no] = luma_mse(cur, prev);
    }
    memcpy(prev, cur, FRAME_SIZE);
  }
  for (frame_no = 0; frame_no < frames; ++frame_no) {
    const int scene_key =
        frame_no > 0 && scene_mse_threshold > 0.0 &&
        mse_by_frame[frame_no] >= scene_mse_threshold;
    int max_interval_key =
        frame_no > 0 && keyint > 0 && frame_no - last_key >= keyint;
    int is_key = frame_no == 0 || scene_key || max_interval_key;
    if (max_interval_key && !scene_key && scene_mse_threshold > 0.0) {
      int lookahead;
      for (lookahead = 1; lookahead <= KEY_SCENE_LOOKAHEAD; ++lookahead) {
        const int future_frame = frame_no + lookahead;
        if (future_frame >= frames) break;
        if (mse_by_frame[future_frame] >= scene_mse_threshold) {
          max_interval_key = 0;
          is_key = 0;
          break;
        }
      }
    }
    if (is_key) {
      key_flags[frame_no] = 1;
      last_key = frame_no;
      ++key_count;
      fprintf(stderr, "key_plan frame=%d reason=%s temporal_luma_mse=%.2f\n",
              frame_no,
              frame_no == 0 ? "first" : (scene_key ? "scene" : "max_interval"),
              mse_by_frame[frame_no]);
    }
  }
  if (fseek(in, 0, SEEK_SET) != 0) die_errno("seek failed");
  free(prev);
  free(cur);
  return key_count;
}

static int cmp_res_gain_per_byte_desc(const void *a, const void *b) {
  const ResCandidate *ra = (const ResCandidate *)a;
  const ResCandidate *rb = (const ResCandidate *)b;
  const uint64_t left = ra->gain * (uint64_t)rb->cost;
  const uint64_t right = rb->gain * (uint64_t)ra->cost;
  if (left != right) return left < right ? 1 : -1;
  if (ra->gain != rb->gain) return ra->gain < rb->gain ? 1 : -1;
  if (ra->mb_index != rb->mb_index) return ra->mb_index - rb->mb_index;
  if (ra->block.plane != rb->block.plane) {
    return (int)ra->block.plane - (int)rb->block.plane;
  }
  return (int)ra->block.block - (int)rb->block.block;
}

static int cmp_mb_index_asc(const void *a, const void *b) {
  const MbAnalysis *ma = (const MbAnalysis *)a;
  const MbAnalysis *mb = (const MbAnalysis *)b;
  return ma->index - mb->index;
}

static double clamp_double(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int parse_metric_token(const char *token, const char *prefix,
                              double *value) {
  const size_t len = strlen(prefix);
  if (strncmp(token, prefix, len) != 0) return 0;
  *value = strtod(token + len, NULL);
  return 1;
}

static void init_quality_metrics(QualityMetrics *metrics, int frames) {
  int i;
  for (i = 0; i < frames; ++i) {
    metrics[i].mse_avg = -1.0;
    metrics[i].mse_y = -1.0;
    metrics[i].mse_u = -1.0;
    metrics[i].mse_v = -1.0;
    metrics[i].valid = 0;
  }
}

static int read_quality_metrics(const char *path, int frames,
                                QualityMetrics *metrics) {
  FILE *f;
  char line[512];
  int loaded = 0;
  if (!path || !path[0]) return 0;
  f = fopen(path, "r");
  if (!f) die_errno("open PSNR log failed");
  init_quality_metrics(metrics, frames);
  while (fgets(line, sizeof(line), f)) {
    int n = -1;
    double mse_avg = -1.0;
    double mse_y = -1.0;
    double mse_u = -1.0;
    double mse_v = -1.0;
    char *token = strtok(line, " \t\r\n");
    while (token) {
      if (strncmp(token, "n:", 2) == 0) {
        n = atoi(token + 2);
      } else if (!parse_metric_token(token, "mse_avg:", &mse_avg) &&
                 !parse_metric_token(token, "mse_y:", &mse_y) &&
                 !parse_metric_token(token, "mse_u:", &mse_u)) {
        (void)parse_metric_token(token, "mse_v:", &mse_v);
      }
      token = strtok(NULL, " \t\r\n");
    }
    if (n > 0 && n <= frames && mse_avg >= 0.0 && mse_y >= 0.0 &&
        mse_u >= 0.0 && mse_v >= 0.0) {
      QualityMetrics *m = &metrics[n - 1];
      m->mse_avg = mse_avg;
      m->mse_y = mse_y;
      m->mse_u = mse_u;
      m->mse_v = mse_v;
      m->valid = 1;
      ++loaded;
    }
  }
  fclose(f);
  return loaded;
}

static double safe_mse_ratio(double previous_mse, double baseline_mse) {
  const double eps = 0.000001;
  if (previous_mse <= eps && baseline_mse <= eps) return 1.0;
  if (previous_mse <= eps) return 0.25;
  if (baseline_mse <= eps) return QUALITY_GAIN_SCALE_MAX;
  return previous_mse / baseline_mse;
}

static double quality_deficit_ratio(const QualityMetrics *baseline,
                                    const QualityMetrics *previous) {
  double avg_ratio;
  double y_ratio;
  double u_ratio;
  double v_ratio;
  double chroma_ratio;
  if (!baseline->valid || !previous->valid) return 1.0;
  avg_ratio = safe_mse_ratio(previous->mse_avg, baseline->mse_avg);
  y_ratio = safe_mse_ratio(previous->mse_y, baseline->mse_y);
  u_ratio = safe_mse_ratio(previous->mse_u, baseline->mse_u);
  v_ratio = safe_mse_ratio(previous->mse_v, baseline->mse_v);
  chroma_ratio = u_ratio > v_ratio ? u_ratio : v_ratio;
  return 0.50 * avg_ratio + 0.15 * y_ratio + 0.35 * chroma_ratio;
}

static int build_quality_bias(const char *baseline_path,
                              const char *previous_path, int frames,
                              double *budget_bias, double *gain_scale,
                              double *plane_gain_scale) {
  QualityMetrics *baseline;
  QualityMetrics *previous;
  int baseline_count;
  int previous_count;
  int active = 0;
  int i;
  for (i = 0; i < frames; ++i) {
    budget_bias[i] = 1.0;
    gain_scale[i] = 1.0;
    plane_gain_scale[i * 3 + PATCH_PLANE_Y] = 1.0;
    plane_gain_scale[i * 3 + PATCH_PLANE_U] = 1.0;
    plane_gain_scale[i * 3 + PATCH_PLANE_V] = 1.0;
  }
  if (!baseline_path || !previous_path) return 0;
  baseline = (QualityMetrics *)xmalloc(sizeof(*baseline) * (size_t)frames);
  previous = (QualityMetrics *)xmalloc(sizeof(*previous) * (size_t)frames);
  baseline_count = read_quality_metrics(baseline_path, frames, baseline);
  previous_count = read_quality_metrics(previous_path, frames, previous);
  for (i = 0; i < frames; ++i) {
    const double ratio = quality_deficit_ratio(&baseline[i], &previous[i]);
    const double scale = sqrt(clamp_double(ratio, 0.10, 12.0));
    double y_scale = 1.0;
    double u_scale = 1.0;
    double v_scale = 1.0;
    budget_bias[i] = clamp_double(scale, QUALITY_BUDGET_BIAS_MIN,
                                  QUALITY_BUDGET_BIAS_MAX);
    gain_scale[i] = clamp_double(scale, QUALITY_GAIN_SCALE_MIN,
                                 QUALITY_GAIN_SCALE_MAX);
    if (baseline[i].valid && previous[i].valid) {
      y_scale = sqrt(sqrt(clamp_double(
          safe_mse_ratio(previous[i].mse_y, baseline[i].mse_y), 0.10, 12.0)));
      u_scale = sqrt(clamp_double(
          safe_mse_ratio(previous[i].mse_u, baseline[i].mse_u), 0.10, 12.0));
      v_scale = sqrt(clamp_double(
          safe_mse_ratio(previous[i].mse_v, baseline[i].mse_v), 0.10, 12.0));
    }
    plane_gain_scale[i * 3 + PATCH_PLANE_Y] =
        clamp_double(y_scale, QUALITY_PLANE_GAIN_MIN, QUALITY_PLANE_GAIN_MAX);
    plane_gain_scale[i * 3 + PATCH_PLANE_U] =
        clamp_double(u_scale, QUALITY_PLANE_GAIN_MIN, QUALITY_PLANE_GAIN_MAX);
    plane_gain_scale[i * 3 + PATCH_PLANE_V] =
        clamp_double(v_scale, QUALITY_PLANE_GAIN_MIN, QUALITY_PLANE_GAIN_MAX);
    if (budget_bias[i] != 1.0 || gain_scale[i] != 1.0 ||
        plane_gain_scale[i * 3 + PATCH_PLANE_Y] != 1.0 ||
        plane_gain_scale[i * 3 + PATCH_PLANE_U] != 1.0 ||
        plane_gain_scale[i * 3 + PATCH_PLANE_V] != 1.0) {
      ++active;
    }
  }
  fprintf(stderr,
          "quality_bias_summary baseline_frames=%d previous_frames=%d "
          "active_frames=%d baseline=%s previous=%s\n",
          baseline_count, previous_count, active, baseline_path, previous_path);
  free(baseline);
  free(previous);
  return active;
}

static double p_budget_weight(double mse, double gop_age) {
  double w = 1.0 + mse / BUDGET_WEIGHT_DEN +
             BUDGET_AGE_WEIGHT * gop_age * gop_age;
  if (w > BUDGET_WEIGHT_CAP) w = BUDGET_WEIGHT_CAP;
  return w;
}

static double p_gop_age(const uint8_t *key_flags, int frames, int frame_no) {
  int prev_key = 0;
  int next_key = frames - 1;
  int i;
  for (i = frame_no - 1; i >= 0; --i) {
    if (key_flags[i]) {
      prev_key = i;
      break;
    }
  }
  for (i = frame_no + 1; i < frames; ++i) {
    if (key_flags[i]) {
      next_key = i;
      break;
    }
  }
  if (next_key <= prev_key) return 0.0;
  return (double)(frame_no - prev_key) / (double)(next_key - prev_key);
}

static size_t sum_future_plan(const size_t *planned_p_budget, int frames,
                              int frame_no) {
  size_t sum = 0;
  int i;
  for (i = frame_no + 1; i < frames; ++i) sum += planned_p_budget[i];
  return sum;
}

static void write_file_header(FILE *out, int frames, double target_mbps,
                              int keyint, int radius, int res_q) {
  if (fwrite(O3VPX_MAGIC, 1, 4, out) != 4) die_errno("write failed");
  put_le16(out, O3VPX_VERSION);
  put_le16(out, WIDTH);
  put_le16(out, HEIGHT);
  put_le16(out, FPS);
  put_le32(out, (uint32_t)frames);
  put_le32(out, (uint32_t)(target_mbps * 1000000.0 + 0.5));
  put_le16(out, (uint16_t)keyint);
  put_le16(out, (uint16_t)radius);
  put_le16(out, (uint16_t)res_q);
}

static void write_frame(FILE *out, int frame_type, const Buffer *payload) {
  put_le16(out, (uint16_t)frame_type);
  put_le32(out, (uint32_t)payload->len);
  if (payload->len &&
      fwrite(payload->data, 1, payload->len, out) != payload->len) {
    die_errno("write failed");
  }
}

static int read_stream_header(O3vpxReader *reader, int *frames, int *res_q) {
  char magic[4];
  uint16_t version;
  uint16_t width;
  uint16_t height;
  uint16_t fps;
  reader_exact(reader, magic, sizeof(magic));
  if (memcmp(magic, O3VPX_MAGIC, 4) != 0) die("bad O3VX magic");
  version = get_le16(reader);
  width = get_le16(reader);
  height = get_le16(reader);
  fps = get_le16(reader);
  *frames = (int)get_le32(reader);
  (void)get_le32(reader);
  (void)get_le16(reader);
  (void)get_le16(reader);
  *res_q = (int)get_le16(reader);
  if (version != O3VPX_VERSION) die("unsupported O3VX version");
  if (width != WIDTH || height != HEIGHT || fps != FPS) {
    die("unsupported O3VX geometry");
  }
  if (*res_q <= 0 || *res_q > RES_MAX_Q) die("bad residual quantizer");
  return 1;
}

int vp8_o3vpx_encode_file(const char *in_path, const char *out_path, int frames,
                          int keyint, double target_mbps, int radius,
                          double scene_mse_threshold, int res_q,
                          double min_gain_per_byte, double p_burst_mult,
                          const char *baseline_psnr_path,
                          const char *previous_psnr_path) {
  FILE *in;
  FILE *out;
  int available_frames;
  int frame_no;
  uint8_t *key_flags;
  int key_count;
  int p_count;
  const size_t min_p_payload = MB_COUNT * COPY16_BYTES;
  size_t p_budget;
  size_t total_p_payload_budget;
  size_t remaining_p_payload_budget;
  int remaining_p_frames;
  uint8_t *src = (uint8_t *)xmalloc(FRAME_SIZE);
  uint8_t *ref = (uint8_t *)xmalloc(FRAME_SIZE);
  uint8_t *recon = (uint8_t *)xmalloc(FRAME_SIZE);
  size_t total_bytes = 4 + 2 + 2 + 2 + 2 + 4 + 4 + 2 + 2 + 2;
  uint64_t total_sse = 0;
  double t0;
  double elapsed;
  double *mse_by_frame;
  size_t *planned_p_budget;
  double *quality_budget_bias;
  double *quality_gain_scale;
  double *quality_plane_gain_scale;
  int quality_bias_active;

  in = fopen(in_path, "rb");
  if (!in) die_errno("open input failed");
  available_frames = (int)(file_size(in) / FRAME_SIZE);
  if (frames == 0) frames = available_frames;
  if (frames <= 0 || frames > available_frames) die("bad frame count");
  if (keyint <= 0) die("bad keyint");
  if (target_mbps <= 0.0) die("bad target_mbps");
  if (radius < 0 || radius > 32) die("bad search radius");
  if (scene_mse_threshold < 0.0) die("bad scene_mse_threshold");
  if (res_q <= 0 || res_q > RES_MAX_Q) die("bad residual quantizer");
  if (min_gain_per_byte < 0.0) die("bad min_gain_per_byte");
  if (p_burst_mult < 1.0 || p_burst_mult > 8.0) die("bad p_burst_mult");

  key_flags = (uint8_t *)xmalloc((size_t)frames);
  mse_by_frame = (double *)xmalloc(sizeof(*mse_by_frame) * (size_t)frames);
  planned_p_budget =
      (size_t *)xmalloc(sizeof(*planned_p_budget) * (size_t)frames);
  quality_budget_bias =
      (double *)xmalloc(sizeof(*quality_budget_bias) * (size_t)frames);
  quality_gain_scale =
      (double *)xmalloc(sizeof(*quality_gain_scale) * (size_t)frames);
  quality_plane_gain_scale =
      (double *)xmalloc(sizeof(*quality_plane_gain_scale) * (size_t)frames * 3);
  key_count = build_key_plan(in, frames, keyint, scene_mse_threshold, key_flags,
                             mse_by_frame);
  quality_bias_active =
      build_quality_bias(baseline_psnr_path, previous_psnr_path, frames,
                         quality_budget_bias, quality_gain_scale,
                         quality_plane_gain_scale);
  p_count = frames - key_count;
  {
    const double target_bytes =
        target_mbps * 1000000.0 / 8.0 * (double)frames / (double)FPS;
    double total_p_payload_budget_d =
        p_count > 0 ? target_bytes - (double)total_bytes -
                          (double)key_count * (double)(FRAME_SIZE + 6) -
                          (double)p_count * 6.0
                    : 0.0;
    if (total_p_payload_budget_d < 0.0) die("key plan exceeds target bitrate");
    total_p_payload_budget = (size_t)total_p_payload_budget_d;
    p_budget = p_count > 0 ? total_p_payload_budget / (size_t)p_count : 0;
    if (p_count > 0 && p_budget < MB_COUNT * 3) {
      die("key plan leaves too little P-frame budget");
    }
    memset(planned_p_budget, 0, sizeof(*planned_p_budget) * (size_t)frames);
    if (p_count > 0) {
      double weight_sum = 0.0;
      size_t allocated = 0;
      int last_p = -1;
      int i;
      for (i = 0; i < frames; ++i) {
        if (!key_flags[i]) {
          weight_sum += p_budget_weight(mse_by_frame[i],
                                        p_gop_age(key_flags, frames, i)) *
                        quality_budget_bias[i];
          last_p = i;
        }
      }
      if (weight_sum <= 0.0 || last_p < 0) die("bad P-frame weight plan");
      for (i = 0; i < frames; ++i) {
        if (!key_flags[i]) {
          size_t plan;
          if (i == last_p) {
            plan = allocated < total_p_payload_budget
                       ? total_p_payload_budget - allocated
                       : min_p_payload;
          } else {
            const double scaled =
                (double)total_p_payload_budget *
                p_budget_weight(mse_by_frame[i],
                                p_gop_age(key_flags, frames, i)) *
                quality_budget_bias[i] /
                weight_sum;
            plan = (size_t)scaled;
          }
          if (plan < min_p_payload) plan = min_p_payload;
          planned_p_budget[i] = plan;
          allocated += plan;
        }
      }
    }
    remaining_p_payload_budget = total_p_payload_budget;
    remaining_p_frames = p_count;
    fprintf(stderr,
            "key_plan_summary frames=%d keys=%d p_frames=%d target_mbps=%.6f "
            "p_payload_budget=%zu total_p_payload_budget=%zu "
            "scene_mse_threshold=%.2f res_q=%d min_gain_per_byte=%.3f "
            "p_burst_mult=%.3f quality_bias_active=%d\n",
            frames, key_count, p_count, target_mbps, p_budget,
            total_p_payload_budget, scene_mse_threshold, res_q,
            min_gain_per_byte, p_burst_mult, quality_bias_active);
  }

  out = fopen(out_path, "wb");
  if (!out) die_errno("open output failed");
  write_file_header(out, frames, target_mbps, keyint, radius, res_q);
  t0 = now_seconds();

  for (frame_no = 0; frame_no < frames; ++frame_no) {
    Buffer payload;
    int is_key = key_flags[frame_no] != 0;
    uint64_t sse;
    double psnr;
    if (fread(src, 1, FRAME_SIZE, in) != FRAME_SIZE) die("short input frame");
    buffer_init(&payload);
    if (is_key) {
      buffer_bytes(&payload, src, FRAME_SIZE);
      memcpy(recon, src, FRAME_SIZE);
      write_frame(out, FRAME_RAW_KEY, &payload);
      total_bytes += 2 + 4 + payload.len;
      fprintf(stderr, "frame %d: type=raw_key bytes=%zu raw_mb=0 copy16=0\n",
              frame_no, payload.len + 6);
    } else {
      MbAnalysis analysis[MB_COUNT];
      MbResState *res_state =
          (MbResState *)xmalloc(sizeof(*res_state) * MB_COUNT);
      ResCandidate *res_candidates =
          (ResCandidate *)xmalloc(sizeof(*res_candidates) *
                                  REPAIR_CANDIDATE_COUNT);
      ResCandidate *uv_candidates =
          (ResCandidate *)xmalloc(sizeof(*uv_candidates) * MB_COUNT);
      ResCandidate *dc_candidates =
          (ResCandidate *)xmalloc(sizeof(*dc_candidates) * MB_COUNT);
      int res_candidate_count = 0;
      int uv_candidate_count = 0;
      size_t p_frame_budget;
      size_t frame_plan_budget;
      size_t burst_cap;
      size_t future_min_payload;
      size_t estimated_payload = MB_COUNT * 3;
      int raw_mb = 0;
      int raw_y_mb = 0;
      int raw_uv_mb = 0;
      int rawuv_res_mb = 0;
      int intra_dc = 0;
      int intra_v = 0;
      int intra_h = 0;
      int copy16 = 0;
      int copy16_dc = 0;
      int res_mb = 0;
      int res4 = 0;
      int raw4 = 0;
      int res_coeff = 0;
      double avg_remaining_payload;
      double frame_min_gain_per_byte;
      const double *frame_plane_gain_scale =
          quality_plane_gain_scale + frame_no * 3;
      int i;
      if (remaining_p_frames <= 0) die("P-frame budget accounting underflow");
      frame_plan_budget = planned_p_budget[frame_no];
      if (frame_plan_budget < min_p_payload) frame_plan_budget = min_p_payload;
      future_min_payload = sum_future_plan(planned_p_budget, frames, frame_no);
      if (remaining_p_payload_budget < future_min_payload + min_p_payload) {
        future_min_payload = (size_t)(remaining_p_frames - 1) * min_p_payload;
        if (remaining_p_payload_budget < future_min_payload + min_p_payload) {
          die("P-frame budget exhausted");
        }
      }
      avg_remaining_payload =
          (double)remaining_p_payload_budget / (double)remaining_p_frames;
      p_frame_budget = remaining_p_payload_budget - future_min_payload;
      burst_cap = (size_t)((double)p_budget * p_burst_mult);
      if (burst_cap < min_p_payload) burst_cap = min_p_payload;
      if (p_frame_budget > burst_cap) p_frame_budget = burst_cap;
      if (p_frame_budget < min_p_payload) p_frame_budget = min_p_payload;
      frame_min_gain_per_byte = min_gain_per_byte;
      if (quality_bias_active && quality_gain_scale[frame_no] > 0.0) {
        frame_min_gain_per_byte =
            min_gain_per_byte / quality_gain_scale[frame_no];
      }
      memset(res_state, 0, sizeof(*res_state) * MB_COUNT);
      memset(dc_candidates, 0, sizeof(*dc_candidates) * MB_COUNT);
      for (i = 0; i < MB_COUNT; ++i) {
        uint64_t pred_sse_y;
        uint64_t pred_sse_u;
        uint64_t pred_sse_v;
        uint64_t raw_gain;
        uint64_t raw_uv_gain;
        analysis[i].index = i;
        analysis[i].mv = find_best_mv(src, ref, i, radius, &analysis[i].sad);
        analysis[i].raw_mode = 0;
        analysis[i].dc_y = 0;
        analysis[i].dc_u = 0;
        analysis[i].dc_v = 0;
        memset(analysis[i].dc_y4, 0, sizeof(analysis[i].dc_y4));
        mb_prediction_sse_planes(src, ref, i, analysis[i].mv, &pred_sse_y,
                                 &pred_sse_u, &pred_sse_v);
        raw_gain =
            scaled_gain(pred_sse_y,
                        frame_plane_gain_scale[PATCH_PLANE_Y]) +
            repair_score_gain(PATCH_PLANE_U, pred_sse_u,
                              frame_plane_gain_scale) +
            repair_score_gain(PATCH_PLANE_V, pred_sse_v,
                              frame_plane_gain_scale);
        if (raw_gain != 0) {
          ResCandidate *candidate = &res_candidates[res_candidate_count++];
          memset(candidate, 0, sizeof(*candidate));
          candidate->mb_index = i;
          candidate->gain = raw_gain;
          candidate->pred_sse = raw_gain;
          candidate->recon_sse = 0;
          candidate->cost = RAW_MB_EXTRA_COST;
          candidate->type = REPAIR_CAND_RAW_MB;
        }
        if (pred_sse_y != 0) {
          ResCandidate *candidate = &res_candidates[res_candidate_count++];
          memset(candidate, 0, sizeof(*candidate));
          candidate->mb_index = i;
          candidate->gain = scaled_gain(
              pred_sse_y, frame_plane_gain_scale[PATCH_PLANE_Y]);
          candidate->pred_sse = pred_sse_y;
          candidate->recon_sse = 0;
          candidate->cost = RAW_Y_MB_EXTRA_COST;
          candidate->type = REPAIR_CAND_RAW_Y_MB;
        }
        raw_uv_gain =
            repair_score_gain(PATCH_PLANE_U, pred_sse_u,
                              frame_plane_gain_scale) +
            repair_score_gain(PATCH_PLANE_V, pred_sse_v,
                              frame_plane_gain_scale);
        if (raw_uv_gain != 0) {
          ResCandidate *candidate = &uv_candidates[uv_candidate_count++];
          memset(candidate, 0, sizeof(*candidate));
          candidate->mb_index = i;
          candidate->gain = raw_uv_gain;
          candidate->pred_sse = pred_sse_u + pred_sse_v;
          candidate->recon_sse = 0;
          candidate->cost = RAW_UV_MB_EXTRA_COST;
          candidate->type = REPAIR_CAND_RAW_UV_MB;
        }
        (void)eval_copy16_dc_candidate(src, ref, i, analysis[i].mv,
                                       frame_plane_gain_scale,
                                       dc_candidates + i);
        res_candidate_count += collect_res_candidates(
            src, ref, i, analysis[i].mv, res_q,
            frame_plane_gain_scale, res_candidates + res_candidate_count);
      }
      qsort(res_candidates, (size_t)res_candidate_count,
            sizeof(res_candidates[0]), cmp_res_gain_per_byte_desc);
      for (i = 0; i < res_candidate_count; ++i) {
        ResCandidate *candidate = &res_candidates[i];
        MbResState *state = &res_state[candidate->mb_index];
        if (frame_min_gain_per_byte > 0.0 &&
            (double)candidate->gain <
                frame_min_gain_per_byte * (double)candidate->cost) {
          break;
        }
        if (candidate->type == REPAIR_CAND_RAW_MB ||
            candidate->type == REPAIR_CAND_RAW_Y_MB ||
            candidate->type == REPAIR_CAND_RAW_UV_MB ||
            candidate->type == REPAIR_CAND_COPY16_DC) {
          int raw_mode = MODE_RAW_MB;
          size_t replaced_cost = 0;
          uint64_t replaced_gain = 0;
          if (candidate->type == REPAIR_CAND_RAW_Y_MB) {
            raw_mode = MODE_RAW_Y_MB;
          } else if (candidate->type == REPAIR_CAND_RAW_UV_MB) {
            raw_mode = MODE_RAW_UV_MB;
          } else if (candidate->type == REPAIR_CAND_COPY16_DC) {
            raw_mode = MODE_COPY16_DC;
          }
          if (analysis[candidate->mb_index].raw_mode) continue;
          if (state->count != 0) {
            replaced_cost = state->cost;
            replaced_gain = state->gain;
            if (candidate->cost > replaced_cost) continue;
            if (candidate->gain <= replaced_gain) continue;
          } else if (candidate->type != REPAIR_CAND_RAW_MB &&
                     candidate->type != REPAIR_CAND_COPY16_DC) {
            continue;
          }
          if (estimated_payload - replaced_cost + candidate->cost >
              p_frame_budget) {
            continue;
          }
          analysis[candidate->mb_index].raw_mode = raw_mode;
          analysis[candidate->mb_index].dc_y = candidate->dc_y;
          analysis[candidate->mb_index].dc_u = candidate->dc_u;
          analysis[candidate->mb_index].dc_v = candidate->dc_v;
          estimated_payload = estimated_payload - replaced_cost + candidate->cost;
          if (state->count != 0) {
            state->block_mask = 0;
            state->count = 0;
            state->cost = 0;
            state->gain = 0;
          }
        } else {
          const int slot =
              res_block_slot(candidate->block.plane, candidate->block.block);
          const size_t extra_cost =
              (state->count == 0 ? 1u : 0u) + candidate->cost;
          if (analysis[candidate->mb_index].raw_mode) continue;
          if (state->block_mask & (1u << slot)) continue;
          if (estimated_payload + extra_cost > p_frame_budget) continue;
          if (state->count >= RES_BLOCKS_PER_MB) {
            die("too many residual blocks");
          }
          state->block_mask |= (uint32_t)(1u << slot);
          state->blocks[state->count++] = candidate->block;
          state->cost += extra_cost;
          state->gain += candidate->gain;
          estimated_payload += extra_cost;
        }
      }
      qsort(dc_candidates, MB_COUNT, sizeof(dc_candidates[0]),
            cmp_res_gain_per_byte_desc);
      for (i = 0; i < MB_COUNT; ++i) {
        ResCandidate *candidate = &dc_candidates[i];
        MbResState *state;
        size_t replaced_cost = 0;
        uint64_t replaced_gain = 0;
        if (candidate->type != REPAIR_CAND_COPY16_DC &&
            candidate->type != REPAIR_CAND_COPY16_QDC) {
          continue;
        }
        state = &res_state[candidate->mb_index];
        if (analysis[candidate->mb_index].raw_mode) continue;
        if (candidate->type == REPAIR_CAND_COPY16_QDC && state->count != 0) {
          continue;
        }
        if (frame_min_gain_per_byte > 0.0 &&
            (double)candidate->gain <
                frame_min_gain_per_byte * (double)candidate->cost) {
          continue;
        }
        if (state->count != 0) {
          replaced_cost = state->cost;
          replaced_gain = state->gain;
          if (candidate->cost > replaced_cost) continue;
          if (candidate->gain <= replaced_gain) continue;
        }
        if (estimated_payload - replaced_cost + candidate->cost >
            p_frame_budget) {
          continue;
        }
        analysis[candidate->mb_index].raw_mode =
            candidate->type == REPAIR_CAND_COPY16_QDC ? MODE_COPY16_QDC
                                                      : MODE_COPY16_DC;
        analysis[candidate->mb_index].dc_y = candidate->dc_y;
        analysis[candidate->mb_index].dc_u = candidate->dc_u;
        analysis[candidate->mb_index].dc_v = candidate->dc_v;
        memcpy(analysis[candidate->mb_index].dc_y4, candidate->dc_y4,
               sizeof(candidate->dc_y4));
        estimated_payload = estimated_payload - replaced_cost + candidate->cost;
        if (state->count != 0) {
          state->block_mask = 0;
          state->count = 0;
          state->cost = 0;
          state->gain = 0;
        }
      }
      qsort(uv_candidates, (size_t)uv_candidate_count,
            sizeof(uv_candidates[0]), cmp_res_gain_per_byte_desc);
      for (i = 0; i < uv_candidate_count; ++i) {
        ResCandidate *candidate = &uv_candidates[i];
        MbResState *state = &res_state[candidate->mb_index];
        size_t replaced_cost = 0;
        uint64_t replaced_gain = 0;
        int has_luma_residual = 0;
        size_t luma_cost = 0;
        uint64_t luma_gain = 0;
        size_t combo_cost = 0;
        uint64_t combo_gain = 0;
        int block_index;
        if (analysis[candidate->mb_index].raw_mode) continue;
        for (block_index = 0; block_index < state->count; ++block_index) {
          if (state->blocks[block_index].plane == PATCH_PLANE_Y) {
            has_luma_residual = 1;
            luma_cost += state->blocks[block_index].cost;
            luma_gain += state->blocks[block_index].gain;
          }
        }
        if (!has_luma_residual) {
          if (state->count != 0) {
            replaced_cost = state->cost;
            replaced_gain = state->gain;
            if (candidate->gain <= replaced_gain) continue;
          }
          if (frame_min_gain_per_byte > 0.0 &&
              (double)candidate->gain <
                  frame_min_gain_per_byte * (double)candidate->cost) {
            break;
          }
          if (estimated_payload - replaced_cost + candidate->cost >
              p_frame_budget) {
            continue;
          }
          analysis[candidate->mb_index].raw_mode = MODE_RAW_UV_MB;
          estimated_payload =
              estimated_payload - replaced_cost + candidate->cost;
          if (state->count != 0) {
            state->block_mask = 0;
            state->count = 0;
            state->cost = 0;
            state->gain = 0;
          }
          continue;
        }

        replaced_cost = state->cost;
        combo_cost = 1 + RAW_UV_MB_BYTES + luma_cost;
        combo_gain = candidate->gain + luma_gain;
        if (combo_gain <= state->gain) continue;
        if (combo_cost > state->cost && frame_min_gain_per_byte > 0.0 &&
            (double)(combo_gain - state->gain) <
                frame_min_gain_per_byte * (double)(combo_cost - state->cost)) {
          continue;
        }
        if (estimated_payload - state->cost + combo_cost > p_frame_budget) {
          continue;
        }
        {
          ResBlock luma_blocks[16];
          uint32_t luma_mask = 0;
          uint8_t out_count = 0;
          for (block_index = 0; block_index < state->count; ++block_index) {
            if (state->blocks[block_index].plane == PATCH_PLANE_Y) {
              luma_blocks[out_count++] = state->blocks[block_index];
              luma_mask |= (uint32_t)(1u << res_block_slot(PATCH_PLANE_Y,
                                                           state->blocks[block_index].block));
            }
          }
          memcpy(state->blocks, luma_blocks, sizeof(luma_blocks[0]) * out_count);
          state->block_mask = luma_mask;
          state->count = out_count;
          state->cost = combo_cost;
          state->gain = combo_gain;
        }
        analysis[candidate->mb_index].raw_mode = MODE_COPY16_RES4_RAWUV;
        estimated_payload = estimated_payload - replaced_cost + combo_cost;
      }
      qsort(analysis, MB_COUNT, sizeof(analysis[0]), cmp_mb_index_asc);
      for (i = 0; i < MB_COUNT; ++i) {
        if (analysis[i].raw_mode == MODE_RAW_MB) {
          buffer_u8(&payload, MODE_RAW_MB);
          write_raw_mb(&payload, src, i);
          copy_raw_mb_to_frame(recon, src, i);
          ++raw_mb;
        } else if (analysis[i].raw_mode == MODE_RAW_Y_MB) {
          buffer_u8(&payload, MODE_RAW_Y_MB);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
          write_raw_y_mb(&payload, src, i);
          copy_mb_from_ref(recon, ref, i, analysis[i].mv);
          copy_raw_y_mb_to_frame(recon, src, i);
          ++raw_y_mb;
        } else if (analysis[i].raw_mode == MODE_RAW_UV_MB) {
          buffer_u8(&payload, MODE_RAW_UV_MB);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
          write_raw_uv_mb(&payload, src, i);
          copy_mb_from_ref(recon, ref, i, analysis[i].mv);
          copy_raw_uv_mb_to_frame(recon, src, i);
          ++raw_uv_mb;
        } else if (analysis[i].raw_mode == MODE_COPY16_RES4_RAWUV) {
          int block_index;
          buffer_u8(&payload, MODE_COPY16_RES4_RAWUV);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
          buffer_u8(&payload, res_state[i].count);
          write_raw_uv_mb(&payload, src, i);
          copy_mb_from_ref(recon, ref, i, analysis[i].mv);
          copy_raw_uv_mb_to_frame(recon, src, i);
          for (block_index = 0; block_index < res_state[i].count;
               ++block_index) {
            const ResBlock *block = &res_state[i].blocks[block_index];
            if (block->plane != PATCH_PLANE_Y) die("rawuv residual not luma");
            write_repair4(&payload, src, i, block);
            apply_res4_to_frame(recon, i, block, res_q);
            ++res4;
            if (block->type == REPAIR_CAND_RAW4) ++raw4;
            res_coeff += block->nz;
          }
          ++rawuv_res_mb;
        } else if (analysis[i].raw_mode == MODE_COPY16_DC) {
          buffer_u8(&payload, MODE_COPY16_DC);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
          buffer_u8(&payload, (uint8_t)analysis[i].dc_y);
          buffer_u8(&payload, (uint8_t)analysis[i].dc_u);
          buffer_u8(&payload, (uint8_t)analysis[i].dc_v);
          apply_copy16_dc_to_frame(recon, ref, i, analysis[i].mv,
                                   analysis[i].dc_y, analysis[i].dc_u,
                                   analysis[i].dc_v);
          ++copy16_dc;
        } else if (res_state[i].count != 0) {
          int block_index;
          buffer_u8(&payload, MODE_COPY16_RES4);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
          buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
          buffer_u8(&payload, res_state[i].count);
          copy_mb_from_ref(recon, ref, i, analysis[i].mv);
          for (block_index = 0; block_index < res_state[i].count;
               ++block_index) {
            const ResBlock *block = &res_state[i].blocks[block_index];
            write_repair4(&payload, src, i, block);
            apply_res4_to_frame(recon, i, block, res_q);
            ++res4;
            if (block->type == REPAIR_CAND_RAW4) ++raw4;
            res_coeff += block->nz;
          }
          ++res_mb;
        } else {
          const uint64_t copy_sse = mb_prediction_sse(src, ref, i, analysis[i].mv);
          uint64_t intra_sse = intra_mb_sse(src, recon, i, MODE_INTRA_DC);
          uint64_t test_sse = intra_mb_sse(src, recon, i, MODE_INTRA_V);
          int intra_mode = MODE_INTRA_DC;
          if (test_sse < intra_sse) {
            intra_sse = test_sse;
            intra_mode = MODE_INTRA_V;
          }
          test_sse = intra_mb_sse(src, recon, i, MODE_INTRA_H);
          if (test_sse < intra_sse) {
            intra_sse = test_sse;
            intra_mode = MODE_INTRA_H;
          }
          if (intra_sse < copy_sse) {
            buffer_u8(&payload, (uint8_t)intra_mode);
            predict_intra_mb(recon, i, intra_mode);
            if (intra_mode == MODE_INTRA_DC) {
              ++intra_dc;
            } else if (intra_mode == MODE_INTRA_V) {
              ++intra_v;
            } else {
              ++intra_h;
            }
          } else {
            buffer_u8(&payload, MODE_COPY16);
            buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.col);
            buffer_u8(&payload, (uint8_t)(int8_t)analysis[i].mv.row);
            copy_mb_from_ref(recon, ref, i, analysis[i].mv);
            ++copy16;
          }
        }
      }
      write_frame(out, FRAME_P, &payload);
      total_bytes += 2 + 4 + payload.len;
      if (payload.len > remaining_p_payload_budget) {
        die("P-frame payload exceeded remaining budget");
      }
      remaining_p_payload_budget -= payload.len;
      --remaining_p_frames;
      fprintf(stderr,
              "frame %d: type=p bytes=%zu raw_mb=%d raw_y_mb=%d "
              "raw_uv_mb=%d rawuv_res_mb=%d intra_dc=%d intra_v=%d "
              "intra_h=%d res_mb=%d res4=%d raw4=%d res_coeff=%d "
              "copy16=%d copy16_dc=%d "
              "p_budget=%zu plan_budget=%zu frame_budget=%zu "
              "avg_remaining_payload=%.1f frame_min_gain_per_byte=%.3f "
              "remaining_p_payload=%zu remaining_p_frames=%d "
              "quality_budget_bias=%.3f "
              "quality_gain_scale=%.3f quality_plane_gain=%.3f/%.3f/%.3f "
              "res_candidates=%d\n",
              frame_no, payload.len + 6, raw_mb, raw_y_mb, raw_uv_mb,
              rawuv_res_mb, intra_dc, intra_v, intra_h, res_mb, res4, raw4,
              res_coeff, copy16, copy16_dc, p_budget, frame_plan_budget,
              p_frame_budget,
              avg_remaining_payload, frame_min_gain_per_byte,
              remaining_p_payload_budget, remaining_p_frames,
              quality_budget_bias[frame_no], quality_gain_scale[frame_no],
              frame_plane_gain_scale[PATCH_PLANE_Y],
              frame_plane_gain_scale[PATCH_PLANE_U],
              frame_plane_gain_scale[PATCH_PLANE_V],
              res_candidate_count);
      free(res_candidates);
      free(uv_candidates);
      free(dc_candidates);
      free(res_state);
    }
    sse = frame_sse(src, recon);
    psnr = frame_psnr(sse);
    total_sse += sse;
    memcpy(ref, recon, FRAME_SIZE);
    buffer_free(&payload);
    (void)psnr;
  }

  elapsed = now_seconds() - t0;
  fprintf(stderr,
          "o3vpx_encode_summary frames=%d bytes=%zu bitrate_mbps=%.6f "
          "elapsed_seconds=%.3f fps=%.3f avg_psnr_db=%.6f\n",
          frames, total_bytes,
          (double)total_bytes * 8.0 * FPS / (double)frames / 1000000.0,
          elapsed, (double)frames / elapsed, sequence_psnr(total_sse, frames));
  free(src);
  free(ref);
  free(recon);
  free(key_flags);
  free(mse_by_frame);
  free(planned_p_budget);
  free(quality_budget_bias);
  free(quality_gain_scale);
  free(quality_plane_gain_scale);
  fclose(in);
  fclose(out);
  return EXIT_SUCCESS;
}

static void parse_p_frame_commands(O3vpxReader *reader,
                                   O3vpxMbCommand *commands,
                                   uint32_t payload_len,
                                   O3vpxFrameInfo *info) {
  uint32_t consumed = 0;
  int mb;
  if (!commands) die("missing P-frame command buffer");
  for (mb = 0; mb < MB_COUNT; ++mb) {
    O3vpxMbCommand *cmd = &commands[mb];
    uint8_t mode = get_u8(reader);
    cmd->mode = mode;
    cmd->block_count = 0;
    consumed += 1;
    if (info && mode < O3VPX_MODE_COUNT) {
      info->mode_counts[mode]++;
    }
    if (mode == MODE_COPY16) {
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      consumed += 2;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("copy MV out of bounds");
      count_mv_mb(info, cmd->mv);
    } else if (mode == MODE_COPY16_DC) {
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      cmd->dc_y = (int8_t)get_u8(reader);
      cmd->dc_u = (int8_t)get_u8(reader);
      cmd->dc_v = (int8_t)get_u8(reader);
      consumed += 5;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("copy-dc MV out of bounds");
      count_mv_mb(info, cmd->mv);
    } else if (mode == MODE_COPY16_QDC) {
      int q;
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      for (q = 0; q < 4; ++q) cmd->dc_y4[q] = (int8_t)get_u8(reader);
      cmd->dc_u = (int8_t)get_u8(reader);
      cmd->dc_v = (int8_t)get_u8(reader);
      consumed += 8;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("copy-qdc MV out of bounds");
      count_mv_mb(info, cmd->mv);
    } else if (mode == MODE_RAW_MB) {
      reader_exact(reader, cmd->data.raw, RAW_MB_BYTES);
      consumed += RAW_MB_BYTES;
    } else if (mode == MODE_RAW_Y_MB) {
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      consumed += 2;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("raw-y MV out of bounds");
      count_mv_mb(info, cmd->mv);
      reader_exact(reader, cmd->data.raw, RAW_Y_MB_BYTES);
      consumed += RAW_Y_MB_BYTES;
    } else if (mode == MODE_RAW_UV_MB) {
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      consumed += 2;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("raw-uv MV out of bounds");
      count_mv_mb(info, cmd->mv);
      reader_exact(reader, cmd->data.raw, RAW_UV_MB_BYTES);
      consumed += RAW_UV_MB_BYTES;
    } else if (mode == MODE_COPY16_RES4_RAWUV) {
      uint8_t block_count;
      uint8_t block_index;
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      block_count = get_u8(reader);
      cmd->block_count = block_count;
      consumed += 3;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) {
        die("rawuv-residual MV out of bounds");
      }
      if (block_count > 16) die("too many rawuv residual blocks");
      count_mv_mb(info, cmd->mv);
      reader_exact(reader, cmd->data.rawuv.raw, RAW_UV_MB_BYTES);
      consumed += RAW_UV_MB_BYTES;
      for (block_index = 0; block_index < block_count; ++block_index) {
        ResBlock *block = &cmd->data.rawuv.blocks[block_index];
        consumed += (uint32_t)read_res4(reader, block);
        count_res4_block(info, block);
        if (block->plane != PATCH_PLANE_Y) die("rawuv residual not luma");
      }
    } else if (mode == MODE_INTRA_DC || mode == MODE_INTRA_V ||
               mode == MODE_INTRA_H) {
    } else if (mode == MODE_COPY16_PATCH4) {
      uint8_t patch_count;
      uint8_t patch;
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      patch_count = get_u8(reader);
      cmd->block_count = patch_count;
      consumed += 3;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("patch MV out of bounds");
      if (patch_count > PATCH_CANDIDATES_PER_MB) {
        die("too many 4x4 patches");
      }
      count_mv_mb(info, cmd->mv);
      for (patch = 0; patch < patch_count; ++patch) {
        ResBlock *block = &cmd->data.blocks[patch];
        block->type = REPAIR_CAND_RAW4;
        block->plane = get_u8(reader);
        block->block = get_u8(reader);
        if (!valid_4x4_block(block->plane, block->block)) {
          die("bad 4x4 patch block");
        }
        reader_exact(reader, block->raw, RAW_4X4_BYTES);
        if (info) info->raw4_blocks++;
        consumed += 2 + RAW_4X4_BYTES;
      }
    } else if (mode == MODE_COPY16_RES4) {
      uint8_t block_count;
      uint8_t block_index;
      cmd->mv.col = (int8_t)get_u8(reader);
      cmd->mv.row = (int8_t)get_u8(reader);
      block_count = get_u8(reader);
      cmd->block_count = block_count;
      consumed += 3;
      if (!mv_in_bounds_same_eye(mb, cmd->mv)) die("residual MV out of bounds");
      if (block_count > RES_BLOCKS_PER_MB) die("too many residual blocks");
      count_mv_mb(info, cmd->mv);
      for (block_index = 0; block_index < block_count; ++block_index) {
        ResBlock *block = &cmd->data.blocks[block_index];
        consumed += (uint32_t)read_res4(reader, block);
        count_res4_block(info, block);
      }
    } else {
      die("bad P-frame mode");
    }
  }
  if (consumed != payload_len) die("bad P-frame payload length");
}

static void apply_mb_command(uint8_t *recon, const uint8_t *ref, int mb,
                             const O3vpxMbCommand *cmd, int res_q) {
  uint8_t block_index;
  if (cmd->mode == MODE_COPY16) {
    copy_mb_from_ref(recon, ref, mb, cmd->mv);
  } else if (cmd->mode == MODE_COPY16_DC) {
    apply_copy16_dc_to_frame(recon, ref, mb, cmd->mv, cmd->dc_y, cmd->dc_u,
                             cmd->dc_v);
  } else if (cmd->mode == MODE_COPY16_QDC) {
    apply_copy16_qdc_to_frame(recon, ref, mb, cmd->mv, cmd->dc_y4, cmd->dc_u,
                              cmd->dc_v);
  } else if (cmd->mode == MODE_RAW_MB) {
    apply_raw_mb_bytes_to_frame(recon, mb, cmd->data.raw);
  } else if (cmd->mode == MODE_RAW_Y_MB) {
    copy_mb_uv_from_ref(recon, ref, mb, cmd->mv);
    apply_raw_y_mb_bytes_to_frame(recon, mb, cmd->data.raw);
  } else if (cmd->mode == MODE_RAW_UV_MB) {
    copy_mb_y_from_ref(recon, ref, mb, cmd->mv);
    apply_raw_uv_mb_bytes_to_frame(recon, mb, cmd->data.raw);
  } else if (cmd->mode == MODE_COPY16_RES4_RAWUV) {
    copy_mb_y_from_ref(recon, ref, mb, cmd->mv);
    apply_raw_uv_mb_bytes_to_frame(recon, mb, cmd->data.rawuv.raw);
    for (block_index = 0; block_index < cmd->block_count; ++block_index) {
      apply_res4_to_frame(recon, mb, &cmd->data.rawuv.blocks[block_index],
                          res_q);
    }
  } else if (cmd->mode == MODE_INTRA_DC) {
    predict_intra_mb(recon, mb, MODE_INTRA_DC);
  } else if (cmd->mode == MODE_INTRA_V) {
    predict_intra_mb(recon, mb, MODE_INTRA_V);
  } else if (cmd->mode == MODE_INTRA_H) {
    predict_intra_mb(recon, mb, MODE_INTRA_H);
  } else if (cmd->mode == MODE_COPY16_PATCH4) {
    copy_mb_from_ref(recon, ref, mb, cmd->mv);
    for (block_index = 0; block_index < cmd->block_count; ++block_index) {
      apply_res4_to_frame(recon, mb, &cmd->data.blocks[block_index], res_q);
    }
  } else if (cmd->mode == MODE_COPY16_RES4) {
    copy_mb_from_ref(recon, ref, mb, cmd->mv);
    for (block_index = 0; block_index < cmd->block_count; ++block_index) {
      apply_res4_to_frame(recon, mb, &cmd->data.blocks[block_index], res_q);
    }
  } else {
    die("bad P-frame mode");
  }
}

static void reconstruct_p_frame_eye(uint8_t *recon, const uint8_t *ref,
                                    const O3vpxMbCommand *commands, int res_q,
                                    int eye) {
  const int start_x = eye ? MB_W / 2 : 0;
  const int end_x = eye ? MB_W : MB_W / 2;
  int mb_y;
  for (mb_y = 0; mb_y < MB_H; ++mb_y) {
    int mb_x;
    for (mb_x = start_x; mb_x < end_x; ++mb_x) {
      const int mb = mb_y * MB_W + mb_x;
      apply_mb_command(recon, ref, mb, &commands[mb], res_q);
    }
  }
}

#ifdef __3DS__
static void o3vpx_reconstruct_worker_thread(void *arg) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)arg;
  while (state->worker_running) {
    if (!state->worker_has_job) {
      svcSleepThread(10000);
      continue;
    }
    reconstruct_p_frame_eye(state->worker_job.recon, state->worker_job.ref,
                            state->worker_job.commands, state->worker_job.res_q,
                            state->worker_job.eye);
    state->worker_has_job = 0;
    state->worker_done = 1;
  }
}

static int o3vpx_start_reconstruct_worker(O3vpxDecoderState *state) {
  s32 prio = 0x2a;
  if (!state || state->worker_thread != NULL) return 0;
  svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
  state->worker_running = 1;
  state->worker_has_job = 0;
  state->worker_done = 1;
  state->worker_thread =
      threadCreate(o3vpx_reconstruct_worker_thread, state, 64 * 1024, prio,
                   -1, false);
  if (state->worker_thread == NULL) {
    state->worker_running = 0;
    return -1;
  }
  return 0;
}

static void o3vpx_stop_reconstruct_worker(O3vpxDecoderState *state) {
  if (!state || state->worker_thread == NULL) return;
  state->worker_running = 0;
  threadJoin(state->worker_thread, UINT64_MAX);
  threadFree(state->worker_thread);
  state->worker_thread = NULL;
  state->worker_has_job = 0;
  state->worker_done = 0;
}
#endif

static void reconstruct_p_frame(O3vpxDecoderState *state, uint8_t *recon,
                                const uint8_t *ref,
                                const O3vpxMbCommand *commands, int res_q) {
#ifdef __3DS__
  if (state && state->worker_thread != NULL) {
    state->worker_job.recon = recon;
    state->worker_job.ref = ref;
    state->worker_job.commands = commands;
    state->worker_job.res_q = res_q;
    state->worker_job.eye = 1;
    state->worker_done = 0;
    state->worker_has_job = 1;
    reconstruct_p_frame_eye(recon, ref, commands, res_q, 0);
    while (!state->worker_done) {
      svcSleepThread(1000);
    }
    return;
  }
#else
  (void)state;
#endif
  reconstruct_p_frame_eye(recon, ref, commands, res_q, 0);
  reconstruct_p_frame_eye(recon, ref, commands, res_q, 1);
}

static int decode_next_frame_from_reader(O3vpxReader *reader, uint8_t *ref,
                                         uint8_t *recon,
                                         O3vpxMbCommand *commands,
                                         O3vpxDecoderState *state, int res_q,
                                         int frame_no, O3vpxFrameInfo *info,
                                         size_t *total_bytes) {
  uint16_t frame_type = get_le16(reader);
  uint32_t payload_len = get_le32(reader);
  if (info) memset(info, 0, sizeof(*info));
  if (total_bytes) *total_bytes += 6 + payload_len;
  if (frame_type == FRAME_RAW_KEY) {
    if (payload_len != FRAME_SIZE) die("bad raw key payload size");
    reader_exact(reader, recon, FRAME_SIZE);
  } else if (frame_type == FRAME_P) {
    parse_p_frame_commands(reader, commands, payload_len, info);
    reconstruct_p_frame(state, recon, ref, commands, res_q);
  } else {
    die("bad frame type");
  }
  if (info) {
    info->frame_no = (unsigned int)frame_no;
    info->frame_type = (unsigned int)frame_type;
    info->frame_size_bytes = (unsigned int)(6 + payload_len);
  }
  return 1;
}

size_t vp8_o3vpx_decoder_size(void) { return sizeof(O3vpxDecoderState); }

size_t vp8_o3vpx_decoder_align(void) { return 16; }

size_t vp8_o3vpx_decoder_internal_bytes(void) {
  return FRAME_SIZE * 2 + sizeof(O3vpxMbCommand) * MB_COUNT;
}

size_t vp8_o3vpx_frame_bytes(void) { return O3VPX_FRAME_SIZE; }

size_t vp8_o3vpx_eye_frame_bytes(void) { return O3VPX_EYE_FRAME_SIZE; }

static void decoder_free_frame_buffers(O3vpxDecoderState *state) {
  if (!state) return;
#ifdef __3DS__
  o3vpx_stop_reconstruct_worker(state);
#endif
  free(state->ref);
  free(state->recon);
  free(state->commands);
  state->ref = NULL;
  state->recon = NULL;
  state->commands = NULL;
}

static int decoder_alloc_frame_buffers(O3vpxDecoderState *state) {
  if (!state) return -1;
  state->ref = (uint8_t *)malloc(FRAME_SIZE);
  state->recon = (uint8_t *)malloc(FRAME_SIZE);
  state->commands =
      (O3vpxMbCommand *)malloc(sizeof(O3vpxMbCommand) * MB_COUNT);
  if (!state->ref || !state->recon || !state->commands) {
    decoder_free_frame_buffers(state);
    return -2;
  }
#ifdef __3DS__
  (void)o3vpx_start_reconstruct_worker(state);
#endif
  return 0;
}

static int decoder_finish_init(O3vpxDecoderState *state) {
  int rc = vp8_o3vpx_decoder_reset(state);
  if (rc != 0) {
    decoder_free_frame_buffers(state);
    memset(state, 0, sizeof(*state));
  }
  return rc;
}

int vp8_o3vpx_decoder_reset(void *decoder) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  if (!state || !state->ref || !state->recon || !state->commands ||
      (!state->stream_file && (!state->stream || state->stream_len == 0))) {
    return -1;
  }
  if (state->stream_file) {
    if (fseek(state->stream_file, 0, SEEK_SET) != 0) return -3;
    reader_init_file(&state->reader, state->stream_file);
  } else {
    reader_init_mem(&state->reader, state->stream, state->stream_len);
  }
  if (!read_stream_header(&state->reader, &state->frames, &state->res_q)) {
    return -2;
  }
  state->frame_no = 0;
  memset(state->ref, 0, FRAME_SIZE);
  memset(state->recon, 0, FRAME_SIZE);
  return 0;
}

int vp8_o3vpx_decoder_init(void *decoder, size_t decoder_size,
                           const unsigned char *stream, size_t stream_len) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  if (!state || decoder_size < sizeof(*state) || !stream || stream_len == 0) {
    return -1;
  }
  memset(state, 0, sizeof(*state));
  state->stream = stream;
  state->stream_len = stream_len;
  if (decoder_alloc_frame_buffers(state) != 0) {
    memset(state, 0, sizeof(*state));
    return -2;
  }
  return decoder_finish_init(state);
}

int vp8_o3vpx_decoder_init_file(void *decoder, size_t decoder_size,
                                FILE *stream) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  if (!state || decoder_size < sizeof(*state) || !stream) {
    return -1;
  }
  memset(state, 0, sizeof(*state));
  state->stream_file = stream;
  if (decoder_alloc_frame_buffers(state) != 0) {
    memset(state, 0, sizeof(*state));
    return -2;
  }
  return decoder_finish_init(state);
}

int vp8_o3vpx_decoder_next_frame(void *decoder, O3vpxFrameInfo *info) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  int rc;
  uint8_t *tmp;
  if (!state || !state->ref || !state->recon || !state->commands) return -1;
  if (state->frame_no >= state->frames) return 0;
  rc = decode_next_frame_from_reader(&state->reader, state->recon, state->ref,
                                     state->commands, state, state->res_q,
                                     state->frame_no, info, NULL);
  if (rc > 0) {
    tmp = state->recon;
    state->recon = state->ref;
    state->ref = tmp;
    ++state->frame_no;
  }
  return rc;
}

int vp8_o3vpx_decoder_write_current_eye_yuv420p(void *decoder, int eye,
                                                unsigned char *out,
                                                size_t out_len) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  const uint8_t *src_y;
  const uint8_t *src_u;
  const uint8_t *src_v;
  uint8_t *out_y;
  uint8_t *out_u;
  uint8_t *out_v;
  int y_x;
  int uv_x;
  int row;
  if (!state || !state->recon || !out ||
      out_len < O3VPX_EYE_FRAME_SIZE || (eye != 0 && eye != 1)) {
    return -1;
  }
  src_y = state->recon;
  src_u = state->recon + Y_SIZE;
  src_v = state->recon + Y_SIZE + UV_SIZE;
  out_y = out;
  out_u = out + O3VPX_EYE_WIDTH * O3VPX_EYE_HEIGHT;
  out_v = out_u + (O3VPX_EYE_WIDTH / 2) * (O3VPX_EYE_HEIGHT / 2);
  y_x = eye == 0 ? 0 : O3VPX_EYE_WIDTH;
  uv_x = eye == 0 ? 0 : O3VPX_EYE_WIDTH / 2;
  for (row = 0; row < HEIGHT; ++row) {
    memcpy(out_y + row * O3VPX_EYE_WIDTH, src_y + row * WIDTH + y_x,
           O3VPX_EYE_WIDTH);
  }
  for (row = 0; row < UV_H; ++row) {
    memcpy(out_u + row * (O3VPX_EYE_WIDTH / 2), src_u + row * UV_W + uv_x,
           O3VPX_EYE_WIDTH / 2);
    memcpy(out_v + row * (O3VPX_EYE_WIDTH / 2), src_v + row * UV_W + uv_x,
           O3VPX_EYE_WIDTH / 2);
  }
  return 0;
}

int vp8_o3vpx_decoder_write_current_frame_yuv420p(void *decoder,
                                                  unsigned char *out,
                                                  size_t out_len) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  if (!state || !state->recon || !out || out_len < O3VPX_FRAME_SIZE) {
    return -1;
  }
  memcpy(out, state->recon, O3VPX_FRAME_SIZE);
  return 0;
}

int vp8_o3vpx_decoder_write_current_yuv420p(void *decoder, unsigned char *left,
                                            size_t left_len,
                                            unsigned char *right,
                                            size_t right_len) {
  int rc;
  if (!right || right_len < O3VPX_EYE_FRAME_SIZE) {
    return -1;
  }
  rc = vp8_o3vpx_decoder_write_current_eye_yuv420p(decoder, 0, left, left_len);
  if (rc != 0) {
    return rc;
  }
  return vp8_o3vpx_decoder_write_current_eye_yuv420p(decoder, 1, right,
                                                     right_len);
}

void vp8_o3vpx_decoder_drop(void *decoder) {
  O3vpxDecoderState *state = (O3vpxDecoderState *)decoder;
  if (!state) return;
  decoder_free_frame_buffers(state);
  memset(state, 0, sizeof(*state));
}

int vp8_o3vpx_decode_file_limit(const char *in_path, const char *out_path,
                                int max_frames) {
  FILE *in;
  FILE *out = NULL;
  O3vpxReader reader;
  int frames;
  int res_q;
  int frame_no;
  uint8_t *ref = (uint8_t *)xmalloc(FRAME_SIZE);
  uint8_t *recon = (uint8_t *)xmalloc(FRAME_SIZE);
  O3vpxMbCommand *commands =
      (O3vpxMbCommand *)xmalloc(sizeof(O3vpxMbCommand) * MB_COUNT);
  size_t total_bytes = 0;
  double t0;
  double elapsed;
  in = fopen(in_path, "rb");
  if (!in) die_errno("open input failed");
  if (out_path) {
    out = fopen(out_path, "wb");
    if (!out) die_errno("open output failed");
  }
  reader_init_file(&reader, in);
  if (!read_stream_header(&reader, &frames, &res_q)) die("missing O3VX header");
  if (max_frames < 0) die("bad decode frame limit");
  if (max_frames > 0 && max_frames < frames) frames = max_frames;
  t0 = now_seconds();
  for (frame_no = 0; frame_no < frames; ++frame_no) {
    uint8_t *tmp;
    decode_next_frame_from_reader(&reader, ref, recon, commands, NULL, res_q,
                                  frame_no, NULL, &total_bytes);
    if (out && fwrite(recon, 1, FRAME_SIZE, out) != FRAME_SIZE) {
      die_errno("write failed");
    }
    tmp = ref;
    ref = recon;
    recon = tmp;
  }
  elapsed = now_seconds() - t0;
  fprintf(stderr,
          "o3vpx_decode_summary frames=%d bytes=%zu elapsed_seconds=%.3f "
          "fps=%.3f\n",
          frames, total_bytes, elapsed, (double)frames / elapsed);
  free(ref);
  free(recon);
  free(commands);
  fclose(in);
  if (out) fclose(out);
  return EXIT_SUCCESS;
}

int vp8_o3vpx_decode_file(const char *in_path, const char *out_path) {
  return vp8_o3vpx_decode_file_limit(in_path, out_path, 0);
}
