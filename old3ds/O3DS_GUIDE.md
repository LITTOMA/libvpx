# O3DS O3VPX Encode/Decode Guide

This guide covers the modified `libvpx` O3VPX route. O3VPX is not a VP8
bitstream; it only lives under `vp8/` because the prototype reuses VP8-derived
motion, transform, and helper code.

Unless noted otherwise, run commands from the workspace root:

```sh
cd /home/john/work/minicodec
```

## Files

```text
libvpx/tools/o3vpx.c        host CLI wrapper
libvpx/vp8/o3vpx.c         encoder/decoder implementation
libvpx/vp8/o3vpx.h         public O3VPX constants/API
libvpx/old3ds/Makefile     Old3DS .3dsx harness build
libvpx/old3ds/source/main.c Old3DS decode/timing/playback harness
```

The stream geometry is fixed:

```text
video: 800x240 YUV420P, 24 fps
SBS:   left eye 400x240, right eye 400x240
frame: 288000 bytes
```

The current Old3DS harness loads `sdmc:/o3vpxbench.o3vx` and writes
`sdmc:/o3yvbench.log`.

## Build The Host CLI

If `tmp/libvpx-o3vpx-build/tools/o3vpx` already exists and is newer than
`libvpx/vp8/o3vpx.c`, you can reuse it. Rebuild after any codec edit:

```sh
mkdir -p tmp/libvpx-o3vpx-build
cd tmp/libvpx-o3vpx-build
../../libvpx/configure \
  --disable-examples \
  --disable-docs \
  --disable-unit-tests \
  --disable-webm-io \
  --disable-libyuv \
  --disable-vp9 \
  --enable-vp8 \
  --enable-realtime-only \
  --disable-runtime-cpu-detect
make -j"$(nproc)" tools/o3vpx
cd ../..
```

Host CLI path:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx
```

## Prepare Input

The encoder input must be raw `800x240` YUV420P.

Current proof window for the 3D source:

```sh
mkdir -p tmp/o3vpx_real_1_f10000_10200
ffmpeg -hide_banner -y -i /mnt/hgfs/deb13/3d/1.mkv \
  -vf "select='between(n,10000,10200)',scale=800:240" \
  -fps_mode passthrough \
  -frames:v 201 \
  -pix_fmt yuv420p \
  -f rawvideo tmp/o3vpx_real_1_f10000_10200/source_201.yuv
```

For a normal 2D source, scale directly to `800x240`. Do not duplicate a
`400x240` image into both eyes unless the test specifically needs fake SBS
stereo content.

```sh
mkdir -p tmp/o3vpx_2d
ffmpeg -hide_banner -y -i /mnt/hgfs/deb13/3d/2.mp4 \
  -an \
  -vf "select='between(n,1000,1200)',scale=800:240:flags=bicubic,format=yuv420p" \
  -fps_mode passthrough \
  -frames:v 201 \
  -f rawvideo tmp/o3vpx_2d/source_201.yuv
```

Check the size:

```sh
wc -c tmp/o3vpx_2d/source_201.yuv
```

For 201 frames it must be:

```text
57888000
```

## Encode

General syntax:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx enc \
  <input-800x240-yuv420p.yuv> \
  <output.o3vx> \
  [frames] [keyint] [target_mbps] [search_radius] \
  [scene_mse_threshold] [res_q] [min_gain_per_byte] [p_burst_mult] \
  [baseline_psnr.log] [previous_o3vpx_psnr.log]
```

Current default 8 Mbit/s proof-style settings:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx enc \
  tmp/o3vpx_2d/source_201.yuv \
  tmp/o3vpx_2d/current_8m.o3vx \
  201 48 8 8 2000 13 3 2.5 \
  >tmp/o3vpx_2d/encode.stdout.log \
  2>tmp/o3vpx_2d/encode.log
```

Useful parameters:

```text
frames               number of input frames, 0 means all frames
keyint               max raw-key gap, usually 48 for proof runs
target_mbps          average stream budget, usually 8
search_radius        full-pel motion search radius before half-pel refine
scene_mse_threshold  forces raw key on large temporal scene change
res_q                residual quantizer; current proof-style value is 13
min_gain_per_byte    repair admission threshold; current value is 3
p_burst_mult         temporary P-frame burst allowance; current value is 2.5
```

For realtime-oriented candidates, the encoder also honors
`O3VPX_DECODE_COST_BUDGET`. A value such as `3500000` caps the estimated
per-P-frame reconstruction cost and biases the encoder away from frames packed
with full-IDCT residual blocks.

Additional realtime caps are available through environment variables:

```text
O3VPX_MAX_P_FRAME_BYTES       hard P-frame size cap including frame header
O3VPX_MAX_FULL_IDCT_BLOCKS    hard full-IDCT residual block cap per P frame
O3VPX_MAX_HALFPEL_MB          hard half-pel motion-vector macroblock cap
O3VPX_MAX_RESIDUAL_BLOCKS     hard residual block cap per P frame
```

A conservative Old3DS realtime starting point is:

```sh
O3VPX_DECODE_COST_BUDGET=2800000 \
O3VPX_MAX_P_FRAME_BYTES=40000 \
O3VPX_MAX_FULL_IDCT_BLOCKS=1400 \
O3VPX_MAX_HALFPEL_MB=560 \
O3VPX_MAX_RESIDUAL_BLOCKS=6500 \
tmp/libvpx-o3vpx-build/tools/o3vpx enc \
  tmp/o3vpx_2d/source_201.yuv \
  tmp/o3vpx_2d/realtime_8m.o3vx \
  201 48 8 8 2000 20 5 1.1
```

The encoder writes per-frame mode and byte summaries to stderr, so keep
`encode.log`.

## Decode On Host

Decode to raw SBS YUV:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx dec \
  tmp/o3vpx_2d/current_8m.o3vx \
  tmp/o3vpx_2d/decoded.yuv \
  201 \
  >tmp/o3vpx_2d/decode.stdout.log \
  2>tmp/o3vpx_2d/decode.log
```

Decode to null for speed:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx decnull \
  tmp/o3vpx_2d/current_8m.o3vx \
  201 \
  >tmp/o3vpx_2d/decnull.stdout.log \
  2>tmp/o3vpx_2d/decnull.log
```

Exercise the public memory decoder API and split/merge path:

```sh
tmp/libvpx-o3vpx-build/tools/o3vpx decmem \
  tmp/o3vpx_2d/current_8m.o3vx \
  tmp/o3vpx_2d/decoded_mem.yuv \
  201
```

## Inspect PNGs

Convert decoded YUV to PNG:

```sh
mkdir -p tmp/o3vpx_2d/png
ffmpeg -hide_banner -y \
  -f rawvideo -pix_fmt yuv420p -s 800x240 -r 24 \
  -i tmp/o3vpx_2d/decoded.yuv \
  -frames:v 201 \
  tmp/o3vpx_2d/png/tmp_%03d.png
```

Rename PNGs with source-frame and key/P labels from `encode.log`:

```sh
python3 - <<'PY'
from pathlib import Path

root = Path("tmp/o3vpx_2d")
png = root / "png"
types = {}
for line in (root / "encode.log").read_text().splitlines():
    if not line.startswith("frame "):
        continue
    parts = line.split()
    idx = int(parts[1].rstrip(":"))
    typ = next(p.split("=", 1)[1] for p in parts if p.startswith("type="))
    types[idx] = "key" if typ == "raw_key" else "p"

rows = ["local_frame,source_frame,type,png"]
source_start = 1000
for i in range(201):
    src = png / f"tmp_{i + 1:03d}.png"
    typ = types.get(i, "unknown")
    dst = png / f"frame_{i:03d}_src{source_start + i}_{typ}.png"
    if src.exists():
        src.replace(dst)
    rows.append(f"{i},{source_start + i},{typ},{dst.name}")
(png / "frame_types.csv").write_text("\n".join(rows) + "\n")
PY
```

## Quality Check

Run PSNR against the source YUV:

```sh
ffmpeg -hide_banner -y \
  -f rawvideo -pix_fmt yuv420p -s 800x240 -r 24 \
  -i tmp/o3vpx_2d/decoded.yuv \
  -f rawvideo -pix_fmt yuv420p -s 800x240 -r 24 \
  -i tmp/o3vpx_2d/source_201.yuv \
  -lavfi "psnr=stats_file=tmp/o3vpx_2d/psnr.log" \
  -f null -
```

Summarize mode counts:

```sh
awk '
/^frame [0-9]+:/ {
  for (i = 1; i <= NF; i++) {
    split($i, a, "=")
    if (a[1] == "type") type[a[2]]++
    else if (a[1] == "bytes") bytes += a[2]
    else if (a[1] == "raw_mb") raw_mb += a[2]
    else if (a[1] == "raw_y_mb") raw_y_mb += a[2]
    else if (a[1] == "raw_uv_mb") raw_uv_mb += a[2]
    else if (a[1] == "rawuv_res_mb") rawuv_res_mb += a[2]
    else if (a[1] == "res_mb") res_mb += a[2]
    else if (a[1] == "res4") res4 += a[2]
    else if (a[1] == "raw4") raw4 += a[2]
    else if (a[1] == "res_coeff") res_coeff += a[2]
    else if (a[1] == "copy16") copy16 += a[2]
    else if (a[1] == "copy16_dc") copy16_dc += a[2]
  }
}
END {
  printf("key=%d p=%d bytes=%d raw_mb=%d raw_y_mb=%d raw_uv_mb=%d rawuv_res_mb=%d res_mb=%d res4=%d raw4=%d res_coeff=%d copy16=%d copy16_dc=%d\n",
    type["raw_key"], type["p"], bytes, raw_mb, raw_y_mb, raw_uv_mb,
    rawuv_res_mb, res_mb, res4, raw4, res_coeff, copy16, copy16_dc)
}' tmp/o3vpx_2d/encode.log
```

## Build The Old3DS Harness

Use the local devkitPro tree:

```sh
export DEVKITPRO=/opt/devkitpro
export DEVKITARM=/opt/devkitpro/devkitARM
export PATH="$DEVKITPRO/tools/bin:$DEVKITARM/bin:$PATH"
```

Build:

```sh
make -C libvpx/old3ds clean
make -C libvpx/old3ds
```

Output:

```text
libvpx/old3ds/build/o3vpxbench.3dsx
```

Optional build knobs:

```sh
make -C libvpx/old3ds \
  O3VPX_TARGET_US=41666 \
  O3VPX_BENCH_ITERATIONS=4 \
  O3VPX_SD_STREAM_PATH='sdmc:/o3vpxbench.o3vx'
```

## Run On Physical Old3DS

Copy these files to the SD-card root:

```text
o3vpxbench.3dsx
o3vpxbench.o3vx
```

Example:

```sh
cp libvpx/old3ds/build/o3vpxbench.3dsx /path/to/sdroot/o3vpxbench.3dsx
cp tmp/o3vpx_2d/current_8m.o3vx /path/to/sdroot/o3vpxbench.o3vx
```

Launch `o3vpxbench.3dsx` from the Homebrew Launcher. The harness:

```text
loads:  sdmc:/o3vpxbench.o3vx
writes: sdmc:/o3yvbench.log
bench:  decode + split-copy timing, 4 iterations by default
plays:  top-screen stereo RGB565 preview after the timing run
exit:   START
```

After the run, copy `sdmc:/o3yvbench.log` back to the host.

Validate the hardware log:

```sh
libvpx/tools/o3vpx-old3ds-check-log.sh /path/to/o3yvbench.log 201 41666
```

For a candidate directory, import the physical hardware log:

```sh
libvpx/tools/o3vpx-candidate-import-old3ds-log.sh \
  proof/candidates/<candidate-dir> \
  /path/to/o3yvbench.log \
  201 \
  41666
```

Do not import Azahar logs as final hardware evidence. Azahar is useful only for
functional smoke testing.

## Package A Simple SD Bundle

```sh
mkdir -p tmp/o3vpx_2d/sdcard
cp libvpx/old3ds/build/o3vpxbench.3dsx tmp/o3vpx_2d/sdcard/o3vpxbench.3dsx
cp tmp/o3vpx_2d/current_8m.o3vx tmp/o3vpx_2d/sdcard/o3vpxbench.o3vx
sha256sum tmp/o3vpx_2d/sdcard/o3vpxbench.3dsx \
  tmp/o3vpx_2d/sdcard/o3vpxbench.o3vx \
  > tmp/o3vpx_2d/sdcard/MANIFEST.sha256
```

## Common Checks

Confirm frame count and bitrate:

```sh
stream=tmp/o3vpx_2d/current_8m.o3vx
bytes=$(wc -c <"$stream")
python3 - <<PY
bytes = $bytes
frames = 201
fps = 24
print(f"bitrate_mbps={bytes * 8 * fps / frames / 1_000_000:.6f}")
PY
```

Confirm decoded size:

```sh
wc -c tmp/o3vpx_2d/decoded.yuv
```

For 201 frames it must be:

```text
57888000
```

Check keyframes:

```sh
grep '^frame ' tmp/o3vpx_2d/encode.log | grep 'type=raw_key'
```

Check host decode speed:

```sh
grep '^o3vpx_decode_summary ' tmp/o3vpx_2d/decnull.log
```

## Current 2D Sanity Result

The latest `2.mp4` direct-scale sanity run used `/mnt/hgfs/deb13/3d/2.mp4`
frames `1000..1200`, scaled directly to `800x240`:

```text
artifact:      tmp/o3vpx_2mp4_f1000_1200_2d_scale800_r001/
stream:        current_8m.o3vx
stream bytes:  3,562,104
real bitrate:  3.402607 Mbit/s
encode speed:  36.569 fps
host decnull:  11138.879 fps
host decode:   5257.603 fps to YUV
PSNR average:  52.855043 dB
Y PSNR:        51.864739 dB
worst frame:   45.655401 dB
keyframes:     source frames 1000, 1048, 1068, 1116, 1137, 1185
```

The decoded PNGs for that run are under:

```text
tmp/o3vpx_2mp4_f1000_1200_2d_scale800_r001/png/
```

## Notes

- Raw keyframes are full `800x240` YUV420P frames.
- P-frames use O3VPX modes such as `COPY16`, `COPY16_RES4`,
  `COPY16_RES4_RAWUV`, raw 4x4 repair, and VP8-style 4x4 residuals.
- The current libvpx O3VPX path stores SBS as one `800x240` frame but clamps
  motion search to the current eye, so left-eye blocks do not copy from the
  right eye.
- The Old3DS harness splits decoded SBS into separate left/right eye buffers
  with `vp8_o3vpx_decoder_write_current_yuv420p()`.
- Keep `encode.log`, `decode.log`, `decnull.log`, `psnr.log`, SHA-256 values,
  and PNG labels with every serious run.
