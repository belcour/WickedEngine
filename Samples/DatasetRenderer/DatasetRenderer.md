# DatasetRenderer

DatasetRenderer is a command-line tool that renders **paired images** of a 3D model with WickedEngine:

- **raster**: the real-time rasterizer (`RenderPath3D`). It uses shadow maps, PBR, IBL from the sky, SSAO and TAA. Optional real-time GI and SSR are available.
- **pathtraced**: the path tracer (`RenderPath3D_PathTracing`), denoised with Open Image Denoise (OIDN) when the engine was built with it.

Both renderers see the same scene, camera, lights, exposure and tonemapper. The tool randomizes cameras and lights from a seed. It produces either:

- **independent views** (the default), or
- **camera animations** with `--animate`. Each animation frame also gets a **motion vector** file.

The source is [main_SDL2.cpp](main_SDL2.cpp). The tool needs a window and a GPU (Vulkan on Linux); it cannot run headless.

---

## 1. Build and run

```sh
cmake --build build --target DatasetRenderer   # CMake option WICKED_DATASET_RENDERER (ON by default, SDL2 only)
build/Samples/DatasetRenderer/DatasetRenderer <model> [options]
```

- The first run compiles the shaders, which is slow. The shader cache directory is set with `--shader-dir`.
- `--hidden` hides the preview window. A window is still created.
- **Exit code**: `0` on success. `1` on error, or when the window was closed.
- **Progress output**: progress goes to stdout and warnings and errors to stderr.

### Input model

The tool takes a single positional argument, the model file:

| Extension | Loader |
|---|---|
| `.gltf`, `.glb`, `.vrm` | Editor glTF importer |
| `.fbx` | Editor FBX importer (ufbx) |
| `.obj` | Editor OBJ importer |
| `.wiscene` | native WickedEngine scene |

- `.blend` files are not supported; export them to glTF first.
- Animations in the model are paused, so the scene is **static**.
- Unless `--keep-scene-lighting` is given, the tool removes the file's lights, weather and probes and replaces them with its own lights.

---

## 2. Options

### Output

| Option | Default | Description |
|---|---|---|
| `-o, --out DIR` | `dataset_out` | Output directory. |
| `--width N`, `--height N` | `512` `512` | Image resolution. |
| `--views N` | `8` | Number of views, or of **animations** with `--animate`. |
| `--first-view N` | `0` | Index of the first view or animation, used to resume or split jobs. |
| `--seed N` | `0` | Random seed. View or animation `i` depends only on `(seed, i)`. |
| `--save-noisy` | off | Also save the path traced image before denoising. |
| `--save-hdr` | off | Also save linear HDR radiance as `.pfm`, taken before exposure and tonemapping. Not allowed with `--aa fxaa`. |
| `--save-aux` | off | Also save the path tracer albedo and normal buffers as `.pfm`. Requires an OIDN build. |

### Camera and focus

The camera is placed around the bounding sphere of a **focus**. The focus is a random scene object by default (`--focus object`), or the whole model (`--focus scene`). The camera looks at the focus center, or 50% of the time at a random point of the focus surface that faces the camera.

| Option | Default | Description |
|---|---|---|
| `--focus object\|scene` | `object` | What the camera is built around. |
| `--min-target-size X` | `0.1` | Minimum radius of focus objects, relative to the model radius. |
| `--fov DEG` | `45` | Vertical field of view. |
| `--dist-min X`, `--dist-max X` | `1.0` `1.4` | Camera distance, relative to the distance at which the focus sphere fits in the view. |
| `--elev-min DEG`, `--elev-max DEG` | `-5` `60` | Camera elevation range. With `--ground`, the minimum is at least 5°. |
| `--clearance X` | `0.02` | Minimum distance from the camera to any geometry, relative to the focus radius. |
| `--min-coverage X` | `0.05` | Minimum fraction of the image covered by the visible focus at the start. |
| `--min-brightness X` | `0.05` | Minimum mean display value of the focus pixels in an 8 spp path traced preview. `0` disables the check. |
| `--max-attempts N` | `200` | Camera starts or paths drawn per view before giving up. A new focus object is drawn every N/10 attempts. |

A start (a still view, or the first frame of an animation) is rejected for any of these reasons:

- **collision**: the camera is inside the clearance, or the path hits geometry.
- **occluded**: there is no room in front of occluders.
- **inside**: the camera is inside a closed object, with more than 50% back faces visible.
- **low coverage**: the visible focus covers too little of the image.
- **dark**: the path traced preview is too dark.

If no valid camera is found, **the view or animation is skipped** and a warning is printed (see [Gotchas](#8-gotchas-for-data-loading)).

### Animation

| Option | Default | Description |
|---|---|---|
| `--animate orbit\|translate\|mix\|random` | (off) | Render camera animations instead of independent views. |
| `--frames N` | `24` | Frames per animation. |
| `--orbit-min DEG`, `--orbit-max DEG` | `20` `90` | Orbit azimuth sweep. |
| `--move-min X`, `--move-max X` | `0.15` `0.5` | Translation length, relative to the camera-target distance. |
| `--min-split X` | `0.5` | A colliding path is cut to its longest valid part if that part keeps at least this fraction of the path. Otherwise a new path is drawn. |

The path types are:

- **orbit**: orbits around the aim point. Azimuth, elevation and distance are interpolated linearly.
- **translate**: moves `forward`, `backward`, `left` or `right` with a fixed orientation.
- **mix**: chains 2 or 3 orbit or translate segments. Orbits in a mix also dolly.
- **random**: picks one of the types above for each animation.

The whole camera path is kept outside the clearance. The camera speed is constant in arc length.

### Lighting

| Option | Default | Description |
|---|---|---|
| `--sun-intensity X` | `8` | Directional sun intensity. `0` disables the sun. |
| `--sun-elev-min DEG`, `--sun-elev-max DEG` | `15` `75` | Sun elevation range. The azimuth is uniform. |
| `--point-lights N` | `0` | Number of random point lights in the upper hemisphere around the model. |
| `--point-intensity X` | `4` | Point light irradiance at the model center. |
| `--sky gradient\|realistic\|none` | `gradient` | Sky model. |
| `--hdri FILE` | | Environment map (`.hdr`, `.dds`, ...) used as the sky. |
| `--rotate-hdri` | off | Random environment map rotation for each view or animation. |
| `--sky-intensity X` | `1` | Sky exposure multiplier. |
| `--ground` | off | Adds a large grey plane under the model (base color 0.5, roughness 0.8). |
| `--keep-scene-lighting` | off | Keeps the file's lights, weather and probes. |

The constant ambient term is always set to 0, because the path tracer ignores it.

### Path tracer

| Option | Default | Description |
|---|---|---|
| `--spp N` | `256` | Samples per pixel. |
| `--bounces N` | engine default (8) | Maximum bounces. |
| `--no-denoise` | off | Disables OIDN. |

### Rasterizer

| Option | Default | Description |
|---|---|---|
| `--raster-frames N` | `32` | Frames rendered before the capture, so that TAA and GI converge. |
| `--aa taa\|msaa\|fxaa\|none` | `taa` | Anti-aliasing. `msaa` uses 4 samples. |
| `--ao none\|ssao\|hbao\|msao` | `msao` | Screen-space ambient occlusion. |
| `--gi none\|ddgi\|vxgi\|surfel` | `none` | Real-time GI. It needs more `--raster-frames`. |
| `--ssr` | off | Screen-space reflections. |
| `--shadow-res N` | `2048` | Shadow map resolution. Cube shadow maps use half of it. |

### Post process (identical for both renderers)

| Option | Default |
|---|---|
| `--exposure X` | `1` |
| `--tonemap aces\|reinhard\|uchimura` | `aces` |
| `--bloom` | off |

The following effects are always disabled: eye adaptation, lens flare, light shafts, volumetric lights, depth of field, motion blur, dithering, color grading, sharpening, chromatic aberration and outlines.

---

## 3. Output layout

`NNNN` / `FFFF` are 4-digit, zero-padded indices (`0000`, `0001`, ...).

### Views mode (default)

```
DIR/
  dataset.json                    run-level settings and conventions
  meta/NNNN.json                  camera, focus, lights of view NNNN
  raster/NNNN.png                 rasterized, tonemapped sRGB
  pathtraced/NNNN.png             path traced (denoised if available), tonemapped sRGB
  pathtraced_noisy/NNNN.png       --save-noisy
  raster_hdr/NNNN.pfm             --save-hdr
  pathtraced_hdr/NNNN.pfm         --save-hdr (denoised if available)
  pathtraced_noisy_hdr/NNNN.pfm   --save-hdr --save-noisy (with denoising only)
  albedo/NNNN.pfm                 --save-aux
  normal/NNNN.pfm                 --save-aux
```

### Animation mode (`--animate`)

```
DIR/
  dataset.json
  anim_NNNN/
    animation.json                path, cameras of every frame, focus, lights
    raster/FFFF.png
    pathtraced/FFFF.png
    motion/FFFF.npy               always written in animation mode
    pathtraced_noisy/FFFF.png     same optional outputs as views mode
    raster_hdr/FFFF.pfm
    pathtraced_hdr/FFFF.pfm
    pathtraced_noisy_hdr/FFFF.pfm
    albedo/FFFF.pfm
    normal/FFFF.pfm
```

There is no `meta/` directory in animation mode; all per-frame cameras are in `animation.json`. `animation.json` is written **before** the frames are rendered, so an interrupted run can leave an animation with missing frames.

---

## 4. File formats

| Kind | Format | Content |
|---|---|---|
| `*.png` | 8-bit **RGBA**, alpha always 255 | Display-referred: exposure, tonemapper and sRGB encoding applied. Drop the alpha channel. |
| `*_hdr/*.pfm` | PFM, RGB float32, little endian (scale −1), **rows stored bottom to top** (standard PFM) | Linear radiance before exposure and tonemapping. For the raster image this is the TAA-resolved HDR buffer. |
| `albedo/*.pfm` | PFM as above | Albedo at the path tracer's first hit, averaged over the samples. 0 for the sky. |
| `normal/*.pfm` | PFM as above | **World-space** shading normal at the first hit, averaged over the samples and not renormalized (length ≤ 1 at edges). 0 for the sky. |
| `motion/*.npy` | NumPy `.npy` v1.0, `<f4`, C order, shape **(H, W, 2)** | Backward motion vectors in pixels (see [§6](#6-motion-vectors-animation-mode)). |
| `*.json` | JSON, UTF-8 | Metadata (see [§5](#5-metadata-json)). |

### Python loaders

```python
import json, numpy as np
from PIL import Image

def load_png(path):            # (H, W, 3) float32 in [0, 1], sRGB encoded
    return np.asarray(Image.open(path).convert("RGB"), np.float32) / 255.0

def load_pfm(path):            # (H, W, 3) float32, top row first
    with open(path, "rb") as f:
        assert f.readline().strip() == b"PF"
        w, h = map(int, f.readline().split())
        scale = float(f.readline())
        data = np.fromfile(f, "<f4" if scale < 0 else ">f4", count=w * h * 3)
    return np.flipud(data.reshape(h, w, 3)).copy()

def load_motion(path):         # (H, W, 2) float32, pixels
    return np.load(path)
```

---

## 5. Metadata (JSON)

All positions and directions are in world space. Angles in the per-view and per-animation files are in **radians**, unless the key says `degrees`. Matrices are 4×4 nested lists of rows (`m[row][col]`).

### `dataset.json`

The run settings. It is rewritten by every run into the same directory (see [Gotchas](#8-gotchas-for-data-loading)).

```jsonc
{
  "input": "/abs/path/model.glb",
  "mode": "views" | "animation",
  "resolution": [W, H],
  "seed": 0,
  "views": [first, end],                 // half-open range of indices of THIS run (views or animations)
  "start": { "focus", "min_target_size", "clearance", "min_coverage", "min_brightness", "max_attempts" },
  "animation": { "type", "frames", "orbit_degrees": [min, max], "move": [min, max], "min_split" },  // animation mode only
  "bounds": { "min": [x,y,z], "max": [x,y,z], "center": [x,y,z], "radius": r },  // whole model, without the ground plane
  "pathtracer": { "spp", "bounces", "denoised" },
  "rasterizer": { "frames", "aa", "ao", "gi", "ssr", "shadow_resolution" },
  "postprocess": { "exposure", "tonemap", "bloom" },
  "lighting": { "keep_scene_lighting", "sun_intensity", "point_lights", "point_intensity", "sky", "hdri", "sky_intensity", "ground" },
  "conventions": "..."                   // a text summary of section 7
}
```

### `meta/NNNN.json` (views mode)

```jsonc
{
  "view": 3,
  "target": {                            // focus
    "name": "object name",
    "object_index": 121,                 // -1 with --focus scene
    "center": [x,y,z], "radius": r,      // focus bounding sphere
    "aim": [x,y,z]                       // point the camera looks at (center or a surface point)
  },
  "start": {
    "attempts": 5,
    "rejections": { "collision": 0, "occluded": 1, "inside": 0, "low coverage": 3, "dark": 0 },
    "clearance": 0.19,                   // absolute world units
    "coverage": 0.31,                    // fraction of the image covered by the visible focus
    "brightness": 0.42                   // mean display value of the focus in the preview
  },
  "camera": {
    "eye": [x,y,z], "forward": [x,y,z], "up": [0,1,0], "target": [x,y,z],
    "fov_y_degrees": 45, "near": n, "far": f,
    "azimuth": a, "elevation": e, "distance": d,   // spherical coords of eye around "target"/aim
    "view_matrix": [[...],[...],[...],[...]],
    "projection_matrix": [[...],[...],[...],[...]]
  },
  "sun": { "direction": [x,y,z], "azimuth": a, "elevation": e, "intensity": i, "color": [1,1,1] },  // absent if no sun
  "point_lights": [ { "position": [x,y,z], "color": [r,g,b], "intensity": i, "range": r }, ... ],
  "sky_rotation": 0.0                    // radians, non-zero only with --rotate-hdri
}
```

### `anim_NNNN/animation.json` (animation mode)

```jsonc
{
  "animation": 0,
  "type": "orbit" | "translate" | "mix",
  "frame_count": 24,
  "target": { "name", "object_index", "center", "radius", "aim" },   // same as views mode
  "clearance": 0.19,
  "attempts": 44,
  "rejections": { "collision", "occluded", "inside", "low coverage", "dark" },
  "start": { "coverage": c, "brightness": b },                      // checks on frame 0
  "split": true,                         // the path was cut to its valid part
  "path_interval": [s0, s1],             // rendered part of the path, normalized arc length
  "path_length": 0.69,                   // world units, rendered part
  "segments": [
    { "type": "orbit", "pivot": [x,y,z], "distance": [d0,d1], "azimuth": [a0,a1], "elevation": [e0,e1] },
    { "type": "translate", "direction": "forward|backward|left|right", "from": [x,y,z], "to": [x,y,z], "forward": [x,y,z] }
  ],                                     // full segments, before the [s0, s1] cut
  "camera": {                            // shared by all frames
    "fov_y_degrees": 45, "near": n, "far": f, "up": [0,1,0],
    "projection_matrix": [[...],[...],[...],[...]]
  },
  "frames": [
    { "frame": 0, "eye": [x,y,z], "forward": [x,y,z], "view_matrix": [[...],[...],[...],[...]] },
    ...
  ],
  "sun": {...}, "point_lights": [...], "sky_rotation": 0.0           // same as views mode, fixed for the animation
}
```

Frame `f` samples the path at normalized arc length `s0 + (s1 − s0) · f / (frame_count − 1)`, so the camera speed is constant.

---

## 6. Motion vectors (animation mode)

`anim_NNNN/motion/FFFF.npy` is a float32 array of shape `(H, W, 2)` holding **backward** motion in **pixels**:

- channel 0 is `u` (x, to the right) and channel 1 is `v` (y, down);
- pixel `(x, y)` of frame `F` shows the surface point that was at the continuous pixel position `(x + u, y + v)` in frame `F − 1`. Integer coordinates are pixel centers, the same as the pixel indices;
- **frame 0 is all zeros**, because it has no previous frame;
- **NaN** where the point was behind the previous camera, which is rare;
- the **sky** is treated as infinitely far away, so it only moves with the camera rotation.

How the vectors are computed:

- They come from the **rasterizer depth** of opaque geometry and the exact camera matrices of both frames. Transparent surfaces get the motion of the opaque surface behind them.
- The TAA jitter is compensated, so the vectors are unjittered and apply to the `raster` and `pathtraced` images alike.
- The scene is static, so all motion comes from the camera.
- Disocclusions are **not** flagged. A pixel that was hidden in frame F−1 still gets the motion of its own surface point, which lands on the occluder in frame F−1. To build a validity mask, compare warped and current images, or reproject depths yourself.

**Warping frame F−1 to frame F** (backward warp):

```python
import numpy as np, torch, torch.nn.functional as F

mv   = torch.from_numpy(np.load(f"{anim}/motion/{f:04d}.npy"))          # (H, W, 2)
prev = torch.from_numpy(load_png(f"{anim}/pathtraced/{f-1:04d}.png"))   # (H, W, 3)
H, W = mv.shape[:2]
ys, xs = torch.meshgrid(torch.arange(H), torch.arange(W), indexing="ij")
px = xs + mv[..., 0]                      # source position in frame f-1 (pixel-center coords)
py = ys + mv[..., 1]
grid = torch.stack([(px + 0.5) / W * 2 - 1, (py + 0.5) / H * 2 - 1], -1)  # align_corners=False
valid = torch.isfinite(grid).all(-1) & (grid.abs() <= 1).all(-1)
warped = F.grid_sample(prev.permute(2, 0, 1)[None], torch.nan_to_num(grid)[None],
                       mode="bilinear", padding_mode="border", align_corners=False)[0].permute(1, 2, 0)
```

On test data, the backward warp with these vectors had its lowest error at scale 1 with no sub-pixel offset. That means the vectors have no scale or half-pixel bias.

---

## 7. Conventions

- **World space**: left-handed, **Y up**. The camera `up` is always `(0, 1, 0)`, and the camera looks along `forward`.
- **Spherical directions**: azimuth `a` and elevation `e` map to `(cos e · cos a, sin e, cos e · sin a)`. A camera at azimuth `a`, elevation `e` and distance `d` around a point `C` has `eye = C + d · dir(a, e)` and `forward = −dir(a, e)`.
- **Sun**: `sun.direction` points **from the scene towards the sun**, using the same spherical mapping.
- **Matrices** are row-major for **row vectors** (DirectXMath): `p_view = [x, y, z, 1] @ V` and `p_clip = p_view @ P`. Transpose them for column-vector code.
  - `view_matrix` is world → view space. View space has +z forward.
  - `projection_matrix` is a perspective projection with **reversed Z**: NDC z is 1 at `near` and 0 at `far`. NDC x and y are in [−1, 1], with +y up. The matrix is **unjittered**.
  - The view-space depth of an NDC depth `d` is `z_view = P[3][2] / (d − P[2][2])`.
  - In animation mode, `near` and `far` are the same for every frame of an animation and are fitted to the whole path. They vary between animations and views.
- **Pixels**: images are stored **top row first**, apart from the PFM files on disk, which the loader flips. The center of pixel `(x, y)` maps to:
  - `ndc_x = (x + 0.5) / W · 2 − 1`
  - `ndc_y = 1 − (y + 0.5) / H · 2`
- **Projecting a world point** to pixel coordinates:
  ```python
  V = np.array(frame["view_matrix"]); P = np.array(cam["projection_matrix"])
  c = np.array([*p_world, 1.0]) @ V @ P
  x = (c[0] / c[3] * 0.5 + 0.5) * W - 0.5     # pixel-center coords
  y = (0.5 - c[1] / c[3] * 0.5) * H - 0.5
  ```
- **Aspect ratio**: `P[0][0] = P[1][1] · H / W`, and `fov_y_degrees` is the vertical FOV.
- **Color**:
  - PNG files are tonemapped and sRGB encoded.
  - PFM radiance is linear and before exposure. Multiply it by `exposure` to get the tonemapper input.
  - Raster and path traced images share the exposure and tonemapper.

---

## 8. Gotchas for data loading

- **Indices can have gaps.** A view or animation with no valid camera within `--max-attempts` is skipped with a warning on stderr. Build the index list from the files that exist (`meta/*.json` or `anim_*/animation.json`), not from `range(views)`.
- **Incomplete animations.** `animation.json` is written before the frames are rendered, so check that every `FFFF` in `range(frame_count)` exists in every directory you use.
- **Split or resumed jobs.** Several runs with different `--first-view` values can write into the same `DIR`, and each rerun overwrites `dataset.json`. Its `views` field then only covers the last run. Keep all runs on the same settings and seed.
- **Determinism.** View or animation `i` is reproducible from `(seed, i)`: the same camera, focus and lights. Pixel values can differ slightly between GPUs and drivers.
- **Raster vs. path traced differences** come from the rasterizer itself:
  - Without `--gi`, the rasterizer has no indirect lighting apart from sky IBL and SSAO. Interiors are lit differently.
  - Shadows come from shadow maps, and there are no glossy interreflections unless `--ssr` is used.
- **Aux buffers** (`albedo`, `normal`) only exist when the engine was built with OIDN. Without OIDN a warning is printed and the files are missing.
- **Noisy outputs.** With `--no-denoise`, `pathtraced/` holds the noisy image and there is no `pathtraced_noisy*/` directory.
- **No depth output.** Depth is only used internally for the motion vectors. Camera matrices are provided for geometric supervision.

---

## 9. Example commands

```sh
# 1000 still pairs at 512x512, Sponza, 256 spp, with HDR + aux buffers
DatasetRenderer Content/models/Sponza/Sponza.wiscene -o data/sponza_views \
    --views 1000 --spp 256 --save-hdr --save-aux --hidden

# 100 animations of 24 frames (pairs + motion vectors), split in 4 jobs of 25
for i in 0 25 50 75; do
  DatasetRenderer model.glb -o data/anims --animate random --frames 24 \
      --views 25 --first-view $i --seed 7 --hidden
done

# Small, quick test
DatasetRenderer Content/models/DamagedHelmet.glb -o /tmp/test --animate random \
    --views 3 --frames 6 --spp 16 --raster-frames 8 --width 256 --height 192
```
