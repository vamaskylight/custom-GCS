# The object finder's network: where it comes from

The app marks people and vehicles in the video (`apk/custom/src/Skydroid/ObjectFinder.h`).
Two files in this folder are the neural network that does it.
They go into the app through `custom.qrc` (as `:/custom/ai/finder.param` and `:/custom/ai/finder.bin`).

## The network: NanoDet-Plus-m, 416

- Made by: RangiLyu and the NanoDet contributors, https://github.com/RangiLyu/nanodet
- Release: `v1.0.0-alpha-1` (2021-12-26), file `nanodet-plus-m_416_ncnn.zip`
- Fetched: 2026-10-13, from https://github.com/RangiLyu/nanodet/releases/download/v1.0.0-alpha-1/nanodet-plus-m_416_ncnn.zip
- Licence: Apache License 2.0 (the `LICENSE` file of that release, copied into `NOTICES.txt`)
- The two files are the ones in the zip, unchanged.
- It was trained on the COCO pictures (80 kinds of things). The app marks five of them: person, car, motorcycle, bus, truck.
- Its makers give 30.4 mAP on COCO for it, with 1.17 million weights.

| File | Bytes | SHA-256 |
|------|-------|---------|
| `nanodet-plus-m_416_ncnn.zip` (the download, not kept here) | 2,218,050 | `16696bb22b77f04d2e098ec1c2ac079d10e7ea94e2c1e0228934345a1bc429e1` |
| `nanodet-plus-m_416.param` (the layer list, text) | 23,123 | `d79e18ecd8595081bb29fdf0c790a47fcea96fbf58598350569a92e71e915b72` |
| `nanodet-plus-m_416.bin` (the weights) | 2,397,184 | `f7555ca6cb9650ee41e2f0a53fdf46b6e187a2a09792208d8c3eec033b6ac615` |

The app checks the two SHA-256 values before it uses the files (`ObjectMarksNet.cc`).
With other files it marks nothing and says why.
The layer list is a text file. `.gitattributes` in this folder tells git to leave both files byte for byte as they are: without it a checkout on Windows changes the line ends, and the app refuses the file.
The release page gives no hash for the zip: the values above are of the files as they came on 2026-10-13.

Check them again with:

```
sha256sum apk/custom/res/ai/nanodet-plus-m_416.param apk/custom/res/ai/nanodet-plus-m_416.bin
```

## The library that runs it: ncnn

- Made by: Tencent and the ncnn contributors, https://github.com/Tencent/ncnn
- Release: `20260526`, file `ncnn-20260526-full-source.zip`, SHA-256 `754659d6fe65545cf2ef4483ffb84526fea631f8764c44b150f1601d0fb4004b` (as the release page gives it)
- Licence: BSD 3-Clause, with parts under the zlib License (its `LICENSE.txt`, copied into `NOTICES.txt`)
- It is not kept in this repo. `apk/build_ncnn.ps1` downloads the source, checks the hash, and builds the library for each Android ABI (and for this PC, for the tests).
- Built with one thread, without the graphics card (Vulkan off, OpenMP off).

## Why these two

- Both licences allow use in a closed, commercial app.
  (The YOLO networks of Ultralytics are AGPL. Networks trained on the VisDrone drone pictures are for non-commercial use only. Neither is used.)
- The network is small enough for the processor of a remote control: one look at 640 x 384 points takes about 45 ms on the development PC, one thread.
- VGCS (the Windows program) uses a larger network of the same kind of licence, YOLOX-S, see `vgcs/assets/models/MODEL_SOURCE.md`. That one is about seven times too slow for a remote control.

## The notices

`NOTICES.txt` holds both licence texts.
The app shows it in the camera settings ("Licences of the object finder").
Keep it in step when the network or the library is replaced.
