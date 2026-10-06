# VAMA GCS logo files

The app name is **VAMA GCS** (given by the client on 2026-10-07).
This folder holds the logo with that name, as SVG and PNG.

## Files

| File | What it is | Use it on |
|------|------------|-----------|
| `vama-mark.svg`, `.png` | The drone mark alone | Any background |
| `vama-gcs-stacked-on-dark.svg`, `.png` | Mark above the name, light letters | Dark backgrounds |
| `vama-gcs-stacked-on-light.svg`, `.png` | Mark above the name, dark letters | Light backgrounds |
| `vama-gcs-horizontal-on-dark.svg`, `.png` | Mark left of the name, light letters | Dark headers |
| `vama-gcs-horizontal-on-light.svg`, `.png` | Mark left of the name, dark letters | Light headers, documents |
| `preview.png` | All versions in one picture | Looking only |
| `vama_gcs_intro.html` | The client's 3D intro page, with the name changed to VAMA GCS | A start screen, or to show the client |

The PNG files are 2000 pixels wide with a transparent background.
The SVG files can be scaled to any size.

Colours: the mark is `#E8812A`, the light letters are `#DCE3EA`, the dark letters are `#1A212D`.

## Where the shapes come from

The client sent a 3D intro page on 2026-10-06 (`vama_gcs_intro.html`).
It holds the logo as outlines: the drone mark, and the wordmark "VAMA SKYLIGHT".
`vama_logo_outlines.json` is those outlines, unchanged.

"VAMA GCS" needs a C, and the wordmark has no C.
The C here is made from the wordmark's own G: the top half of its ring, and the mirror image of that half.
So it has the same curve, the same stroke and the same tip as the other letters.
If the client has the real font of the wordmark, its C can replace this one.

## Make the files again

From the repo root (needs Pillow):

```powershell
py branding\make_logo.py
py branding\make_logo.py --intro "vama_gcs_intro (1).html"
```

The second line also writes `vama_gcs_intro.html` from the client's page.
