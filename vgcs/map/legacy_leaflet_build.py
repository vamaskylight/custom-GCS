"""Build legacy Leaflet+Cesium HTML (git e48c1a7) for optional 3D WebEngine overlay."""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Iterable
from urllib.parse import quote

_leaflet_template: str | None = None

# Used only when the vendored copy is missing from a build (see below).
_CESIUM_CDN = "https://unpkg.com/cesium@1.125/Build/Cesium/"
# The satellite source of the 2D map. Its tiles, and any imported tile pack,
# are stored under a folder named after this template.
_SATELLITE_TEMPLATE = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)


def _template() -> str:
    global _leaflet_template
    if _leaflet_template is None:
        p = Path(__file__).with_name("legacy_leaflet_map.html")
        _leaflet_template = p.read_text(encoding="utf-8")
    return _leaflet_template


def local_tile_templates(offline_tile_root: str | Path | None = None) -> list[str]:
    """``file:`` URL templates of the map tiles already on this computer.

    The 3D view draws them under its online layers, so it shows the same
    ground as the 2D map when there is no internet. Bottom layer first: the
    seed tiles shipped with VGCS, the 2D map's disk cache of the satellite
    source (where "Cache area offline" and imported tile packs put their
    tiles), and the operator's offline folder when one is set.
    """
    from vgcs.map import native_tile_map as ntm

    cache = ntm._TILE_CACHE_ROOT / ntm._tile_source_id(_SATELLITE_TEMPLATE)  # noqa: SLF001
    roots: list[Path] = [ntm.bundled_seed_root(), cache]
    if offline_tile_root:
        roots.append(Path(offline_tile_root))
    # The cache is listed even before its folder exists: the page is built
    # once, and the first tile may be stored or imported after that.
    return _tile_templates_for(roots, always=(cache,))


def _tile_templates_for(roots: Iterable[Path], always: Iterable[Path] = ()) -> list[str]:
    """A ``{z}/{x}/{y}.png`` URL template for each folder that exists, or is in ``always`` (once each)."""
    keep = {Path(p).resolve() for p in always}
    out: list[str] = []
    for root in roots:
        try:
            resolved = Path(root).resolve()
            if resolved not in keep and not resolved.is_dir():
                continue
            template = resolved.as_uri() + "/{z}/{x}/{y}.png"
        except (OSError, ValueError):
            continue
        if template not in out:
            out.append(template)
    return out


def build_leaflet_html(offline_tile_root: str | Path | None = None) -> str:
    """Substitute asset paths into the vendored template (same logic as e48c1a7 `MapWidget._build_leaflet_html`).

    ``offline_tile_root`` is the operator's offline tile folder, if one is set.
    """
    assets_dir = Path(__file__).resolve().parents[1] / "assets"
    assets_root = assets_dir.resolve()

    def src_under_assets(path: Path) -> str:
        rel = path.resolve().relative_to(assets_root)
        return "/".join(quote(part, safe="") for part in rel.parts)

    logo_candidates = [
        assets_dir / "Vama Logo.png",
        assets_dir / "vama_logo.jpg",
        Path(__file__).resolve().parents[2] / "Vama Logo New.png",
    ]
    logo_src = ""
    for p in logo_candidates:
        if not p.is_file():
            continue
        pr = p.resolve()
        try:
            logo_src = src_under_assets(pr)
            break
        except ValueError:
            try:
                raw = pr.read_bytes()
            except Exception:
                continue
            if not raw:
                continue
            mime = "image/png" if pr.suffix.lower() == ".png" else "image/jpeg"
            logo_src = f"data:{mime};base64,{base64.b64encode(raw).decode('ascii')}"
            break
    icon_files = {
        "__ICON_HOLD_SRC__": assets_dir / "header_icons" / "hold.svg",
        "__ICON_LINK_SRC__": assets_dir / "header_icons" / "link.svg",
        "__ICON_GPS_SRC__": assets_dir / "header_icons" / "gps.svg",
        "__ICON_BATTERY_SRC__": assets_dir / "header_icons" / "battery.svg",
        "__ICON_REMOTE_ID_SRC__": assets_dir / "header_icons" / "remote_id.svg",
    }
    icon_data: dict[str, str] = {}
    for token, icon_path in icon_files.items():
        if not icon_path.is_file():
            icon_data[token] = ""
            continue
        try:
            icon_data[token] = src_under_assets(icon_path)
        except ValueError:
            icon_data[token] = ""

    empty_plan_src = ""
    for _empty_name in ("empty plan.png", "emtpy plan.png"):
        ep = assets_dir / _empty_name
        if ep.is_file():
            try:
                empty_plan_src = src_under_assets(ep)
            except ValueError:
                empty_plan_src = quote(_empty_name, safe="")
            break
    survey_p = assets_dir / "survey.png"
    corr_p = assets_dir / "Corridor Scan.png"
    stru_p = assets_dir / "Structure Scan.png"
    plan_tpl_images = {
        "__PLAN_TPL_EMPTY_SRC__": empty_plan_src,
        "__PLAN_TPL_SURVEY_SRC__": src_under_assets(survey_p) if survey_p.is_file() else "",
        "__PLAN_TPL_CORRIDOR_SRC__": src_under_assets(corr_p) if corr_p.is_file() else "",
        "__PLAN_TPL_STRUCTURE_SRC__": src_under_assets(stru_p) if stru_p.is_file() else "",
    }

    # Leaflet is vendored under assets/vendor so the map renders with no
    # internet. If the vendored copy is somehow absent, leave the CDN URL in
    # place rather than an empty src — the page's own onerror then falls back.
    leaflet_dir = assets_dir / "vendor" / "leaflet"
    leaflet_js = leaflet_dir / "leaflet.js"
    leaflet_css = leaflet_dir / "leaflet.css"
    js_src = (
        src_under_assets(leaflet_js)
        if leaflet_js.is_file()
        else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    )
    css_src = (
        src_under_assets(leaflet_css)
        if leaflet_css.is_file()
        else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    )

    # Cesium (the 3D view) is vendored the same way, and for the same reason.
    # Its base URL is absolute: Cesium loads its workers and textures from it,
    # and a relative one would be resolved against the wrong place.
    cesium_dir = assets_dir / "vendor" / "cesium"
    cesium_js = cesium_dir / "Cesium.js"
    cesium_css = cesium_dir / "Widgets" / "widgets.css"
    if cesium_js.is_file() and cesium_css.is_file():
        cesium_js_src = src_under_assets(cesium_js)
        cesium_css_src = src_under_assets(cesium_css)
        cesium_base = cesium_dir.resolve().as_uri() + "/"
    else:
        cesium_js_src = _CESIUM_CDN + "Cesium.js"
        cesium_css_src = _CESIUM_CDN + "Widgets/widgets.css"
        cesium_base = _CESIUM_CDN

    html = _template().replace("__LOGO_SRC__", logo_src)
    html = html.replace("__LEAFLET_JS_SRC__", js_src)
    html = html.replace("__LEAFLET_CSS_SRC__", css_src)
    html = html.replace("__CESIUM_JS_SRC__", cesium_js_src)
    html = html.replace("__CESIUM_CSS_SRC__", cesium_css_src)
    html = html.replace("__CESIUM_BASE_URL__", cesium_base)
    html = html.replace(
        "__VGCS_LOCAL_TILE_TEMPLATES_JSON__",
        json.dumps(local_tile_templates(offline_tile_root)),
    )
    for token, data_uri in icon_data.items():
        html = html.replace(token, data_uri)
    for token, data_uri in plan_tpl_images.items():
        html = html.replace(token, data_uri)
    return html
