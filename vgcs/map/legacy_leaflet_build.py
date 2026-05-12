"""Build legacy Leaflet+Cesium HTML (git e48c1a7) for optional 3D WebEngine overlay."""

from __future__ import annotations

import base64
from pathlib import Path
from urllib.parse import quote

_leaflet_template: str | None = None


def _template() -> str:
    global _leaflet_template
    if _leaflet_template is None:
        p = Path(__file__).with_name("legacy_leaflet_map.html")
        _leaflet_template = p.read_text(encoding="utf-8")
    return _leaflet_template


def build_leaflet_html() -> str:
    """Substitute asset paths into the vendored template (same logic as e48c1a7 `MapWidget._build_leaflet_html`)."""
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

    html = _template().replace("__LOGO_SRC__", logo_src)
    for token, data_uri in icon_data.items():
        html = html.replace(token, data_uri)
    for token, data_uri in plan_tpl_images.items():
        html = html.replace(token, data_uri)
    return html
