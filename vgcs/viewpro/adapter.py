"""Viewpro/ViewLink gimbal — TCP control-port client.

The ViewLink Software User Manual (V4.0.9, 2025-11-17) documents the network
transport (TCP port 2000 by default, §3.2.1(7)) and shows one example raw
frame for its "M-packet" protocol (§3.7 Extended Command), but does not
publish the command-ID table for pan/tilt/zoom/focus/photo/record/track
control — those bytes are not in this manual. See
DOCS/VIEWPRO-CAMERA-REFERENCE.md "Known gaps" for what's missing and why
PTZ/zoom/focus/photo/record are deliberately left as diagnostic no-ops below
rather than guessed at.

``send_raw_command`` is the one fully-specified capability this manual gives:
it mirrors the software's own "Extended Command" debug panel (§3.7) — send
arbitrary bytes to the gimbal's TCP control port. Useful once the real
command bytes are known (from Viewpro's protocol/SDK doc) or for capturing
ViewLink's own traffic to reverse-engineer specific commands.
"""

from __future__ import annotations

from vgcs.skydroid import GimbalStatus
from vgcs.viewpro.transport import ViewproTcpTransport

_NOT_IMPLEMENTED_HINT = (
    "not yet implemented — the ViewLink software manual does not publish the "
    "PTZ/zoom/focus/photo/record command bytes (only an example M-packet frame). "
    "See DOCS/VIEWPRO-CAMERA-REFERENCE.md."
)


class ViewproGimbalTcpAdapter:
    """TCP client to the Viewpro gimbal core's control port (default 2000)."""

    def __init__(
        self,
        *,
        host: str,
        port: int = 2000,
        timeout_s: float = 1.0,
    ) -> None:
        self._transport = ViewproTcpTransport(host, port, timeout_s=timeout_s)
        self._warned_actions: set[str] = set()

    def start(self) -> None:
        # Lazy-connect on first use (matches SiyiGimbalUdpAdapter/transport
        # style) rather than blocking construction on a TCP handshake.
        return

    def stop(self) -> None:
        self._transport.close()

    def get_status(self) -> GimbalStatus:
        """No attitude telemetry parsing implemented — see module docstring."""
        return GimbalStatus(supported=False)

    def send_raw_command(self, payload: bytes) -> bytes:
        """Send raw bytes to the gimbal's TCP control port; returns whatever
        reply arrives within the transport timeout (possibly empty — many
        commands are fire-and-forget). Mirrors the ViewLink "Extended
        Command" debug panel (manual §3.7)."""
        return self._transport.send_and_receive(payload)

    def _not_implemented(self, action: str) -> None:
        # ptz() in particular is called continuously while a jog key/button is
        # held — print once per action kind per adapter lifetime, not per call.
        if action in self._warned_actions:
            return
        self._warned_actions.add(action)
        print(f"[VGCS:viewpro] {action} {_NOT_IMPLEMENTED_HINT}")

    def camera_photo(self) -> None:
        self._not_implemented("photo")

    def camera_record_toggle(self) -> None:
        self._not_implemented("record toggle")

    def camera_zoom(self, direction: int) -> None:
        del direction
        self._not_implemented("zoom")

    def camera_focus_step(self, direction: int) -> None:
        del direction
        self._not_implemented("focus")

    def ptz(self, action: str) -> None:
        del action
        self._not_implemented("gimbal move (PTZ)")

    def set_angle(self, yaw: float, pitch: float) -> None:
        del yaw, pitch
        self._not_implemented("gimbal angle")

    def set_rotation_speed(self, yaw: float, pitch: float) -> None:
        del yaw, pitch
        self._not_implemented("gimbal speed")
