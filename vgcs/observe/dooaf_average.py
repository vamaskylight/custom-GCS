"""Averaging a fire correction over several rounds.

Client request 2026-08-31: "if we did the 2 DOOAF test then how can we calculate
the average". Until now VGCS produced one correction per report, from the most
recent impact only, so two tests meant two reports and arithmetic on paper.

There is one way to get this wrong that matters, and it is the obvious way:
**averaging the miss distances**. Two rounds landing 19 m short and 12 m long
are not "15 m off on average" — they are 3.5 m long on average, with 15 m of
scatter. Averaging magnitudes throws away direction and produces a correction
that is not merely imprecise but meaningless.

So the components are averaged, never the distances, and the result is reported
as two separate numbers because they call for different responses:

* **Bias** — the mean miss. This is systematic and it is what the correction
  cancels.
* **Dispersion** — how far the rounds sit from that mean. This is scatter. It
  cannot be corrected, and an operator who "corrects" for it will chase their
  own noise from round to round.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "RoundMiss",
    "AveragedCorrection",
    "average_rounds",
]


@dataclass(frozen=True)
class RoundMiss:
    """One observed round: where it landed relative to the aim point."""

    north_m: float
    east_m: float
    label: str = ""

    @property
    def distance_m(self) -> float:
        return math.hypot(self.north_m, self.east_m)


@dataclass(frozen=True)
class AveragedCorrection:
    """Bias and dispersion over a group of rounds."""

    rounds: int
    mean_north_m: float
    mean_east_m: float
    dispersion_m: float
    """RMS distance of the rounds from their own mean — the scatter that no
    correction removes."""
    worst_round_m: float
    per_round: tuple[RoundMiss, ...] = ()

    @property
    def mean_miss_m(self) -> float:
        """Magnitude of the AVERAGE miss. Deliberately not the average of the
        magnitudes — see the module docstring."""
        return math.hypot(self.mean_north_m, self.mean_east_m)

    @property
    def correction_north_m(self) -> float:
        return -self.mean_north_m

    @property
    def correction_east_m(self) -> float:
        return -self.mean_east_m

    def along_across(self, firing_bearing_deg: float) -> tuple[float, float]:
        """Mean miss rotated onto a gun line: ``(along, right)``.

        Positive along is beyond the target; positive right is right of the
        line, matching :class:`~vgcs.observe._dooaf_types.FireCorrection`.
        """
        b = math.radians(float(firing_bearing_deg))
        along = self.mean_north_m * math.cos(b) + self.mean_east_m * math.sin(b)
        right = -self.mean_north_m * math.sin(b) + self.mean_east_m * math.cos(b)
        return (along, right)

    def summary(self) -> str:
        return (
            f"{self.rounds} rounds · mean miss {self.mean_miss_m:.1f} m "
            f"({_ns(self.mean_north_m)}, {_ew(self.mean_east_m)}) · "
            f"spread {self.dispersion_m:.1f} m"
        )


def average_rounds(rounds: list[RoundMiss]) -> AveragedCorrection | None:
    """Mean miss and dispersion. ``None`` for an empty group.

    A single round is a valid group — it just has zero dispersion, which is
    honest: one round tells you nothing about scatter.
    """
    pts = [r for r in rounds if r is not None]
    if not pts:
        return None
    n = float(len(pts))
    mean_n = sum(r.north_m for r in pts) / n
    mean_e = sum(r.east_m for r in pts) / n
    # RMS distance from the mean point. Population form, not sample: this
    # describes the rounds observed, it does not estimate a wider population.
    var = sum((r.north_m - mean_n) ** 2 + (r.east_m - mean_e) ** 2 for r in pts) / n
    return AveragedCorrection(
        rounds=len(pts),
        mean_north_m=mean_n,
        mean_east_m=mean_e,
        dispersion_m=math.sqrt(var),
        worst_round_m=max(r.distance_m for r in pts),
        per_round=tuple(pts),
    )


def rounds_from_marks(
    impact_points: list[tuple[float, float]],
    target: tuple[float, float],
) -> list[RoundMiss]:
    """Convert impact lat/lons into misses about one aim point."""
    out: list[RoundMiss] = []
    for i, (lat, lon) in enumerate(impact_points or []):
        try:
            la, lo = float(lat), float(lon)
        except (TypeError, ValueError):
            continue
        north = (la - float(target[0])) * 111320.0
        east = (
            (lo - float(target[1]))
            * 111320.0
            * math.cos(math.radians(float(target[0])))
        )
        out.append(RoundMiss(north_m=north, east_m=east, label=f"Round {i + 1}"))
    return out


def _ns(v: float) -> str:
    return f"{abs(v):.1f} m {'N' if v >= 0 else 'S'}"


def _ew(v: float) -> str:
    return f"{abs(v):.1f} m {'E' if v >= 0 else 'W'}"
