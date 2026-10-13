#include "ObjectFinder.h"

#include <algorithm>
#include <cmath>

namespace skydroid::find {

namespace {

// The places of a grid along one side of the input: the last one may reach over the side.
int gridSide(int inputSide, int step)
{
    return (inputSide + step - 1) / step;
}

int inputSide(int wanted)
{
    const int rounded = (std::max(wanted, 0) + kInputStep / 2) / kInputStep * kInputStep;
    return std::clamp(rounded, kInputMin, kInputMax);
}

int upToStep(int points)
{
    return (std::max(points, 1) + kInputStep - 1) / kInputStep * kInputStep;
}

double areaOf(const Found &f)
{
    return std::max(f.w, 0.0) * std::max(f.h, 0.0);
}

// The area two boxes have in common.
double common(const Found &a, const Found &b)
{
    const double left = std::max(a.cx - a.w / 2.0, b.cx - b.w / 2.0);
    const double right = std::min(a.cx + a.w / 2.0, b.cx + b.w / 2.0);
    const double top = std::max(a.cy - a.h / 2.0, b.cy - b.h / 2.0);
    const double bottom = std::min(a.cy + a.h / 2.0, b.cy + b.h / 2.0);
    if (!(right > left) || !(bottom > top)) {
        return 0.0;
    }
    return (right - left) * (bottom - top);
}

bool surerFirst(const Found &a, const Found &b)
{
    return a.score > b.score;
}

} // namespace

bool kindOf(int label, Kind &kind)
{
    switch (label) {
    case 0:  // person
        kind = Kind::Person;
        return true;
    case 2:  // car
    case 3:  // motorcycle
    case 5:  // bus
    case 7:  // truck
        kind = Kind::Vehicle;
        return true;
    default:
        return false;
    }
}

const char *kindName(Kind kind)
{
    return kind == Kind::Person ? "person" : "vehicle";
}

int rowsFor(int inputW, int inputH)
{
    int rows = 0;
    for (const int step : kGridStep) {
        rows += gridSide(inputW, step) * gridSide(inputH, step);
    }
    return rows;
}

Fit fitArea(int areaW, int areaH, int inputW, int inputH, bool keepShape)
{
    Fit fit;
    fit.inputW = inputSide(inputW);
    fit.inputH = inputSide(inputH);
    if (areaW <= 0 || areaH <= 0) {
        return fit;  // nothing to lay in: filledW stays 0
    }
    if (keepShape) {
        const double scale = std::min(static_cast<double>(fit.inputW) / areaW, static_cast<double>(fit.inputH) / areaH);
        fit.filledW = std::clamp(static_cast<int>(std::lround(areaW * scale)), 1, fit.inputW);
        fit.filledH = std::clamp(static_cast<int>(std::lround(areaH * scale)), 1, fit.inputH);
    } else {
        fit.filledW = fit.inputW;
        fit.filledH = fit.inputH;
    }
    fit.padLeft = (fit.inputW - fit.filledW) / 2;
    fit.padTop = (fit.inputH - fit.filledH) / 2;
    fit.scaleX = static_cast<double>(fit.filledW) / areaW;
    fit.scaleY = static_cast<double>(fit.filledH) / areaH;
    return fit;
}

bool decode(const float *rows, int rowCount, int rowLength, int inputW, int inputH, double scoreMin,
            std::vector<Found> &out)
{
    if (!rows || rowLength != kRowLength || inputW <= 0 || inputH <= 0 || rowCount != rowsFor(inputW, inputH)) {
        return false;
    }
    const float *row = rows;
    for (const int step : kGridStep) {
        const int across = gridSide(inputW, step);
        const int down = gridSide(inputH, step);
        for (int y = 0; y < down; ++y) {
            for (int x = 0; x < across; ++x, row += rowLength) {
                // The class the network is surest of at this place. A person or a vehicle
                // counts only when no other class is surer: a boat is not marked as a car.
                int best = 0;
                float bestScore = row[0];
                for (int c = 1; c < kClasses; ++c) {
                    if (row[c] > bestScore) {
                        bestScore = row[c];
                        best = c;
                    }
                }
                Kind kind = Kind::Person;
                if (!(bestScore >= scoreMin) || !kindOf(best, kind)) {
                    continue;
                }
                // Each side: the mean of its steps, weighed by how likely each is.
                double side[4];
                for (int s = 0; s < 4; ++s) {
                    const float *steps = row + kClasses + s * kSideSteps;
                    const float highest = *std::max_element(steps, steps + kSideSteps);
                    double sum = 0.0;
                    double weighed = 0.0;
                    for (int i = 0; i < kSideSteps; ++i) {
                        const double share = std::exp(static_cast<double>(steps[i] - highest));
                        sum += share;
                        weighed += i * share;
                    }
                    side[s] = weighed / sum * step;
                }
                const double placeX = static_cast<double>(x) * step;
                const double placeY = static_cast<double>(y) * step;
                const double left = std::max(placeX - side[0], 0.0);
                const double top = std::max(placeY - side[1], 0.0);
                const double right = std::min(placeX + side[2], static_cast<double>(inputW));
                const double bottom = std::min(placeY + side[3], static_cast<double>(inputH));
                if (!(right > left) || !(bottom > top)) {
                    continue;  // an empty box, or numbers that are none
                }
                Found found;
                found.kind = kind;
                found.label = best;
                found.score = std::min(static_cast<double>(bestScore), 1.0);
                found.cx = (left + right) / 2.0;
                found.cy = (top + bottom) / 2.0;
                found.w = right - left;
                found.h = bottom - top;
                out.push_back(found);
            }
        }
    }
    return true;
}

void toPicture(std::vector<Found> &found, const Fit &fit, const Area &area)
{
    std::vector<Found> kept;
    kept.reserve(found.size());
    if (fit.scaleX > 0.0 && fit.scaleY > 0.0) {
        for (Found f : found) {
            const double left = std::clamp((f.cx - f.w / 2.0 - fit.padLeft) / fit.scaleX, 0.0, 1.0 * area.width);
            const double right = std::clamp((f.cx + f.w / 2.0 - fit.padLeft) / fit.scaleX, 0.0, 1.0 * area.width);
            const double top = std::clamp((f.cy - f.h / 2.0 - fit.padTop) / fit.scaleY, 0.0, 1.0 * area.height);
            const double bottom = std::clamp((f.cy + f.h / 2.0 - fit.padTop) / fit.scaleY, 0.0, 1.0 * area.height);
            if (!(right - left >= kLeastSide) || !(bottom - top >= kLeastSide)) {
                continue;  // in the black bars, or too small to be anything
            }
            f.cx = area.left + (left + right) / 2.0;
            f.cy = area.top + (top + bottom) / 2.0;
            f.w = right - left;
            f.h = bottom - top;
            kept.push_back(f);
        }
    }
    found.swap(kept);
}

double shared(const Found &a, const Found &b)
{
    const double both = common(a, b);
    const double together = areaOf(a) + areaOf(b) - both;
    return together > 0.0 ? both / together : 0.0;
}

std::vector<Found> pick(std::vector<Found> found, std::size_t most)
{
    std::stable_sort(found.begin(), found.end(), surerFirst);
    std::vector<Found> kept;
    for (const Found &f : found) {
        if (kept.size() >= most) {
            break;
        }
        bool isNew = true;
        for (const Found &k : kept) {
            if (k.kind == f.kind && shared(k, f) > kSameObjectShare) {
                isNew = false;
                break;
            }
        }
        if (isNew) {
            kept.push_back(f);
        }
    }
    return kept;
}

std::vector<Found> merge(const std::vector<Found> &whole, const std::vector<Found> &part, const Area &partArea,
                         const Area &picture)
{
    // A side of the part that is not a side of the picture cuts what lies across it.
    const double partLeft = partArea.left;
    const double partTop = partArea.top;
    const double partRight = partArea.left + partArea.width;
    const double partBottom = partArea.top + partArea.height;
    const auto cut = [&](const Found &f) {
        return (partLeft > picture.left && f.cx - f.w / 2.0 <= partLeft + kCutMargin) ||
               (partTop > picture.top && f.cy - f.h / 2.0 <= partTop + kCutMargin) ||
               (partRight < picture.left + picture.width && f.cx + f.w / 2.0 >= partRight - kCutMargin) ||
               (partBottom < picture.top + picture.height && f.cy + f.h / 2.0 >= partBottom - kCutMargin);
    };

    std::vector<Found> joined;
    std::vector<bool> wholeTaken(whole.size(), false);
    for (const Found &p : part) {
        if (cut(p)) {
            // Only a part of the object may be in this box. With the object in the whole
            // picture, that box is the one to show.
            bool inWhole = false;
            for (const Found &w : whole) {
                if (w.kind == p.kind && areaOf(p) > 0.0 && common(w, p) / areaOf(p) >= kPartOfShare) {
                    inWhole = true;
                    break;
                }
            }
            if (inWhole) {
                continue;
            }
        } else {
            // The same object in the whole picture: the part saw it larger, its box is the better one.
            for (std::size_t i = 0; i < whole.size(); ++i) {
                if (whole[i].kind == p.kind && shared(whole[i], p) > kSameObjectShare) {
                    wholeTaken[i] = true;
                }
            }
        }
        joined.push_back(p);
    }
    for (std::size_t i = 0; i < whole.size(); ++i) {
        if (!wholeTaken[i]) {
            joined.push_back(whole[i]);
        }
    }
    std::stable_sort(joined.begin(), joined.end(), surerFirst);
    return joined;
}

Plan planLooks(const Area &picture, double unitMs)
{
    Plan plan;
    plan.whole.area = picture;
    plan.whole.keepShape = true;
    plan.middle.keepShape = true;
    if (picture.width <= 0 || picture.height <= 0) {
        return plan;
    }
    // The whole picture: as large as its look allows, and never larger than it is.
    const double scale = std::min({static_cast<double>(kWholeInputW) / picture.width,
                                   static_cast<double>(kWholeInputH) / picture.height, 1.0});
    plan.whole.inputW = std::min(upToStep(static_cast<int>(std::lround(picture.width * scale))), kWholeInputW);
    plan.whole.inputH = std::min(upToStep(static_cast<int>(std::lround(picture.height * scale))), kWholeInputH);
    if (scale >= kOwnSizeScale) {
        return plan;  // the picture is small: its whole look is at its own size already
    }
    // The middle: the largest of its sizes that the device has time for.
    int size = 0;
    if (unitMs > 0.0) {
        const double unitPoints = static_cast<double>(kWholeInputW) * kWholeInputH;
        for (int i = 1; i < 4; ++i) {
            if (unitMs * kMiddleSizes[i][0] * kMiddleSizes[i][1] / unitPoints <= kMiddleBudgetMs) {
                size = i;
            }
        }
    }
    const int width = std::min(kMiddleSizes[size][0], picture.width);
    const int height = std::min(kMiddleSizes[size][1], picture.height);
    plan.closer = true;
    plan.middle.area = Area{picture.left + (picture.width - width) / 2, picture.top + (picture.height - height) / 2,
                            width, height};
    plan.middle.inputW = upToStep(width);
    plan.middle.inputH = upToStep(height);
    return plan;
}

} // namespace skydroid::find
