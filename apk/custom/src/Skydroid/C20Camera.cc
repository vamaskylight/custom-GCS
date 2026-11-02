#include "C20Camera.h"

namespace c20 {

Bytes packet(std::uint8_t group, std::uint8_t command, const Bytes &payload)
{
    Bytes out(6, 0);
    out.push_back(group);
    out.push_back(command);
    out.push_back(static_cast<std::uint8_t>((payload.size() >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(payload.size() & 0xFF));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

Bytes zoomIn()
{
    return packet(kGroupZoom, kCmdZoomMove, {0x0B, 0x00, 0x01});
}

Bytes zoomOut()
{
    return packet(kGroupZoom, kCmdZoomMove, {0x0C, 0x00, 0x01});
}

Bytes zoomStop()
{
    return packet(kGroupZoom, kCmdZoomMove, {0x0B, 0x01, 0x01});
}

Bytes zoomQuery()
{
    return packet(kGroupZoom, kCmdZoomRead, {0x00, 0x00});
}

std::vector<Bytes> splitPackets(const Bytes &read)
{
    std::vector<Bytes> out;
    std::size_t i = 0;
    while (read.size() - i >= kHeaderLen) {
        const std::size_t length = (static_cast<std::size_t>(read[i + 8]) << 8) | read[i + 9];
        const std::size_t end = i + kHeaderLen + length;
        const bool shortZoom = read[i + 6] == kGroupZoom && read[i + 7] == kCmdZoomRead && length < kZoomReadMinPayload;
        if (end > read.size() || shortZoom) {
            out.emplace_back(read.begin() + static_cast<std::ptrdiff_t>(i), read.end());
            break;
        }
        out.emplace_back(read.begin() + static_cast<std::ptrdiff_t>(i), read.begin() + static_cast<std::ptrdiff_t>(end));
        i = end;
    }
    return out;
}

std::optional<int> zoomOfReply(const Bytes &packet)
{
    if (packet.size() < kHeaderLen + kZoomReadMinPayload) {
        return std::nullopt;
    }
    if (packet[6] != kGroupZoom || packet[7] != kCmdZoomRead) {
        return std::nullopt;
    }
    const int value = packet[kHeaderLen + 9];
    if (value >= 1 && value <= 3 * kZoomMax) {
        return (value - 1) / 3 + 1;
    }
    return std::nullopt;
}

} // namespace c20
