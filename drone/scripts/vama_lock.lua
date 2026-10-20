--[[
   VAMA lock for ArduCopter (4.6 and newer), part 1 of the firmware lock:
   the drone arms only while a bound ground station is connected.

   Bound means: the drone holds the team's MAVLink signing key (sent to it by
   VGCS or the VAMA app), and the ground station signs everything it sends
   with the same key. With a key on the drone, ArduPilot itself drops every
   unsigned or wrongly signed message on its telemetry ports, so nobody else
   can command the drone over the radio. What signing cannot stop is a pilot
   who only uses the sticks. This script stops that: arming, by sticks, switch
   or ground station, is refused until a signed heartbeat of a ground station
   arrives on a telemetry port, and for VLCK_HOLD seconds after the last one.

   It only takes part in the arming checks, so it never acts on a drone that
   is armed or flying.

   What it does not stop (see vama_lock.md): anyone with the USB port, the SD
   card or a firmware flasher (they can turn it off), a forced arm (that needs
   the key or USB), and arming with all arming checks turned off. Without a
   signing key on the drone it stops only a pilot who has no ground station:
   any ground station can turn it off.
--]]

local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

local LOOP_MS = 100             -- 10 times a second
local STATUS_EVERY_MS = 1000    -- the lock state for ground stations (VLCK), always
local WARN_EVERY_MS = 30000     -- at most one warning text of a kind every 30 s
local QUEUE_LENGTH = 20         -- received heartbeats kept between two loops

local MSG_HEARTBEAT = 0
local MAV_TYPE_GCS = 6
local HEARTBEAT_CRC_EXTRA = 50
local IFLAG_SIGNED = 0x01
-- MAVLink channel 0 is the USB port. ArduPilot takes unsigned and wrongly
-- signed messages there even with a key, so nothing on it unlocks.
local USB_CHANNEL = 0

-- A received message as ArduPilot hands it to scripts (mavlink_message_t),
-- byte positions from 1: checksum as ArduPilot worked it out 1-2, magic 3,
-- payload length 4, incompat flags 5, compat flags 6, seq 7, system id 8,
-- component id 9, message id 10-12, payload 13-276, checksum as received
-- 277-278, signature 279-291.
local POS_MAGIC, POS_LEN, POS_INCOMPAT, POS_SEQ, POS_SYSID = 3, 4, 5, 7, 8
local POS_MSGID, POS_PAYLOAD, POS_CK = 10, 13, 277
local MESSAGE_BYTES = 291
local MAGIC_V2, MAGIC_V1 = 0xFD, 0xFE

local STATE_OFF, STATE_LOCKED, STATE_UNLOCKED = 0, 1, 2

-- 4.7 skips all arming checks when ARMING_SKIPCHK holds all of these bits.
local ALL_CHECKS_47 = 0x1FFFFE

---------------------------------------------------------------------------
-- Parameters
---------------------------------------------------------------------------

-- Must be a number that no other script on this flight controller uses (0 to 200).
-- The wind failsafe uses 196.
local PARAM_TABLE_KEY = 197
local PARAM_TABLE_PREFIX = "VLCK_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "VAMA lock: could not add parameter " .. name)
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), "VAMA lock: could not add the parameter table")

--[[
  // @Param: VLCK_ENABLE
  // @DisplayName: VAMA lock
  // @Description: 1: the drone arms only while a ground station that signs with the drone's MAVLink signing key is connected on a telemetry port. Set it only after the signing key is on the drone. 0: off.
  // @Values: 0:Off,1:On
  // @User: Standard
--]]
local VLCK_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: VLCK_HOLD
  // @DisplayName: VAMA lock hold time
  // @Description: How long a signed heartbeat of a ground station keeps the drone unlocked. Ground stations send one a second.
  // @Units: s
  // @Range: 2 60
  // @User: Advanced
--]]
local VLCK_HOLD = bind_add_param("HOLD", 2, 5)

---------------------------------------------------------------------------
-- State
---------------------------------------------------------------------------

local auth_id = arming:get_aux_auth_id()
-- ArduPilot hands out 3 of these until the next boot.
assert(auth_id ~= nil, "VAMA lock: no arming check free, reboot the flight controller")

mavlink:init(QUEUE_LENGTH, 1)
local heartbeats_are_ours = mavlink:register_rx_msgid(MSG_HEARTBEAT)

local state = nil              -- STATE_OFF, STATE_LOCKED or STATE_UNLOCKED, nil before the first loop
local last_signed_ms = nil     -- the last signed heartbeat of a ground station on a telemetry port
local last_signed_sysid = 0
local last_signed_chan = 0
local last_status_ms = -STATUS_EVERY_MS
local last_warn_ms = {unsigned = -WARN_EVERY_MS, usb = -WARN_EVERY_MS, checks = -WARN_EVERY_MS,
                      shared = -WARN_EVERY_MS, error = -WARN_EVERY_MS}

---------------------------------------------------------------------------
-- Helpers
---------------------------------------------------------------------------

local function now_ms()
    return millis():tofloat()
end

local function say(severity, text)
    gcs:send_text(severity, "VAMA lock: " .. text)
end

local function warn(kind, t_ms, text)
    if t_ms - last_warn_ms[kind] >= WARN_EVERY_MS then
        last_warn_ms[kind] = t_ms
        say(MAV_SEVERITY.WARNING, text)
    end
end

local function lock_enabled()
    return math.floor((VLCK_ENABLE:get() or 0) + 0.5) == 1
end

local function hold_ms()
    local hold = VLCK_HOLD:get() or 5
    return math.max(2, math.min(60, hold)) * 1000
end

-- True when every arming check is off: then ArduCopter runs only a few
-- mandatory checks, and this script's check is not one of them.
local function arming_checks_all_off()
    local check = param:get("ARMING_CHECK")          -- 4.6: a bitmask of the checks to run
    if check ~= nil then
        return math.floor(check + 0.5) == 0
    end
    local skip = param:get("ARMING_SKIPCHK")         -- 4.7: a bitmask of the checks to skip
    if skip ~= nil then
        local bits = math.floor(skip + 0.5)
        return (bits & ALL_CHECKS_47) == ALL_CHECKS_47
    end
    return false
end

-- The MAVLink checksum (CRC-16/MCRF4XX) of one more byte.
local function crc_add(crc, byte)
    local tmp = byte ~ (crc & 0xFF)
    tmp = (tmp ~ (tmp << 4)) & 0xFF
    return ((crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)) & 0xFFFF
end

-- A received heartbeat of a ground station: (signed, system id).
-- nil for anything else, and for a message whose checksum is wrong:
-- ArduPilot hands those to scripts too, without checking their signature.
local function read_gcs_heartbeat(msg)
    if #msg < MESSAGE_BYTES or string.unpack("<I3", msg, POS_MSGID) ~= MSG_HEARTBEAT then
        return nil
    end
    local magic = string.byte(msg, POS_MAGIC)
    local len = string.byte(msg, POS_LEN)
    local crc = 0xFFFF
    if magic == MAGIC_V2 then
        -- length, flags, seq, system, component, the 3 bytes of the message id, payload
        for i = POS_LEN, POS_PAYLOAD - 1 + len do
            crc = crc_add(crc, string.byte(msg, i))
        end
    elseif magic == MAGIC_V1 then
        -- length, seq, system, component, the message id in one byte, payload
        crc = crc_add(crc, len)
        for i = POS_SEQ, POS_SYSID + 1 do
            crc = crc_add(crc, string.byte(msg, i))
        end
        crc = crc_add(crc, MSG_HEARTBEAT)
        for i = POS_PAYLOAD, POS_PAYLOAD - 1 + len do
            crc = crc_add(crc, string.byte(msg, i))
        end
    else
        return nil
    end
    crc = crc_add(crc, HEARTBEAT_CRC_EXTRA)
    if crc ~= string.unpack("<H", msg, POS_CK) then
        return nil
    end
    -- The vehicle type is the 5th byte of the payload. A shorter payload ends in zeros.
    local mav_type = 0
    if len >= 5 then
        mav_type = string.byte(msg, POS_PAYLOAD + 4)
    end
    if mav_type ~= MAV_TYPE_GCS then
        return nil
    end
    local signed = magic == MAGIC_V2 and (string.byte(msg, POS_INCOMPAT) & IFLAG_SIGNED) ~= 0
    return signed, string.byte(msg, POS_SYSID)
end

local function set_state(new_state)
    if new_state == state then
        return
    end
    state = new_state
    if state == STATE_LOCKED then
        arming:set_aux_auth_failed(auth_id, "VAMA locked: no signed ground station")
        say(MAV_SEVERITY.WARNING, "locked, no signed ground station")
    elseif state == STATE_UNLOCKED then
        arming:set_aux_auth_passed(auth_id)
        say(MAV_SEVERITY.INFO, string.format("unlocked by GCS %d, channel %d", last_signed_sysid, last_signed_chan))
    else
        arming:set_aux_auth_passed(auth_id)
        say(MAV_SEVERITY.INFO, "off (VLCK_ENABLE 0)")
    end
end

---------------------------------------------------------------------------
-- The loop
---------------------------------------------------------------------------

local function update()
    local t_ms = now_ms()
    local enabled = lock_enabled()

    -- Every heartbeat that arrived since the last loop.
    for _ = 1, QUEUE_LENGTH do
        local msg, chan = mavlink:receive_chan()
        if msg == nil then
            break
        end
        local signed, sysid = read_gcs_heartbeat(msg)
        if signed ~= nil then
            if chan == USB_CHANNEL then
                if enabled and state == STATE_LOCKED then
                    warn("usb", t_ms, "USB does not unlock, use the radio link")
                end
            elseif signed then
                last_signed_ms = t_ms
                last_signed_sysid = sysid
                last_signed_chan = chan
            elseif enabled then
                -- With a key on the drone ArduPilot drops these before any script sees them.
                warn("unsigned", t_ms, string.format("unsigned GCS %d on channel %d, no key?", sysid, chan))
            end
        end
    end

    if enabled and not heartbeats_are_ours then
        warn("shared", t_ms, "another script reads heartbeats")
    end

    -- Armed or flying: never touched. The lock is only in the arming checks.
    if not arming:is_armed() then
        if not enabled then
            set_state(STATE_OFF)
        elseif last_signed_ms ~= nil and t_ms - last_signed_ms <= hold_ms() then
            set_state(STATE_UNLOCKED)
        else
            set_state(STATE_LOCKED)
        end
        if enabled and arming_checks_all_off() then
            warn("checks", t_ms, "arming checks are all off, no lock")
        end
    end

    if t_ms - last_status_ms >= STATUS_EVERY_MS then
        last_status_ms = t_ms
        gcs:send_named_float("VLCK", state or STATE_LOCKED)
    end
end

-- A mistake in this script must never leave the drone unlocked, and never
-- stop it silently.
local function protected_update()
    local ok, err = pcall(update)
    if ok then
        return protected_update, LOOP_MS
    end
    local still_on = true
    local read_ok, on = pcall(lock_enabled)
    if read_ok then
        still_on = on
    end
    if still_on then
        state = STATE_LOCKED
        arming:set_aux_auth_failed(auth_id, "VAMA locked: script error")
    else
        state = STATE_OFF
        arming:set_aux_auth_passed(auth_id)
    end
    local t_ms = now_ms()
    if t_ms - last_warn_ms.error >= WARN_EVERY_MS then
        last_warn_ms.error = t_ms
        -- "file.lua:123: what went wrong" becomes "line 123: what went wrong"
        local line, what = tostring(err):match(":(%d+): (.*)$")
        if line ~= nil then
            say(MAV_SEVERITY.ERROR, "error line " .. line .. ": " .. what)
        else
            say(MAV_SEVERITY.ERROR, "error " .. tostring(err))
        end
    end
    return protected_update, 1000
end

say(MAV_SEVERITY.INFO, "ready (" .. (lock_enabled() and "on" or "off") .. ")")

return protected_update()
