--[[
   VAMA high wind failsafe for ArduCopter (4.6 and newer).

   It runs on the flight controller, so it works with any ground station and
   also when the radio link is lost. It watches two things while the autopilot
   is holding a position or flying a route:

   1. Motors close to their maximum for several seconds. The aircraft has no
      thrust left to fight wind, a weak battery or a failing motor.
   2. Pushed away: the aircraft leans hard against the wind and still moves
      away from the point the autopilot is steering to. The wind is stronger
      than it can fly against. ArduPilot limits the lean angle, so this
      happens with the motors far from their maximum.

   What it does then is set by WFS_ACTION: nothing, a warning only, RTL, or
   LAND. RTL needs a position. If the autopilot refuses RTL (GPS lost or
   jammed), it lands instead. If RTL itself is pushed away, it lands too.

   It acts once per flight. After that the pilot's mode changes are left alone.

   See vama_wind_failsafe.md for the setup, the parameters and the tests.
--]]

local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

local LOOP_MS = 200            -- 5 times a second
local MIN_FLYING_MS = 10000    -- ignore the first seconds after take-off
local MOTOR_TC_S = 1.0         -- smoothing of the motor output, seconds
local PUSH_TC_S = 1.5          -- smoothing of the pushed-away speed, seconds
local WARN_EVERY_MS = 60000    -- at most one warning text of a kind per minute
local REPORT_EVERY_MS = 1000   -- readings for the ground station in flight, once a second
local STATUS_EVERY_MS = 2000   -- "I am running, and this is my action", always
local SLOWING_MPS = 0.5        -- slower by this much than at the start means stopping, not pushed away
local LEAN_LIMIT_SHARE = 0.8   -- "leaning hard" is never asked to be more than this share of the lean limit
local TARGET_MIN_M = 0.5       -- closer than this to its target, the aircraft is holding it
local STILL_MPS = 2.0          -- slower than this over the ground, a lean angle is caused by wind
local RTL_GRACE_MS = 10000     -- RTL gets this long to stop the drift before it is judged

-- Copter flight modes
local MODE_AUTO, MODE_GUIDED, MODE_LOITER, MODE_RTL, MODE_CIRCLE, MODE_LAND = 3, 4, 5, 6, 7, 9
local MODE_POSHOLD, MODE_BRAKE, MODE_AVOID_ADSB, MODE_SMART_RTL = 16, 17, 19, 21
local MODE_FOLLOW, MODE_ZIGZAG, MODE_AUTO_RTL = 23, 24, 27

-- The autopilot holds a position or flies a route by itself in these modes.
local HOLD_MODES = {
    [MODE_AUTO] = true, [MODE_GUIDED] = true, [MODE_LOITER] = true, [MODE_CIRCLE] = true,
    [MODE_POSHOLD] = true, [MODE_BRAKE] = true, [MODE_AVOID_ADSB] = true,
    [MODE_FOLLOW] = true, [MODE_ZIGZAG] = true,
}
-- Already on the way home.
local RETURN_MODES = {[MODE_RTL] = true, [MODE_SMART_RTL] = true, [MODE_AUTO_RTL] = true}

local ACTION_OFF, ACTION_WARN, ACTION_RTL, ACTION_LAND = 0, 1, 2, 3

-- Output functions of motors 1 to 12
local MOTOR_FUNCTIONS = {33, 34, 35, 36, 37, 38, 39, 40, 82, 83, 84, 85}

---------------------------------------------------------------------------
-- Parameters
---------------------------------------------------------------------------

-- Must be a number that no other script on this flight controller uses (0 to 200).
local PARAM_TABLE_KEY = 196
local PARAM_TABLE_PREFIX = "WFS_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "Wind failsafe: could not add parameter " .. name)
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), "Wind failsafe: could not add the parameter table")

--[[
  // @Param: WFS_ACTION
  // @DisplayName: Wind failsafe action
  // @Description: What the wind failsafe does when it trips. RTL lands instead when RTL is refused or is pushed away too.
  // @Values: 0:Off,1:Warning only,2:RTL,3:Land
  // @User: Standard
--]]
local WFS_ACTION = bind_add_param("ACTION", 1, ACTION_WARN)

--[[
  // @Param: WFS_MOT_PCT
  // @DisplayName: Motor output limit
  // @Description: The failsafe trips when the highest motor output stays at or above this for WFS_TIME seconds. 0 turns this check off.
  // @Units: %
  // @Range: 0 100
  // @User: Standard
--]]
local WFS_MOT_PCT = bind_add_param("MOT_PCT", 2, 90)

--[[
  // @Param: WFS_TIME
  // @DisplayName: Time before the failsafe trips
  // @Description: How long a condition must last before the failsafe acts.
  // @Units: s
  // @Range: 2 30
  // @User: Standard
--]]
local WFS_TIME = bind_add_param("TIME", 3, 5)

--[[
  // @Param: WFS_WARN_PCT
  // @DisplayName: Motor output warning
  // @Description: A warning is sent when the highest motor output stays at or above this. It also counts as high effort for the pushed-away check. 0 turns the warning off.
  // @Units: %
  // @Range: 0 100
  // @User: Standard
--]]
local WFS_WARN_PCT = bind_add_param("WARN_PCT", 4, 80)

--[[
  // @Param: WFS_PUSH_SPD
  // @DisplayName: Pushed-away speed
  // @Description: The failsafe trips when the aircraft is pushed away from its target at this speed or more for WFS_TIME seconds, at high effort and without slowing down. 0 turns this check off.
  // @Units: m/s
  // @Range: 0 10
  // @User: Standard
--]]
local WFS_PUSH_SPD = bind_add_param("PUSH_SPD", 5, 1)

--[[
  // @Param: WFS_LEAN
  // @DisplayName: Lean angle that counts as high effort
  // @Description: A warning is sent when the aircraft leans this much to stay in place. For the pushed-away check it must lean at least this much, or have its motors at WFS_WARN_PCT. The script never asks for more than 80 percent of the aircraft's lean limit.
  // @Units: deg
  // @Range: 5 45
  // @User: Standard
--]]
local WFS_LEAN = bind_add_param("LEAN", 6, 20)

--[[
  // @Param: WFS_WSPD
  // @DisplayName: Wind speed warning
  // @Description: A warning is sent when the autopilot's wind estimate reaches this speed. It needs the EKF drag parameters. 0 turns the warning off.
  // @Units: m/s
  // @Range: 0 40
  // @User: Standard
--]]
local WFS_WSPD = bind_add_param("WSPD", 7, 0)

---------------------------------------------------------------------------
-- State
---------------------------------------------------------------------------

local flying_since_ms = nil
local returning_since_ms = nil -- when a return mode started
local tripped = false          -- acted in this flight
local return_judged = false    -- a return that was pushed away has been acted on
local motor_filtered = nil     -- highest motor output, smoothed, 0 to 1
local motor_high_ms = 0        -- how long it has been over the limit
local push_filtered = 0        -- pushed-away speed, smoothed, m/s
local push_ms = 0              -- how long the aircraft has been pushed away
local push_start_spd = 0       -- pushed-away speed when that started
local lean_still_ms = 0        -- how long it has leaned hard to stay in place
local last_warn_ms = {motor = -WARN_EVERY_MS, wind = -WARN_EVERY_MS, lean = -WARN_EVERY_MS}
local last_report_ms = 0
local last_status_ms = -STATUS_EVERY_MS
local last_error_ms = -WARN_EVERY_MS
local last_loop_ms = nil
local slow_params = nil        -- values read from the autopilot's own parameters
local slow_params_read_ms = -60000

---------------------------------------------------------------------------
-- Readings
---------------------------------------------------------------------------

local function now_ms()
    return millis():tofloat()
end

-- The autopilot parameters this script depends on. Read again every 5 s.
-- pwm_low and pwm_span: the PWM of 0 % and of 100 % motor output
-- (MOT_SPIN_MIN to MOT_SPIN_MAX). lean_limit: the largest lean angle, degrees.
local function read_slow_params(t_ms)
    if slow_params ~= nil and t_ms - slow_params_read_ms < 5000 then
        return slow_params
    end
    slow_params_read_ms = t_ms
    local pwm_min = param:get("MOT_PWM_MIN") or 1000
    local pwm_max = param:get("MOT_PWM_MAX") or 2000
    if pwm_min <= 0 or pwm_max <= pwm_min then
        pwm_min, pwm_max = 1000, 2000
    end
    local spin_min = param:get("MOT_SPIN_MIN") or 0.15
    local spin_max = param:get("MOT_SPIN_MAX") or 0.95
    if spin_max <= spin_min then
        spin_min, spin_max = 0.15, 0.95
    end
    -- 4.7 has ATC_ANGLE_MAX in degrees, 4.6 has ANGLE_MAX in centidegrees.
    local lean_limit = param:get("ATC_ANGLE_MAX")
    if lean_limit == nil then
        local centi = param:get("ANGLE_MAX")
        if centi ~= nil then
            lean_limit = centi / 100
        end
    end
    if lean_limit == nil or lean_limit < 5 or lean_limit > 80 then
        lean_limit = 30
    end
    -- The position controller can have its own, smaller limit.
    local psc_limit = param:get("PSC_ANGLE_MAX")
    if psc_limit ~= nil and psc_limit >= 5 and psc_limit < lean_limit then
        lean_limit = psc_limit
    end
    local width = pwm_max - pwm_min
    slow_params = {
        pwm_low = pwm_min + width * spin_min,
        pwm_span = width * (spin_max - spin_min),
        lean_limit = lean_limit,
    }
    return slow_params
end

-- The highest motor output right now, 0 to 1 of the usable range, or nil.
local function highest_motor_output(slow)
    local highest = nil
    for _, fn in ipairs(MOTOR_FUNCTIONS) do
        local pwm = SRV_Channels:get_output_pwm(fn)
        if pwm ~= nil then
            local out = (pwm - slow.pwm_low) / slow.pwm_span
            if highest == nil or out > highest then
                highest = out
            end
        end
    end
    if highest == nil then
        return nil
    end
    return math.max(0, math.min(1, highest))
end

-- Roll, pitch and yaw in radians. 4.7 has get_roll_rad(), 4.6 has get_roll().
local function attitude()
    if ahrs.get_roll_rad ~= nil then
        return ahrs:get_roll_rad(), ahrs:get_pitch_rad(), ahrs:get_yaw_rad()
    end
    return ahrs:get_roll(), ahrs:get_pitch(), ahrs:get_yaw()
end

-- How the aircraft moves, from its attitude, its velocity and the autopilot's target:
--   lean   lean angle, degrees
--   speed  speed over the ground, m/s
--   push   speed at which it is pushed away, m/s. Zero when it is not pushed.
-- Pushed means: the thrust points against the movement. When the flight mode
-- has a target (a hold point or a waypoint), the aircraft must also be off
-- that target and moving away from it. A fast leg with the wind from behind
-- leans back too, but it gets closer to its waypoint.
local function motion()
    local roll, pitch, yaw = attitude()
    local lean = math.deg(math.acos(math.max(-1, math.min(1, math.cos(roll) * math.cos(pitch)))))
    local vel = ahrs:get_velocity_NED()
    if vel == nil then
        return lean, 0, 0
    end
    local speed = math.sqrt(vel:x() * vel:x() + vel:y() * vel:y())
    -- Horizontal part of the thrust direction, north and east.
    local tn = -(math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll))
    local te = -(math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll))
    local len = math.sqrt(tn * tn + te * te)
    if len < 0.02 then
        return lean, speed, 0
    end
    local against = -(vel:x() * tn + vel:y() * te) / len
    if against <= 0 then
        return lean, speed, 0
    end
    -- The distance is zero in flight modes that have no target.
    local dist = vehicle:get_wp_distance_m() or 0
    local bearing = vehicle:get_wp_bearing_deg()
    if dist <= 0 or bearing == nil then
        return lean, speed, against
    end
    if dist < TARGET_MIN_M then
        return lean, speed, 0
    end
    local b = math.rad(bearing)
    local away = -(vel:x() * math.cos(b) + vel:y() * math.sin(b))
    return lean, speed, math.max(0, away)
end

---------------------------------------------------------------------------
-- Acting
---------------------------------------------------------------------------

-- Every text starts the same way, and stays within the 50 characters of one status text.
local function say(severity, text)
    gcs:send_text(severity, "Wind failsafe: " .. text)
end

local function warn(kind, t_ms, text)
    if t_ms - last_warn_ms[kind] >= WARN_EVERY_MS then
        last_warn_ms[kind] = t_ms
        say(MAV_SEVERITY.WARNING, text)
    end
end

local function land(reason)
    if vehicle:set_mode(MODE_LAND) then
        say(MAV_SEVERITY.CRITICAL, reason .. ", LAND")
    else
        say(MAV_SEVERITY.CRITICAL, "could not change the flight mode")
    end
end

-- The failsafe has tripped: do what WFS_ACTION says, once per flight.
local function trip(what, action)
    tripped = true
    if action == ACTION_LAND then
        land(what)
    elseif action == ACTION_RTL then
        if vehicle:set_mode(MODE_RTL) then
            say(MAV_SEVERITY.CRITICAL, what .. ", RTL")
        else
            -- RTL needs a position. Without one (GPS lost or jammed) the aircraft lands.
            land(what .. ", no RTL")
        end
    else
        say(MAV_SEVERITY.CRITICAL, what .. " (warning only)")
    end
end

---------------------------------------------------------------------------
-- Main loop
---------------------------------------------------------------------------

local function reset_flight()
    flying_since_ms = nil
    returning_since_ms = nil
    tripped = false
    return_judged = false
    motor_filtered = nil
    motor_high_ms = 0
    push_filtered = 0
    push_ms = 0
    lean_still_ms = 0
end

local function update()
    local t_ms = now_ms()
    local dt_ms = LOOP_MS
    if last_loop_ms ~= nil then
        dt_ms = math.max(1, math.min(1000, t_ms - last_loop_ms))
    end
    last_loop_ms = t_ms

    local action = math.floor((WFS_ACTION:get() or ACTION_WARN) + 0.5)
    -- Always, also on the ground: the ground station shows that the failsafe runs and what it is set to.
    if t_ms - last_status_ms >= STATUS_EVERY_MS then
        last_status_ms = t_ms
        gcs:send_named_float("WFS_ACT", action)
    end
    if action == ACTION_OFF or not arming:is_armed() or not vehicle:get_likely_flying() then
        reset_flight()
        return
    end
    if flying_since_ms == nil then
        flying_since_ms = t_ms
    end

    -- Readings
    local slow = read_slow_params(t_ms)
    local motor_now = highest_motor_output(slow)
    if motor_now ~= nil then
        if motor_filtered == nil then
            motor_filtered = motor_now
        else
            motor_filtered = motor_filtered + dt_ms / (MOTOR_TC_S * 1000 + dt_ms) * (motor_now - motor_filtered)
        end
    end
    local motor_pct = (motor_filtered or 0) * 100
    local lean, speed, push_now = motion()
    push_filtered = push_filtered + dt_ms / (PUSH_TC_S * 1000 + dt_ms) * (push_now - push_filtered)

    if t_ms - last_report_ms >= REPORT_EVERY_MS then
        last_report_ms = t_ms
        gcs:send_named_float("WFS_MOT", motor_pct)
        gcs:send_named_float("WFS_LEAN", lean)
        gcs:send_named_float("WFS_PUSH", push_filtered)
    end

    if t_ms - flying_since_ms < MIN_FLYING_MS then
        return
    end

    local mode = vehicle:get_mode()
    local landing = vehicle:is_landing()
    local holding = HOLD_MODES[mode] == true and not landing
    local returning = RETURN_MODES[mode] == true and not landing
    local time_ms = math.max(2, WFS_TIME:get() or 5) * 1000
    local mot_limit = WFS_MOT_PCT:get() or 0
    local warn_limit = WFS_WARN_PCT:get() or 0
    local push_limit = WFS_PUSH_SPD:get() or 0
    local lean_hard = math.min(WFS_LEAN:get() or 20, slow.lean_limit * LEAN_LIMIT_SHARE)

    -- Early warnings
    if (holding or returning) and speed < STILL_MPS and lean >= lean_hard then
        lean_still_ms = lean_still_ms + dt_ms
    else
        lean_still_ms = 0
    end
    if holding or returning then
        if warn_limit > 0 and motor_pct >= warn_limit then
            warn("motor", t_ms, string.format("motors at %.0f%%", motor_pct))
        end
        if lean_still_ms >= time_ms then
            warn("lean", t_ms, string.format("leaning %.0f deg to hold position", lean))
        end
        local wind_limit = WFS_WSPD:get() or 0
        if wind_limit > 0 then
            local wind = ahrs:wind_estimate()
            if wind ~= nil then
                local wind_speed = math.sqrt(wind:x() * wind:x() + wind:y() * wind:y())
                if wind_speed >= wind_limit then
                    warn("wind", t_ms, string.format("wind %.0f m/s", wind_speed))
                end
            end
        end
    end

    -- 1. Motors close to their maximum
    if holding and mot_limit > 0 and motor_pct >= mot_limit then
        motor_high_ms = motor_high_ms + dt_ms
    else
        motor_high_ms = 0
    end

    -- 2. Pushed away at high effort, and not slowing down
    local effort = lean >= lean_hard or (warn_limit > 0 and motor_pct >= warn_limit)
    if (holding or returning) and push_limit > 0 and push_filtered >= push_limit and effort then
        if push_ms == 0 then
            push_start_spd = push_filtered
        end
        push_ms = push_ms + dt_ms
        -- Stopping after a fast leg can look the same, but it gets slower. Start again then.
        if push_filtered < push_start_spd - SLOWING_MPS then
            push_ms = 0
        end
    else
        push_ms = 0
    end

    if returning then
        if returning_since_ms == nil then
            returning_since_ms = t_ms
        end
        if t_ms - returning_since_ms < RTL_GRACE_MS then
            -- RTL has just started. Give it time to stop the drift.
            push_ms = 0
        elseif not return_judged and push_ms >= time_ms then
            -- On the way home and still pushed away: the aircraft cannot get there. Land.
            return_judged = true
            tripped = true
            local what = "RTL is pushed back"
            if action == ACTION_WARN then
                say(MAV_SEVERITY.CRITICAL, what .. " (warning only)")
            else
                land(what)
            end
        end
        return
    end
    returning_since_ms = nil

    if tripped or not holding then
        return
    end

    if motor_high_ms >= time_ms then
        trip(string.format("motors %.0f%% for %.0fs", motor_pct, motor_high_ms / 1000), action)
    elseif push_ms >= time_ms then
        trip(string.format("pushed back %.0f m/s", push_filtered), action)
    end
end

-- A mistake in this script must never stop it silently.
local function protected_update()
    local ok, err = pcall(update)
    if not ok then
        local t_ms = now_ms()
        if t_ms - last_error_ms >= WARN_EVERY_MS then
            last_error_ms = t_ms
            -- "file.lua:123: what went wrong" becomes "line 123: what went wrong"
            local line, what = tostring(err):match(":(%d+): (.*)$")
            if line ~= nil then
                say(MAV_SEVERITY.ERROR, "error line " .. line .. ": " .. what)
            else
                say(MAV_SEVERITY.ERROR, "error " .. tostring(err))
            end
        end
        return protected_update, 2000
    end
    return protected_update, LOOP_MS
end

local ACTION_NAMES = {[ACTION_OFF] = "off", [ACTION_WARN] = "warning only", [ACTION_RTL] = "RTL", [ACTION_LAND] = "LAND"}
local start_action = math.floor((WFS_ACTION:get() or ACTION_WARN) + 0.5)
say(MAV_SEVERITY.INFO, "ready (" .. (ACTION_NAMES[start_action] or ("action " .. start_action)) .. ")")

return protected_update, 2000
