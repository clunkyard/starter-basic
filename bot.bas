' A robot on the Clunkyard core's FreeBASIC binding. The core owns tracking,
' geometry and protocol handling; this strategy chooses the tactics.
' Clunkyard robot API 2.0.0: one core under every language, the same
' answers everywhere. Facts (a call shows its arguments):
' match: mode ruleset control robots is_3d tick_rate max_ticks
'   robot_id(slot)
' clock: tick seconds remaining_ticks
' arena: size gravity terrain_kind ground_height(x y)
' me: health position velocity heading pitch roll angular_velocity
'   attitude grounded altitude wall_contact applied hull_mounts
'   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
' battery: charge capacity regen sustainable_drive compute
'   drive_cost(fraction)
' drive: hover_lift
' gun: aim_now cooldown_ticks spread max_power damage_scale
'   can_fire(power reserve) shot_speed(power) shot_cost(power)
'   reload_ticks(power) shot_damage(power) power_for_speed(speed)
'   power_for_flight_time(range seconds) lead(contact power)
'   lead_point(position velocity power)
' scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
' sensors: contacts nearest cues contact(slot) position_of(contact)
' tracker: cue nearest_bearing_rate velocity(slot) position(slot)
'   history(slot) last_seen(slot)
' nav: zone zone.margin(point) zone.contains(point margin)
'   zone.distance_from(point) storm storm.margin(point)
'   storm.contains(point margin) storm.distance_from(point) hill
'   hill.margin(point) hill.contains(point margin)
'   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
'   field_distance(a b)
' score: me best armour
' angles: normalize(degrees) angle_diff(target current)
'   bearing_to(from to) elevation_to(from to) direction(heading pitch)
'   distance(a b) to_chassis(heading pitch attitude)
'   from_chassis(heading pitch attitude) clamp(value low high)
' raw: number(path) text(path) has(path) flag(path) count(path) line()
' Controls: drive.set(throttle turn pitch_turn)
'   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
'   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
'   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
'   status.set(text value) debug.log(line) raw.extra(json)
' Every control argument is optional; a second call keeps what the first
' set. Absent facts are your language's none. Names are snake_case here;
' your language cases them its own way. Angles are degrees from the +x
' axis; pitch is positive up.
' Manual: https://clunkyard.com/manual#sdk
#include once "clunkyard.bi"
Dim Shared freeFlight As Boolean
Dim Shared As Long dodgeTicks, dodges, slot
' Bearing is earned: a scan contact's bearing is off by up to its bearing error
' (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
' one aims at it. Contacts within 120 units are exact.
Const SEARCH_ARC = 40  ' degrees: sweeps the circle in half a second
Const HOLD_WIDTH = 60  ' units across at the target: it stays in the beam from tick to tick
Const MIN_ARC = 6      ' degrees: the narrowest the beam holds
Const AVERAGE_TICKS = 12      ' ticks of sightings the aim averages
Const SIGHTINGS = 8    ' narrow sightings in the window before a shot
Const LOOK_EVERY = 40  ' ticks between looks around the target, for a nearer robot
Const REACQUIRE = 10   ' ticks to look again where a lost target was
Const PI = 3.14159265358979
Function Number(ByVal robot As Any Ptr, ByVal query As Function Cdecl(ByVal As Any Ptr, ByVal As Double Ptr) As Long) As Double
    Dim value As Double
    CyCheck(robot, query(robot, @value))
    Return value
End Function
Function Clamp(ByVal value As Double) As Double
    Dim result As Double
    CyCheck(0, cy_angles_clamp(value, -1, 1, @result))
    Return result
End Function
Function Steer(ByVal target As Double, ByVal current As Double, ByVal scale As Double = 45) As Double
    Dim delta As Double
    CyCheck(0, cy_angles_angle_diff(target, current, @delta))
    Return Clamp(delta / scale)
End Function
Sub Drive(ByVal robot As Any Ptr, ByVal throttle As Double, ByVal turn As Double, ByVal pitch As Double)
    CyCheck(robot, cy_drive_throttle(robot, throttle))
    CyCheck(robot, cy_drive_turn(robot, turn))
    CyCheck(robot, cy_drive_pitch_turn(robot, pitch))
End Sub
Sub Scan(ByVal robot As Any Ptr, ByVal heading As Double, ByVal arc As Double, ByVal pitch As Double)
    CyCheck(robot, cy_scanner_heading(robot, heading))
    CyCheck(robot, cy_scanner_arc(robot, arc))
    CyCheck(robot, cy_scanner_pitch(robot, pitch))
End Sub
Sub ShowStatus(ByVal robot As Any Ptr, ByRef text As Const String, ByVal value As Double = 0, ByVal withValue As Boolean = True)
    CyCheck(robot, cy_status_text(robot, StrPtr(text), Len(text)))
    If withValue Then CyCheck(robot, cy_status_value(robot, value))
End Sub
' A straight line through the narrowest of the last AVERAGE_TICKS ticks of sightings: where
' the robot is now and how fast it moves. Each sighting is off by its bearing error;
' the line averages that out. Returns the sightings it used, 0 for no fit.
Function Fit(ByVal robot As Any Ptr, ByVal target As Long, ByVal tick As Long, ByRef where As CyVec3, ByRef speed As CyVec3) As Long
    Dim history(0 To 31) As CySighting, total As ULong, rate As Long
    If cy_tracker_history(robot, target, @history(0), 32, @total) <> CY_OK Then Return 0
    CyCheck(robot, cy_match_tick_rate(robot, @rate))
    Dim best As Double = 1e30, seen(0 To 31) As Long, n As Long = 0
    For i As Integer = 0 To CInt(total) - 1
        If history(i).tick_value > tick - AVERAGE_TICKS AndAlso history(i).bearing_error_value < best Then best = history(i).bearing_error_value
    Next
    For i As Integer = 0 To CInt(total) - 1
        If history(i).tick_value > tick - AVERAGE_TICKS AndAlso history(i).bearing_error_value <= 1.5 * best Then seen(n) = i: n += 1
    Next
    If n < 2 Then Return 0
    Dim t0 As Double = 0, spread As Double = 0
    For k As Integer = 0 To n - 1: t0 += history(seen(k)).tick_value: Next
    t0 /= n
    For k As Integer = 0 To n - 1: spread += (history(seen(k)).tick_value - t0) ^ 2: Next
    Dim p(0 To 2) As Double, v(0 To 2) As Double
    For axis As Integer = 0 To 2
        Dim mean As Double = 0, slope As Double = 0
        For k As Integer = 0 To n - 1
            Dim s As CySighting = history(seen(k))
            mean += IIf(axis = 0, s.position_value.x_value, IIf(axis = 1, s.position_value.y_value, s.position_value.z_value))
        Next
        mean /= n
        For k As Integer = 0 To n - 1
            Dim s As CySighting = history(seen(k))
            slope += (s.tick_value - t0) * (IIf(axis = 0, s.position_value.x_value, IIf(axis = 1, s.position_value.y_value, s.position_value.z_value)) - mean)
        Next
        slope /= spread
        p(axis) = mean + slope * (tick - t0): v(axis) = slope * rate
    Next
    where.x_value = p(0): where.y_value = p(1): where.z_value = p(2)
    speed.x_value = v(0): speed.y_value = v(1): speed.z_value = v(2)
    Return n
End Function
Sub Start(ByVal robot As Any Ptr)
    Dim gravity As CyVec3, is3d As Long
    CyCheck(robot, cy_arena_gravity(robot, @gravity))
    CyCheck(robot, cy_match_is_3d(robot, @is3d))
    freeFlight = is3d <> 0 AndAlso gravity.z_value = 0
End Sub
Sub Turn(ByVal robot As Any Ptr)
    Dim position As CyVec3, target As CyContact, cue As CyCue, lastScan As CyScan, tick As Long, seen As Long
    CyCheck(robot, cy_me_position(robot, @position))
    CyCheck(robot, cy_scanner_last(robot, @lastScan))
    CyCheck(robot, cy_clock_tick(robot, @tick))
    Dim heading As Double = Number(robot, @cy_me_heading), pitch As Double = Number(robot, @cy_me_pitch)
    CyCheck(robot, cy_drive_brake(robot, 0.2))
    If cy_sensors_nearest(robot, @target) = CY_OK Then
        Dim throttle As Double = IIf(target.range_value > 350, 0.7, IIf(target.range_value < 200, -0.4, 0.2))
        Drive(robot, throttle, Steer(target.bearing_value, heading), Steer(target.elevation_value, pitch))
        slot = target.slot_value
        ' Halve the beam each tick until it is HOLD_WIDTH across at the target:
        ' narrow enough for a precise bearing, wide enough to keep it in.
        Dim hold As Double = 2 * Atan2(HOLD_WIDTH / 2, target.range_value) * 180 / PI
        If hold < MIN_ARC Then hold = MIN_ARC
        ' In a crowd a narrow beam sees one robot: every LOOK_EVERY ticks, widen it for a tick.
        Dim halved As Double = lastScan.arc_value / 2
        If halved > SEARCH_ARC Then halved = SEARCH_ARC
        Dim beam As Double = IIf(halved > hold, halved, hold)
        Scan(robot, target.bearing_value, IIf(tick Mod LOOK_EVERY = 0, SEARCH_ARC, beam), target.elevation_value)
        Dim power As Double, aim As CySolution, nowAim As CyDirection, delta As Double, canFire As Long
        Dim where As CyVec3, speed As CyVec3, used As Long = 0, ready As Boolean = True, led As Boolean
        CyCheck(robot, cy_gun_power_for_flight_time(robot, target.range_value, 1, @power))
        If target.bearing_error_value > 0 Then
            ' A noisy contact: lead the fitted line, and fire only once it rests on enough sightings.
            used = Fit(robot, slot, tick, where, speed)
            led = used > 0 AndAlso cy_gun_lead_point(robot, @where, @speed, power, @aim) = CY_OK
            ready = led AndAlso used >= SIGHTINGS
        Else
            ' An exact contact: cy_gun_lead is the intercept if the target holds course.
            led = cy_gun_lead(robot, @target, power, @aim) = CY_OK
        End If
        If Not led Then aim.heading_value = target.bearing_value: aim.pitch_value = target.elevation_value
        CyCheck(robot, cy_gun_aim_heading(robot, aim.heading_value))
        CyCheck(robot, cy_gun_aim_pitch(robot, aim.pitch_value))
        CyCheck(robot, cy_gun_aim_now(robot, @nowAim))
        CyCheck(robot, cy_angles_angle_diff(aim.heading_value, nowAim.heading_value, @delta))
        CyCheck(robot, cy_gun_can_fire(robot, power, 10, @canFire))
        If ready AndAlso Abs(delta) < 3 AndAlso Abs(aim.pitch_value - nowAim.pitch_value) < 3 AndAlso canFire <> 0 Then
            CyCheck(robot, cy_gun_fire(robot)): CyCheck(robot, cy_gun_power(robot, power))
        End If
        ShowStatus(robot, IIf(ready, "TRACK", "NARROW"), target.range_value)
    ElseIf slot <> 0 AndAlso cy_tracker_last_seen(robot, slot, @seen) = CY_OK AndAlso tick - seen <= REACQUIRE Then
        ' Lost it: look again where it was last seen, widening the beam, before searching.
        Dim where As CyVec3, bearing As Double, elevation As Double = 0, arc As Double = lastScan.arc_value * 2
        CyCheck(robot, cy_tracker_position(robot, slot, @where))
        CyCheck(robot, cy_angles_bearing_to(@position, @where, @bearing))
        If freeFlight Then CyCheck(robot, cy_angles_elevation_to(@position, @where, @elevation))
        If arc > SEARCH_ARC Then arc = SEARCH_ARC
        Drive(robot, 0.5, Steer(bearing, heading), Steer(elevation, pitch))
        Scan(robot, bearing, arc, elevation)
        ShowStatus(robot, "REACQUIRE", 0, False)
    ElseIf cy_tracker_cue(robot, @cue) = CY_OK Then
        Dim bearing As Double, distance As Double, elevation As Double = 0, arc As Double
        CyCheck(robot, cy_angles_bearing_to(@position, @cue.position_value, @bearing))
        CyCheck(robot, cy_angles_distance(@position, @cue.position_value, @distance))
        If freeFlight Then CyCheck(robot, cy_angles_elevation_to(@position, @cue.position_value, @elevation))
        Dim spread As Double = IIf(cue.bearing_error_value < 30, cue.bearing_error_value, 30)
        Dim across As Double = (tick Mod 5 - 2) / 2 * spread, up As Double = IIf(freeFlight, ((tick \ 5) Mod 3 - 1) * spread / 2, 0)
        Drive(robot, 0.7, Steer(bearing, heading), Steer(elevation, pitch))
        CyCheck(robot, cy_scanner_arc_for_reach(robot, distance + 100, @arc))
        Scan(robot, bearing + across, arc, elevation + up)
        ShowStatus(robot, "CUE", distance)
    Else
        Dim sweep As Double
        CyCheck(robot, cy_angles_normalize(lastScan.heading_value + 36, @sweep))
        Drive(robot, 0.5, 0.33, Steer(0, pitch))
        Scan(robot, sweep, 40, IIf(freeFlight, ((tick \ 10) Mod 3 - 1) * 35, 0))
        ShowStatus(robot, "SEARCH", 0, False)
    End If
    ' A simple dodge: vary it, or predict the shooter.
    Dim cues(0 To 31) As CyContact, heard As ULong, fired As Boolean = False
    If cy_sensors_cues(robot, @cues(0), 32, @heard) = CY_OK Then
        For i As Integer = 0 To CInt(heard) - 1
            If cues(i).source_value = CY_SOURCE_MUZZLE Then fired = True
        Next
    End If
    If dodgeTicks = 0 AndAlso fired Then dodges += 1: dodgeTicks = 10
    If dodgeTicks > 0 Then
        ' Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
        Dim leftward As Boolean = (dodges Mod 2 = 1), side As Double = IIf(leftward, 1.0, -1.0), control As Long
        CyCheck(robot, cy_match_control(robot, @control))
        dodgeTicks -= 1
        If control = CY_CONTROL_FORCES Then
            CyCheck(robot, cy_drive_throttle(robot, 0.5))
            CyCheck(robot, cy_drive_strafe(robot, -side))
        Else
            CyCheck(robot, cy_drive_throttle(robot, 1))
            CyCheck(robot, cy_drive_turn(robot, side))
        End If
        ShowStatus(robot, IIf(leftward, "dodge:left", "dodge:right"), dodges)
    End If
    If Number(robot, @cy_battery_charge) < Number(robot, @cy_battery_capacity) / 4 Then CyCheck(robot, cy_drive_throttle(robot, Number(robot, @cy_drive_get_throttle) * 0.5))
    Dim storm As CyStorm, zone As CyZone, margin As Double, bearing As Double, climb As Double
    If cy_nav_storm(robot, @storm) = CY_OK Then
        If cy_nav_storm_margin(robot, @position, @margin) = CY_OK AndAlso margin > -60 Then
            CyCheck(robot, cy_angles_bearing_to(@storm.center_value, @position, @bearing))
            CyCheck(robot, cy_drive_throttle(robot, 0.8)): CyCheck(robot, cy_drive_turn(robot, Steer(bearing, heading, 30)))
        End If
    End If
    If cy_nav_zone(robot, @zone) = CY_OK Then
        If cy_nav_zone_margin(robot, @position, @margin) = CY_OK AndAlso margin < 60 Then
            climb = 0
            If zone.shape_value = CY_SHAPE_SPHERE Then CyCheck(robot, cy_angles_elevation_to(@position, @zone.center_value, @climb))
            CyCheck(robot, cy_angles_bearing_to(@position, @zone.center_value, @bearing))
            Drive(robot, 0.8, Steer(bearing, heading, 30), Clamp((climb - pitch) / 30))
        End If
    End If
End Sub
CyRun(@Turn, @Start)
