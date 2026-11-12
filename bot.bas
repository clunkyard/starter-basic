' A robot on the Clunkyard core's FreeBASIC binding. The core owns tracking,
' geometry and protocol handling; this strategy chooses the tactics.
' Clunkyard robot API 2.0.0: one core under every language, the same
' answers everywhere. Facts (a call shows its arguments):
' match: mode ruleset control robots is_3d tick_rate max_ticks
'   robot_id(slot)
' clock: tick seconds remaining_ticks
' arena: size gravity terrain_kind ground_height(x y)
' me: health position velocity heading pitch roll angular_velocity
'   attitude grounded altitude applied
' battery: charge capacity regen sustainable_drive compute
'   drive_cost(fraction)
' drive: hover_lift
' gun: aim_now cooldown_ticks spread max_power damage_scale
'   can_fire(power reserve) shot_speed(power) shot_cost(power)
'   reload_ticks(power) shot_damage(power) power_for_speed(speed)
'   power_for_flight_time(range seconds) lead(contact power)
'   lead_point(position velocity power)
' scanner: last proximity_range reach(arc) arc_for_reach(range)
' sensors: contacts nearest cues contact(slot) position_of(contact)
' tracker: cue nearest_bearing_rate velocity(slot) position(slot)
'   last_seen(slot)
' nav: zone zone.margin(point) zone.contains(point margin)
'   zone.distance_from(point) storm storm.margin(point)
'   storm.contains(point margin) storm.distance_from(point) hill
'   hill.margin(point) hill.contains(point margin)
'   hill.distance_from(point) pickups nearest_pickup(kind)
'   field_distance(a b)
' score: me best armour
' angles: normalize(degrees) angle_diff(target current)
'   bearing_to(from to) elevation_to(from to) direction(heading pitch)
'   distance(a b) clamp(value low high)
' raw: number(path) text(path) has(path) flag(path) count(path) line()
' Controls: drive.set(throttle turn pitch_turn)
'   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
'   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
'   tracker.configure(smoothing forget_ticks cue_ticks)
'   status.set(text value) raw.extra(json)
' Every control argument is optional; a second call keeps what the first
' set. Absent facts are your language's none. Names are snake_case here;
' your language cases them its own way. Angles are degrees from the +x
' axis; pitch is positive up.
' Manual: https://clunkyard.com/manual#sdk
#include once "clunkyard.bi"
Dim Shared freeFlight As Boolean
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
Sub Start(ByVal robot As Any Ptr)
    Dim gravity As CyVec3, is3d As Long
    CyCheck(robot, cy_arena_gravity(robot, @gravity))
    CyCheck(robot, cy_match_is_3d(robot, @is3d))
    freeFlight = is3d <> 0 AndAlso gravity.z_value = 0
End Sub
Sub Turn(ByVal robot As Any Ptr)
    Dim position As CyVec3, target As CyContact, cue As CyCue, lastScan As CyScan, tick As Long
    CyCheck(robot, cy_me_position(robot, @position))
    CyCheck(robot, cy_scanner_last(robot, @lastScan))
    CyCheck(robot, cy_clock_tick(robot, @tick))
    Dim heading As Double = Number(robot, @cy_me_heading), pitch As Double = Number(robot, @cy_me_pitch)
    CyCheck(robot, cy_drive_brake(robot, 0.2))
    If cy_sensors_nearest(robot, @target) = CY_OK Then
        Dim throttle As Double = IIf(target.range_value > 350, 0.7, IIf(target.range_value < 200, -0.4, 0.2))
        Drive(robot, throttle, Steer(target.bearing_value, heading), Steer(target.elevation_value, pitch))
        Scan(robot, target.bearing_value, 20, target.elevation_value)
        Dim power As Double, aim As CySolution, nowAim As CyDirection, delta As Double, canFire As Long
        CyCheck(robot, cy_gun_power_for_flight_time(robot, target.range_value, 1, @power))
        If cy_gun_lead(robot, @target, power, @aim) <> CY_OK Then
            aim.heading_value = target.bearing_value: aim.pitch_value = target.elevation_value
        End If
        CyCheck(robot, cy_gun_aim_heading(robot, aim.heading_value))
        CyCheck(robot, cy_gun_aim_pitch(robot, aim.pitch_value))
        CyCheck(robot, cy_gun_aim_now(robot, @nowAim))
        CyCheck(robot, cy_angles_angle_diff(aim.heading_value, nowAim.heading_value, @delta))
        CyCheck(robot, cy_gun_can_fire(robot, power, 10, @canFire))
        If Abs(delta) < 3 AndAlso Abs(aim.pitch_value - nowAim.pitch_value) < 3 AndAlso canFire <> 0 Then
            CyCheck(robot, cy_gun_fire(robot)): CyCheck(robot, cy_gun_power(robot, power))
        End If
        ShowStatus(robot, "TRACK", target.range_value)
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
