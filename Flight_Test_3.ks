// Initialize
wait until ship:unpacked.
ship:partstitled("RC-001S Remote Guidance Unit")[0]:controlfrom().
clearscreen.
Print "Beginning Flight Test 3".
switch to 0.
// Control steering and thrust
sas off.
rcs off.
lock throttle to 0.
lock steering to heading(90,90).
stage.  // Start booster engines

//
//   Variables
//

// Flight parameters
set tgt_altitude to 80000.  // Target orbital altitude in m
set tgt_twr to 1.1.  // Initial takeoff TWR
set hover_alt to 15000.  // Altitude in m at which the hoverslam will begin
set ref_area to 11.262.  // Cross-sectional reference area in m^2

set launchpad to ship:geoposition.  // Geocoordinates structure to help navigation back to launchpad
set equatorial_normal to v(0,1,0).  // Unit vector normal to the equatorial plane and pointing towards the north pole
set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s, calculated when booster_engines list is created. Note: value is always negative

// Lists and lexicons

// Create a list of booster_engines and initialize associated variables
set booster_engines to list().  // List of booster engines
// Populate booster_engines list and measure max mass outflow rate
set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
        set max_mass_outflow to max_mass_outflow - eng:maxmassflow * 1000.
    }
}
set sl_isp to booster_engines[0]:slisp.  // Sea level isp for the booster in s
set v_isp to booster_engines[0]:visp.  // Vacuum isp for the booster in s
// Lexicons of temperature curves and modifiers for Kerbin based on game data
set temperatureCurve to lexicon(0,288.15,8815.22,216.65,16050.39,216.65,25729.23,228.65,37879.44,270.65,41129.24,270.65,57440.13,214.65,68797.88,186.946,70000,186.946).  // Keyed by altitude
set temperatureSunMultCurve to lexicon(0,1,8815.22,0.3,16050.39,0,25729.23,0,37879.44,0.2,57440.13,0.2,63902.72,1,70000,1.2).  // Keyed by altitude
set temperatureLatitudeBiasCurve to lexicon(0,17,10,12,18,6.36371,30,0,35,-10,45,-23,55,-31,70,-37,90,-50).  // Keyed by latitude
set temperatureLatitudeSunMultCurve to lexicon(0,9,40,14.2,55,14.9,68,12.16518,76,8.582909,90,5).  // Keyed by latitude
// Read mach_cd.json and reverse the order of keys
set cd_lex to lexicon().
set avg_cd to 0.  // Average of all drag coefficient entries
set read_lex to readJson("mach_cd.json").
set read_index to read_lex:length - 1.
until read_index = -1 {
    set cd_lex[read_lex:keys[read_index]] to read_lex:values[read_index].  // List of drag coefficients keyed by mach number
    set avg_cd to avg_cd + read_lex:values[read_index].
    set read_index to read_index - 1.
}
set avg_cd to avg_cd / read_lex:length.
// Butcher Tableu for RK-45


//
//   Functions
//

function fineTuneOrbit {
    // Zeros the inclination and eccentricity of the orbit

    local lock orbital_vel to ship:obt:velocity:orbit.
    local lock tgt_vel to sqrt(kerbin:mu/kerbin:position:mag) * heading(90,0):forevector.
    local lock burn_vec to tgt_vel - orbital_vel.

    if vdot(kerbin:position, equatorial_normal) > 0 {
        lock steering to heading(180,0).
    } else {
        lock steering to heading(0,0).
    }

    // Wait to adjust attitude until close to node
    clearscreen.
    until abs(vdot(kerbin:position, equatorial_normal)) / (kerbin:position:mag * sin(ship:obt:inclination)) < 0.3 {
        print "vdot: " + round(abs(vdot(kerbin:position, equatorial_normal)) / (kerbin:position:mag * sin(ship:obt:inclination)),2) at(0,1).
        print "lat: " + round(abs(90-vang(equatorial_normal, kerbin:position)),2) at(0,2).
        wait 1.
    }
    
    // Burn
    rcs on.
    lock steering to burn_vec.
    until abs(90-vang(equatorial_normal, kerbin:position)) < 0.0001 and vang(ship:facing:forevector, burn_vec) < 0.1 {
        print "lat: " + round(abs(90-vang(equatorial_normal, kerbin:position)),3) at(0,2).
    }
    lock throttle to 1.
    //wait timeToBurn(burn_vec:mag, v_isp, ship:mass * 1000, 1).
    wait until burn_vec:mag < 1.

    lock throttle to 0.
    rcs off.
}

function projectOntoEquatorialPlane {
    // Projects the input vector onto the equatorial plane of Kerbin
    parameter input_vec.

    return input_vec - vdot(input_vec, equatorial_normal) * equatorial_normal.
}

function getAtmTemperature {
    // Returns an estimated temperature based on given position vectors for Kerbin and Kerbol
    parameter kerbin_pos.  // Kerbin position vector in m
    parameter sun_pos.  // Kerbol position vector in m from the ship

    // Calculate altitude
    local atm_alt to kerbin_pos:mag - kerbin:radius.

    // Calculate latitude
    // Latitude = abs(90-angle between kerbin_pos and the equatorial normal)
    local lat to abs(90-vang(equatorial_normal, kerbin_pos)).

    // Calculate sunDotNormalized
    // sunDotNormalized = 0.5 * cos(hour_angle-45) + 0.5. 
    local kerbin_sun_pos to kerbin_pos - sun_pos.  // Kerbin position vector from Kerbol
    local proj_kerbin_pos to projectOntoEquatorialPlane(kerbin_pos).  // The projection of kerbin position vector onto the equatorial plane.
    local ref_vector to vcrs(kerbin_sun_pos, equatorial_normal):normalized.  // A unit vector in the equatorial plane which serves as a reference for positive and negative hour angles, if the dot product with kerbin position vector is negative then the hour angle is negative
    local hour_angle to vang(kerbin_sun_pos, proj_kerbin_pos).
    if vdot(proj_kerbin_pos, ref_vector) < 0 {
        set hour_angle to -1 * hour_angle.
    }
    local sunDotNormalized to 0.5 * cos(hour_angle - 45) + 0.5.

    // Calculate base temperature based on atmospheric height
    local base_temp to interpolate(temperatureCurve, atm_alt).

    // Calculate temperature modifier
    // Modifier = temperatureSunMultCurve * [ temperatureLatitudeBiasCurve + temperatureLatitudeSunMultCurve * sunDotNormalized]
    set temp_mod to interpolate(temperatureSunMultCurve, atm_alt) * ( interpolate(temperatureLatitudeBiasCurve, lat) + interpolate(temperatureLatitudeSunMultCurve, lat) * sunDotNormalized).

    return base_temp + temp_mod.
}

function interpolate {
    // Performs linear interpolation on a lexicon to return a value based on a lookup value
    parameter lex_arg.  // Lexicon to evaluate
    parameter lookup_value.  // Value to lookup (x)

    local i to lex_arg:length - 1.
    // Search the lexicon keys in reverse until a key is less than the lookup value
    until lex_arg:keys[i] < lookup_value {
        set i to i - 1.
    }
    local lo_key to lex_arg:keys[i].  // Key of the low endpoint (x1)
    local hi_key to lex_arg:keys[i+1].  // Key of the high endpoint (x2)
    local lo_endpoint to lex_arg[lo_key].  // Value of the low endpoint (y1)
    local hi_endpoint to lex_arg[hi_key].  // Value of the high endpoint (y2)

    // y = (y2 - y1) * (x - x1) / (x2 - x1) + y1
    return (hi_endpoint - lo_endpoint) * (lookup_value - lo_key) / (hi_key - lo_key) + lo_endpoint.
}

function F_gravity {
    // Returns the gravity force vector in N from a given mass and position vector using Newton's law of gravity
    // F_gravity = mu * m/r^2
    parameter position_arg.  // Kerbin position vector in m
    parameter mass_arg.  // ship mass in kg

    return kerbin:mu * mass_arg / position_arg:mag^2 * position_arg:normalized.
}

function F_drag {
    // Returns the drag force vector (for descent) in N from a given velocity and position using the drag equation
    // F_drag = 0.5 * rho * v^2 * c_drag * A
    // Density (rho) = pressure / (287.053 * temperature)
    parameter position_arg.  // Kerbin position vector in m
    parameter velocity_arg.  // Ship surface velocity vector in m/s
    parameter sun_pos.  // Kerbol position vector in m

    local ship_alt to position_arg:mag - kerbin:radius.
    if ship_alt > 70000 {
        return 0 * velocity_arg.  // Returns zero vector if ship is outside atmosphere
    }
    local atm_temp to getAtmTemperature(position_arg, sun_pos).

    // Calculate atmospheric density
    local density to kerbin:atm:altitudepressure(ship_alt) * constant:atmtokpa * 1000 / (287.053 * atm_temp).  // Density in kg/m^3
    
    // Calculate surface velocity and velocity relative to air
    // srf_vel = r * omega * cos(latitude)
    local position_proj to projectOntoEquatorialPlane(position_arg).  // The projection of position_arg vector onto the equatorial plane.
    local srf_dir to -1 * vcrs(position_proj, equatorial_normal):normalized.  // The unit vector pointing in the direction of Kerbin's rotation
    local lat to abs(90-vang(equatorial_normal, position_arg)).  // Latitude
    local srf_vel to kerbin:radius * kerbin:angularvel:mag * cos(lat) * srf_dir.
    local relative_vel to velocity_arg - srf_vel.

    // Calculate mach number and interpolate drag coefficient
    local c_drag to avg_cd.
    local mach to relative_vel:mag / sqrt(1.4*287.053*atm_temp).
    if cd_lex:haskey(round(mach, 1)) {
        set c_drag to cd_lex[round(mach, 1)].
    }

    return 0.5 * density * relative_vel:mag^2 * c_drag * ref_area * -1 * relative_vel:normalized.
}

function countdownTimer {
    // Performs a countdown. Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
    parameter countdown.  // Countdown timer in s

    until countdown = 0 {
        print "T - " + countdown + " " at (0,1).
        set countdown to countdown - 1.
        wait 1.
    }
}

function massAfterBurn {
    // Returns the ship mass in kg after a given burn using Tsiolkovsky's rockey equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    parameter deltav.  // Delta-v in m/s
    parameter isp.  // Specific impulse in s
    parameter m_initial.  // Initial mass in kg

    return m_initial * constant:e^(-1 * deltav / (isp * 9.81)).
}

function timeToBurn {
    // Returns the burn time in s for a given delta-v, isp, and initial mass
    // Burn time = (m_final - m_initial) / mass outflow
    parameter deltav.  // Delta-v in m/s
    parameter isp.  // Specific impulse in s
    parameter m_initial.  // Initial mass in kg
    parameter throttle_arg.  // Throttle %

    local m_final to massAfterBurn(deltav, isp, m_initial).
    return (m_final - m_initial) / (max_mass_outflow * throttle_arg).
}

function gravityTurn {
    // Performs a gravity turn during ascent then cuts engine and coasts until ship has left the atmosphere
    parameter initial_pitch.  // Inital pitch from the vertical in degrees

    // Pitch after tower is cleared
    wait until ship:altitude > 200.
    clearscreen.
    print "Begin roll program".
    lock steering to heading(90,90-initial_pitch).
    wait until vang(ship:srfprograde:forevector, heading(90,90-initial_pitch):forevector) < 0.5.
    lock steering to ship:srfprograde.

    // Dynamically adjusts throttle
    local lock theta to vang(heading(90,0):forevector, ship:srfprograde:forevector).
    local lock tgt_theta to 88 * constant:e^(-0.05723 * ship:altitude / 1000).
    lock throttle to min(1, max(0.05,-1/10 * (theta-tgt_theta) + 1)).
    wait until ship:obt:apoapsis > tgt_altitude.
    lock throttle to 0.
    rcs on.
    wait until ship:altitude > 70000.

    // Release control
    unlock steering.
    rcs off.
    clearscreen.
    print "Coasting to apoapsis.".
}

function executeBurn {
    // Executes the given maneuver node
    parameter burn_node.  // Manuever node
    parameter throttle_arg.  // Throttle % for the burn

    // Take control
    rcs on.
    lock steering to burn_node:deltav.

    // Get burn time
    local burn_time to timeToBurn(burn_node:deltav:mag, v_isp, ship:mass * 1000, throttle_arg).
    // Execute burn
    wait until nextnode:eta < (burn_time / 2).
    lock steering to ship:facing:forevector.
    lock throttle to throttle_arg.
    wait burn_time.
    lock throttle to 0.

    // Release control
    rcs off.
    unlock steering.
    remove burn_node.
}

function orbitalInsertion {
    // Calculates and executes the maneuver for insertion into a roughly circular orbit at apoapsis

    // Calculate velocity at apoapsis using conservation of energy
    // v_final = sqrt(v_inital^2 + 2*kerbin:mu*(1/r_initial - 1/r_final))
    local apoapsis_vel to sqrt(ship:velocity:orbit:mag^2 + 2*kerbin:mu*(1/(kerbin:radius+ship:obt:apoapsis) - 1/(kerbin:radius+ship:altitude))).
    // Calculate orbital velocity at ship apoapsis
    // v = sqrt(MU/r)
    local tgt_obt_vel to sqrt(kerbin:mu / (kerbin:radius + ship:obt:apoapsis)).

    // Create a maneuver node at apoapsis with nesecary delta v in the prograde direction
    local obt_insertion_burn to node(timespan(ship:obt:eta:apoapsis), 0, 0, tgt_obt_vel-apoapsis_vel).
    add obt_insertion_burn.
    executeBurn(obt_insertion_burn, 1).
}

function deployPayload {
    // Deploys the booster's payload

    // Take control
    rcs on.
    lock steering to north.
    wait until vang(ship:facing:forevector, north:forevector) < 1.
    stage.  // Deploy payload decoupler
    clearscreen.
    print "Payload deployed.".

    // Release control
    unlock steering.
    rcs off.
}

function integrateReturnTrajectory{
    // Uses RK45 to model the return trajectory to the surface then executes a precise deorbiting burn

    clearscreen.
    print "Calculate return trajectory...".

    // Differential equations
    local function drdt {
        // dr/dt = -v
        parameter velocity_arg.  // Velocity vector in m/s

        return -1 * velocity_arg.
    }

    local function dvdt {
        // dv/dt = (F_gravity + F_drag) / m
        parameter position_arg.  // Position vector in m
        parameter velocity_arg.  // Velocity vector in m/s

        return (F_gravity(position_arg, mass_final) + F_drag(position_arg, velocity_arg, sun:position)) / mass_final.
    }

    // Calculate decaying orbit
    local deorbit_semimajoraxis to (kerbin:position:mag + kerbin:radius + 52000) / 2.
    local deorbit_velocity to sqrt(kerbin:mu * ( 2 / kerbin:position:mag - 1/deorbit_semimajoraxis)).
    local deltav to ship:obt:velocity:orbit:mag - deorbit_velocity.

    // Initial conditions
    global pos_list to list().
    global vel_list to list().
    pos_list:add(kerbin:position).
    vel_list:add(deorbit_velocity * ship:prograde:forevector).
    local mass_final to massAfterBurn(deltav, v_isp, ship:mass * 1000).

    // Parameters
    local n to 0.  // Index
    local step to 1.  // Initial step size
    local tolerance to 1000.  // Tolerance

       // RK-45
    until pos_list[n]:mag - kerbin:radius < hover_alt {
        // drdt = f(vel)
        // dvdt = f(pos, vel)
        print "Loop #: " + (n+1) + "   " at(0,1).
        print "Step size: " + step + "                " at(0,2).
        print "Expected Altitude: " + round(pos_list[n]:mag - kerbin:radius) + "  " at(0,3).
        print "Expected Velocity: " + round(vel_list[n]:mag, 1) + "   " at(0,4).

        local pos_k1 to drdt(vel_list[n]).
        local vel_k1 to dvdt(pos_list[n], vel_list[n]).

        local pos2 to pos_list[n] + step * pos_k1 * 1/5.
        local vel2 to vel_list[n] + step * vel_k1 * 1/5.
        local pos_k2 to drdt(vel2).
        local vel_k2 to dvdt(pos2, vel2).

        local pos3 to pos_list[n] + step * (3/40 * pos_k1 + 9/40 * pos_k2).
        local vel3 to vel_list[n] + step * (3/40 * vel_k1 + 9/40 * vel_k2).
        local pos_k3 to drdt(vel3).
        local vel_k3 to dvdt(pos3, vel3).
        
        local pos4 to pos_list[n] + step * (44/45 * pos_k1 - 56/15 * pos_k2 + 32/9 * pos_k3).
        local vel4 to vel_list[n] + step * (44/45 * vel_k1 - 56/15 * vel_k2 + 32/9 * vel_k3).
        local pos_k4 to drdt(vel4).
        local vel_k4 to dvdt(pos4, vel4).
        
        local pos5 to pos_list[n] + step * (19372/6561 * pos_k1 - 25360/2187 * pos_k2 + 64448/6561 * pos_k3 - 212/729 * pos_k4).
        local vel5 to vel_list[n] + step * (19372/6561 * vel_k1 - 25360/2187 * vel_k2 + 64448/6561 * vel_k3 - 212/729 * vel_k4).
        local pos_k5 to drdt(vel5).
        local vel_k5 to dvdt(pos5, vel5).        

        local pos6 to pos_list[n] + step * (9017/3168 * pos_k1 - 355/33 * pos_k2 + 46732/5247 * pos_k3 + 49/176 * pos_k4 - 5103/18656 * pos_k5).
        local vel6 to vel_list[n] + step * (9017/3168 * vel_k1 - 355/33 * vel_k2 + 46732/5247 * vel_k3 + 49/176 * vel_k4 - 5103/18656 * vel_k5).
        local pos_k6 to drdt(vel6).
        local vel_k6 to dvdt(pos6, vel6).

        local pos7 to pos_list[n] + step * (35/384 * pos_k1 + 500/1113 * pos_k3 + 125/192 * pos_k4 - 2187/6784 * pos_k5 + 11/84 * pos_k6).
        local vel7 to vel_list[n] + step * (35/384 * vel_k1 + 500/1113 * vel_k3 + 125/192 * vel_k4 - 2187/6784 * vel_k5 + 11/84 * vel_k6).
        local pos_k7 to drdt(vel7).

        // Check error
        local pos_rk4_estimate to pos_list[n] + step * (5179/57600 * pos_k1 + 7571/16695 * pos_k3 + 393/640 * pos_k4 - 92097/339200 * pos_k5 + 187/2100 * pos_k6 + 1/40 * pos_k7).
        local error to (pos7 - pos_rk4_estimate):mag.
        if error < tolerance {
            pos_list:add(pos7).
            vel_list:add(vel7).
            set n to n+1.
        }
        if not error = 0. {
            set step to 0.9 * step * (tolerance/error)^(1/5).
        }
    }
}

function calculateReturnTrajectory {
    // Calculates the return trajectory and executes a precise deorbiting burn
    // Uses the 4th order Runge-Kutta method to model the return trajectory for the booster

    // Use the vis-viva equation to determine the deltav of a deorbiting burn targeting a periapsis of 50km
    // v^2 = mu * (2/r - 1/a)
    // a = (2*body radius + apoapsis + periapsis)/2
    local deorbit_semimajor_axis to (2 * kerbin:radius + ship:obt:semimajoraxis - kerbin:radius + 52000) / 2.
    local deorbit_vel to sqrt(kerbin:mu * (2/kerbin:position:mag - 1/deorbit_semimajor_axis)).  // The velocity required to deorbit
    local deltav to ship:obt:velocity:orbit:mag - deorbit_vel.  // The delta-v for a deorbiting burn

    clearscreen.
    print "Calculating return trajectory...".
    print "Deorbit burn paramaters:" at(0,1).
    print "Semimajor axis: " + deorbit_semimajor_axis at(0,2).
    print "Deorbit velocity: " + deorbit_vel at (0,3).
    print "Delta-V: " + deltav at (0,4).
    
    // Parameters
    local n to 0.  // Index
    local t to 0.  // Time in s
    local step to 0.5.  // KSP tries to do a physics update 50 times a second.

    // Inital conditions
    local m_final to massAfterBurn(deltav, v_isp, ship:mass * 1000).
    global pos_list to list().
    global vel_list to list().
    pos_list:add(kerbin:position).
    vel_list:add(deorbit_vel * ship:prograde:forevector).

    // Differential equation setup
    local function drdt {
        // dr/dt = -v
        parameter velocity_arg.  // Velocity vector in m/s

        return -1 * velocity_arg.
    }

    local function dvdt {
        // dv/dt = 1/m (F_gravity + F_drag)
        parameter position_arg.  // Position vector in m
        parameter velocity_arg.  // Velocity vector in m/s

        return 1/m_final * ( F_gravity(position_arg, m_final) + F_drag(position_arg, velocity_arg, sun:position)).
    }

    // 4th order Runge-Kutta method
    until pos_list[n]:mag - kerbin:radius < hover_alt {
        // Readout
        print "Loop #: " + (n+1) at(0,6).
        print "Alt: " + (pos_list[n]:mag - kerbin:radius) + " " at(0,7).
        print "Vel mag: " + vel_list[n]:mag + " " at(0,8).
        print "Drag: " + F_drag(pos_list[n], vel_list[n], sun:position):mag at (0,9).

        // drdt = f(vel)
        // dvdt = f(pos, vel)

        local pos_k1 to drdt(vel_list[n]).
        local vel_k1 to dvdt(pos_list[n], vel_list[n]).

        local pos_k2 to drdt(vel_list[n]+vel_k1*step/2).
        local vel_k2 to dvdt(pos_list[n]+pos_k1*step/2, vel_list[n]+vel_k1*step/2).

        local pos_k3 to drdt(vel_list[n]+vel_k2*step/2).
        local vel_k3 to dvdt(pos_list[n]+pos_k2*step/2, vel_list[n]+vel_k2*step/2).

        local pos_k4 to drdt(vel_list[n]+vel_k3*step).
        local vel_k4 to dvdt(pos_list[n]+pos_k3*step, vel_list[n]+vel_k3*step).

        // Iterate
        pos_list:add(pos_list[n] + step/6 * (pos_k1 + 2*pos_k2 + 2*pos_k3 + pos_k4)).
        vel_list:add(vel_list[n] + step/6 * (vel_k1 + 2*vel_k2 + 2*vel_k3 + vel_k4)).
        set t to t+step.
        set n to n+1.
    }

    local ship_theta to vang(projectOntoEquatorialPlane(pos_list[0]), projectOntoEquatorialPlane(pos_list[n])).
    local kerbin_theta to t * kerbin:angularvel:mag * 180 / constant:pi.
    // Readout
    clearscreen.
    print "Tracjetory set!" at(0,0).
    print "Ship rot angle: " + ship_theta at(0,1).
    print "Kerbin rot angle: " + kerbin_theta at(0,2).

    rcs on.
    lock steering to ship:retrograde.

    wait until abs(ship:longitude - (launchpad:lng + ship_theta + kerbin_theta)) < 0.01.

    local wait_time to timeToBurn(deltav, v_isp, ship:mass * 1000, 1).
    lock throttle to 1.
    wait wait_time.
    lock throttle to 0.
}

function deorbit {
    // Use the vis-viva equation to determine the deltav of a deorbiting burn targeting a periapsis of 50km
    // v^2 = mu * (2/r - 1/a)
    // a = (2*body radius + apoapsis + periapsis)/2
    local deorbit_semimajor_axis to (2 * kerbin:radius + ship:obt:semimajoraxis - kerbin:radius + 52000) / 2.
    local deorbit_vel to sqrt(kerbin:mu * (2/kerbin:position:mag - 1/deorbit_semimajor_axis)).  // The velocity required to deorbit
    local deltav to ship:obt:velocity:orbit:mag - deorbit_vel.  // The delta-v for a deorbiting burn

    wait 10.
    // Create a maneuver node and execute
    local deorbit_burn to node(timespan(0,0,0,4,0), 0, 0, -1*deltav).
    add deorbit_burn.
    executeBurn(deorbit_burn, 1).
}

function landingBurn {
    // Controls attitude during atmospheric rentry and performs a hoverslam on the launchpad

    // Take control
    rcs on.
    brakes on.
    lock steering to ship:srfretrograde.

    // Calculate time until hoverslam altitude using simple projectile motion, solves for t with the quadratic formula
    // Note: neglecting drag gives a built in buffer since actual acceleration will be slower than this formula predicts
    // 0 = y0 - vertical_speed*t - g/2*t^2
    local lock impact_time to (-1*ship:verticalspeed - sqrt((ship:verticalspeed)^2 + 2*9.81*(ship:altitude))) / -9.81.

    // Calculate the burn time to negate air speed.
    local lock burn_time to timeToBurn(ship:airspeed, sl_isp, ship:mass*1000, 1).

    //// Slow down at the given altitude
    clearscreen.
    print "Reentry Program".
    local t to 0.
    until impact_time < burn_time {
        if ship:altitude < 70000 {
            rcs off.
        }
        print "T: " + round(t,1) + "   " at(0,1).
        print "Expected Alt: " + round(pos_list[t*2]:mag-kerbin:radius,1) + "   " at(0,2).
        print "Expected Vel: " + round(vel_list[t*2]:mag,1) + "   " at(0,3).
        set t to t + 1.
        wait 1.
    }
    lock steering to ship:srfretrograde.
    //// Locks throttle to target velocity of 20 m/s
    lock throttle to min(1, max(0, ((ship:airspeed - 20) / 4.2 + 1) * F_gravity(kerbin:position, ship:mass * 1000):mag / (ship:maxthrust * 1000))).
    wait until ship:airspeed < 25.
    lock steering to up.
    wait until ship:altitude - ship:geoposition:terrainheight < 50.
    //// Locks throttle to target velocity of 5 m/s
    lock throttle to min(1, max(0, ((ship:airspeed - 5) / 0.83 + 1) * F_gravity(kerbin:position, ship:mass * 1000):mag / (ship:maxthrust * 1000))).
    //lock steering to up.
    wait until ship:altitude - ship:geoposition:terrainheight < 15.
    lock throttle to 0.
    wait 1.
    clearscreen.
    print "Landed!".
}

lock twr_throttle to min(1, max(0, tgt_twr * F_gravity(kerbin:position, ship:mass * 1000):mag / (ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)) * 1000))).  // A throttle ratio [0,1] that provides target TWR

//
//   Flight Program
//

countdownTimer(5).
// Launch
lock throttle to twr_throttle.
stage.
clearscreen.
print "Liftoff!".
gravityTurn(2).
stage.  // Deploy payload fairing
clearscreen.
print "Payload fairing separation.".
orbitalInsertion().
fineTuneOrbit().
integrateReturnTrajectory().
landingBurn().