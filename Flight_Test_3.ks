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
set hover_alt to 5000.  // Altitude in m at which the hoverslam will begin

set launchpad to ship:geoposition.
set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s, calculated when booster_engines list is created. Note: value is always negative

// Lists
set booster_engines to list().  // List of booster engines
// Populate booster_engines list and measure max mass outflow rate
set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
        set max_mass_outflow to max_mass_outflow - eng:maxmassflow * 1000.  // maxmassflow is in Mg/s
    }
}

//
//   Functions
//

function F_gravity {
    // Returns the gravity force vector in N from a given mass and position vector using Newton's law of gravity
    // F_gravity = mu * m/r^2
    parameter position_arg.  // Kerbin position vector in m
    parameter mass_arg.  // ship mass in kg

    return kerbin:mu * mass_arg / position_arg:mag^2 * position_arg:normalized.
}

function F_drag {
    // Returns the drag force vector in N from a given velocity and position using Ferram Aerospace's formula
    parameter position_arg.  // Kerbin position vector in m
    parameter velocity_arg.  // Ship velocity vector in m/s

    local ship_altitude to position_arg:mag - kerbin:radius.  // Ship's altitude in m
    // Copy the velocity_arg vector and modify its direction for the purposes of the aeroforceat() method.
    local modified_vel to velocity_arg:vec.
    set modified_vel:direction to ship:facing:inverse.

    local drag to addons:far:aeroforceat(ship_altitude, modified_vel) * 1000 * 3.5.  // Drag vector in N
    set drag:direction to velocity_arg:direction:inverse.  // Rotate the vector to oppose the velocity argument
    return drag.
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

    local m_final to massAfterBurn(deltav, isp, m_initial).
    return (m_final - m_initial) / max_mass_outflow.
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
    parameter burn_node.

    // Take control
    rcs on.
    lock steering to burn_node:deltav.

    // Get burn time
    local burn_time to timeToBurn(burn_node:deltav:mag, booster_engines[0]:visp, ship:mass * 1000).
    // Execute burn
    wait until nextnode:eta < (burn_time / 2).
    lock steering to ship:facing:forevector.
    lock throttle to 1.
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
    executeBurn(obt_insertion_burn).
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

function calculateReturnTrajectory {
    // Calculates the return trajectory and executes a precise deorbiting burn
    // Uses the 4th order Runge-Kutta method to model the return trajectory for the booster

    // Deploy airbrakes for the purpose of calculating drag
    brakes on.

    // Use the vis-viva equation to determine the deltav of a deorbiting burn targeting a periapsis of 50km
    // v^2 = mu * (2/r - 1/a)
    // a = (2*body radius + apoapsis + periapsis)/2
    local deorbit_semimajor_axis to (2 * kerbin:radius + ship:obt:semimajoraxis - kerbin:radius + 50000) / 2.
    local deorbit_vel to sqrt(kerbin:mu * (2/kerbin:position:mag - 1/deorbit_semimajor_axis)).  // The velocity required to deorbit
    local current_avg_vel to sqrt(kerbin:mu / ship:obt:semimajoraxis).  // The average velocity of the current orbit
    local deltav to current_avg_vel - deorbit_vel.  // The delta-v for a deorbiting burn

    clearscreen.
    print "Calculating return trajectory...".
    print "Deorbit burn paramaters:" at(0,1).
    print "Semimajor axis: " + deorbit_semimajor_axis at(0,2).
    print "Deorbit velocity: " + deorbit_vel at (0,3).
    print "Delta-V: " + deltav at (0,4).
    
    // Parameters
    local n to 0.  // Index
    local t to 0.  // Time in s
    local step to 1.  // KSP tries to do a physics update 50 times a second.

    // Inital conditions
    local m_final to massAfterBurn(deltav, booster_engines[0]:visp, ship:mass * 1000).
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

        return 1/m_final * ( F_gravity(position_arg, m_final) + F_drag(position_arg, velocity_arg) ).
    }

    // 4th order Runge-Kutta method
    until pos_list[n]:mag - kerbin:radius < hover_alt {
        // Readout
        print "Loop #: " + (n+1) at(0,6).
        print "Pos mag: " + pos_list[n]:mag + " " at(0,7).
        print "Vel mag: " + vel_list[n]:mag + " " at(0,8).

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

    // Release brakes
    brakes off.

    // Calculate angle the ship rotated around Kerbin
    local ship_theta to vang(pos_list[0], pos_list[n]).

    // Calculate the angle Kerbin rotates during the time elapsed
    // t(s) * 360 (deg) / rotation_period (s) = angle rotated in degrees
    local kerbin_theta to t * 360 / kerbin:rotationperiod.

    // Calculate the angle between the landing zone and the ship at the start of the deorbiting burn
    local target_theta to ship_theta - kerbin_theta.

    // Calculate the ship's current average angular velocity in degrees/s
    // omega (rad/s) = v / r
    // NOTE: must convert to degrees and subtract kerbin's angular velocity
    local current_angular_vel to current_avg_vel / ship:obt:semimajoraxis * 180 / constant:pi - 360 / kerbin:rotationperiod.

    // Calculate time until target_theta is reached
    // t = 1/omega * (lng of landing zone - target theta - ship lng)
    local deorbit_time to 1/current_angular_vel * launchpad:lng - target_theta - ship:geoposition:lng.

    // Readout
    print "Tracjetory set!" at(0,10).
    print "Ship rot angle: " + ship_theta at(0,11).
    print "Kerbin rot angle: " + kerbin_theta at(0,12).
    print "Current angular vel: " + current_angular_vel at(0,13).
    print "Deorbiting time: " + deorbit_time at (0,14).

    // Create a maneuver node and execute
    local deorbit_burn to node(timespan(deorbit_time), 0, 0, -1*deltav).
    add deorbit_burn.
    executeBurn(deorbit_burn).
}

function deorbit {
    // Use the vis-viva equation to determine the deltav of a deorbiting burn targeting a periapsis of 50km
    // v^2 = mu * (2/r - 1/a)
    // a = (2*body radius + apoapsis + periapsis)/2
    local deorbit_semimajor_axis to (2 * kerbin:radius + ship:obt:semimajoraxis - kerbin:radius + 50000) / 2.
    local deorbit_vel to sqrt(kerbin:mu * (2/kerbin:position:mag - 1/deorbit_semimajor_axis)).  // The velocity required to deorbit
    local current_avg_vel to sqrt(kerbin:mu / ship:obt:semimajoraxis).  // The average velocity of the current orbit
    local deltav to current_avg_vel - deorbit_vel.  // The delta-v for a deorbiting burn

    wait 10.
    // Create a maneuver node and execute
    local deorbit_burn to node(timespan(0,0,0,4,0), 0, 0, -1*deltav).
    add deorbit_burn.
    executeBurn(deorbit_burn).
}

function landingBurn {
    // Controls attitude during atmospheric rentry and performs a hoverslam on the launchpad

    // Take control
    rcs on.
    brakes on.
    lock steering to ship:srfretrograde.

    // Release control when ship enters atmosphere
    wait until ship:altitude < 70000.
    unlock steering.
    rcs off.
    sas off.

    // Calculate time until hoverslam altitude using simple projectile motion, solves for t with the quadratic formula
    // Note: neglecting drag gives a built in buffer since actual acceleration will be slower than this formula predicts
    // 0 = y0 - vertical_speed*t - g/2*t^2
    local lock impact_time to (-1*ship:verticalspeed - sqrt((ship:verticalspeed)^2 + 2*9.81*(ship:altitude - hover_alt))) / -9.81.

    // Calculate the burn time to negate air speed.
    local lock burn_time to timeToBurn(ship:airspeed, booster_engines[0]:slisp, ship:mass*1000).

    // Slow down at the given altitude
    clearscreen.
    print "Reentry Program".
    local cd_lex to lexicon().  // A list of drag coefficients keyed by airspeed
    until impact_time < burn_time {
        if not cd_lex:haskey(round(ship:airspeed)) and mod(ship:airspeed,5) < 0.1 {
            // If the ship's airspeed is close to a multiple of 5, document drag coefficient
            cd_lex:add(round(ship:airspeed), addons:far:cd).
        }
        wait 0.
    }
    writeJson(cd_lex, "cd_lex.json").
    lock steering to ship:srfretrograde.
    // Locks throttle to target velocity of 20 m/s
    lock throttle to min(1, max(0, ((ship:airspeed - 20) / 4.2 + 1) * F_gravity(kerbin:position, ship:mass * 1000):mag / (ship:maxthrust * 1000))).
    wait until ship:airspeed < 25.
    lock steering to up.

    // ADD CODE HERE TO STEER TOWARDS LANDING PAD

    wait until ship:altitude - ship:geoposition:terrainheight < 50.
    // Locks throttle to target velocity of 5 m/s
    lock throttle to min(1, max(0, ((ship:airspeed - 5) / 0.83 + 1) * F_gravity(kerbin:position, ship:mass * 1000):mag / (ship:maxthrust * 1000))).
    lock steering to up.
    wait until ship:altitude - ship:geoposition:terrainheight < 5.
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
deorbit().
landingBurn().