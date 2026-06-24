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
set hover_slam_alt to 1000.  // Altitude in m at which the hoverslam will begin

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

set launchpad to ship:geoposition.

//
//   Functions
//

function F_gravity {
    // Returns the gravity force vector in N from a given mass and position vector using Newton's law of gravity
    // F_gravity = mu * m/r^2
    parameter position_arg.  // Kerbin osition vector in m
    parameter mass_arg.  // ship mass in kg
    return kerbin:mu * mass_arg / (position_arg:mag)^2 * position_arg:normalized.
}

function F_drag {
    // Returns the drag force vector in N from a given velocity and position using Ferram Aerospace's formula
    parameter position_arg.  // Kerbin position vector in m
    parameter velocity_arg.  // Ship velocity vector in m/s

    declare ship_altitude to position_arg:mag - kerbin:radius.  // Ship's altitude in m
    // Copy the velocity_arg vector and modify its direction for the purposes of the aeroforceat() method.
    declare modified_vel to velocity_arg:vec.
    set modified_vel:direction to ship:facing:inverse.

    declare drag to addons:far:aeroforceat(ship_altitude, modified_vel) * 1000.  // Drag force magnitude in N
    declare drag_vector to drag * -1 * velocity_arg:normalized.  // Drag vector in N
    return drag_vector.
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

function burnMass {
    // Returns the required propellant mass required for a given burn using Tsiolkovsky's rockey equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    parameter deltav.  // Delta-v in m/s
    parameter isp.  // Specific impulse in s
    parameter m_initial.  // Initial mass in kg

    declare m_final to m_initial * constant:e^(-1 * deltav / (isp * 9.81)).
    return m_initial - m_final.
}

function gravityTurn {
    // Performs a gravity turn during ascent
    parameter initial_pitch.  // Inital pitch from the vertical in degrees

    // Initial pitch after tower is cleared
    wait until ship:altitude > 200.
    lock steering to heading(90,90-initial_pitch).
    wait until vang(ship:srfprograde:forevector, heading(90,90-initial_pitch):forevector) < 0.5.
    lock steering to ship:srfprograde.

    // Dynamically adjusts throttle
    lock theta to vang(heading(90,0):forevector, ship:srfprograde:forevector).
    lock tgt_theta to 88 * constant:e^(-0.05723 * ship:altitude / 1000).
    lock throttle to min(1, max(0.05,-1/10 * (theta-tgt_theta) + 1)).

    wait until ship:obt:apoapsis > tgt_altitude.
    rcs on.
    lock throttle to 0.
}

function executeBurn {
    // Executes the given maneuver node
    parameter burn_node.
    rcs on.
    lock steering to burn_node:deltav.
    
    // Calculate burn time in seconds
    // burn time = - burn mass (kg) / mass outflow (kg/s)
    set burn_time to -1 * burnMass(burn_node:deltav:mag, booster_engines[0]:visp, ship:mass * 1000) / max_mass_outflow.

    // Execute burn
    wait until nextnode:eta < (burn_time / 2) AND vang(ship:facing:forevector, burn_node:deltav) < 1.
    lock throttle to 1.

    // Lock steering to prevent oscilation at end of burn
    wait burn_time - 1.
    lock steering to ship:facing:forevector.
    wait 1.

    lock throttle to 0.
    rcs off.
    unlock steering.
    remove burn_node.
}

function orbitalInsertion {
    // Calculates and executes the maneuver node for insertion into a roughly circular orbit at apoapsis

    // Calculate velocity at apoapsis using conservation of energy
    // v_final = sqrt(v_inital^2 + 2*kerbin:mu*(1/r_initial - 1/r_final))
    declare apoapsis_vel to sqrt(ship:velocity:orbit:mag^2 + 2*kerbin:mu*(1/(kerbin:radius+ship:obt:apoapsis) - 1/(kerbin:radius+ship:altitude))).
    // Calculate orbital velocity at ship apoapsis
    // v = sqrt(MU/r)
    declare tgt_obt_vel to sqrt(kerbin:mu / (kerbin:radius + ship:obt:apoapsis)).

    // Create a maneuver node at apoapsis with nesecary delta v in the prograde direction
    declare obt_insertion_burn to node(timespan(ship:obt:eta:apoapsis), 0, 0, tgt_obt_vel-apoapsis_vel).
    add obt_insertion_burn.
    executeBurn(obt_insertion_burn).
}

function deployPayload {
    // Deploys the booster's payload

    // Adjust deploy direction so that the payload is out of the booster's way
    rcs on.
    lock steering to north.
    wait until vang(ship:facing:forevector, north:forevector) < 1.
    stage.  // Deploy payload decoupler
    unlock steering.
    rcs off.
}

function calculateReturnTrajectory {
    // Calculates the the time elapsed and angle rotated during return trajectory, then creates a maneuver node
    // Uses the 4th order Runge-Kutta method to model the return trajectory for the booster

    // Use the vis-viva equation to determine the deltav of a deorbiting burn targeting a periapsis of 50km
    // v^2 = mu * (2/r - 1/a)
    // a = (2*body radius + apoapsis + periapsis)/2
    declare semimajor_axis to (2 * kerbin:radius + ship:obt:semimajoraxis + 50000) / 2.
    declare deorbit_vel to sqrt(kerbin:mu * (2/kerbin:position:mag * 1/semimajor_axis)).  // The velocity required to deorbit
    declare avg_vel to sqrt(kerbin:mu / ship:semimajoraxis).  // The average velocity of the current orbit
    declare deltav to avg_vel - deorbit_vel.  // The delta-v for a deorbiting burn
    
    // Calculate the mass remaining after the given burn using Tsiolkovsky's rocket equation
}

function deorbit {
    // Performs a deorbiting burn

    rcs on.
    unlock steering.
    sas on.  // Using SAS mode makes the ship pitch faster
    wait 1.
    set sasmode to "RETROGRADE".
    wait until vang(ship:facing:forevector, ship:retrograde:forevector) < 0.5.
    lock throttle to 0.5.
    wait until ship:obt:periapsis < 51000.
    lock throttle to 0.
    sas off.
    rcs off.
}

function landingBurn {
    // Controls attitude during atmospheric rentry and performs a landing burn

    brakes on.
    unlock steering.
    rcs off.

    // Calculate time until hoverslam altitude using simple projectile motion, solves for t with the quadratic formula
    // Note: neglecting drag gives a built in buffer since actual acceleration will be slower than this formula predicts
    // 0 = y_0 - v_y * t - 1/2 * g * t^2
    lock impact_time to (-1*ship:verticalspeed - sqrt((ship:verticalspeed)^2 + 2*9.81*(ship:altitude - hover_slam_alt))) / -9.81.

    // Calculate the burn time to negate air speed.
    lock landing_burn_time to -1 * burnMass(ship:airspeed, booster_engines[0]:slisp, ship:mass * 1000) / max_mass_outflow.

    wait until impact_time < landing_burn_time.
    // Locks throttle to target velocity of 20 m/s
    lock steering to ship:srfretrograde.
    lock throttle to min(1, max(0, ((ship:airspeed - 20) / 4.2 + 1) * F_gravity(ship:mass*1000, kerbin:position):mag / (ship:maxthrust * 1000))).
    wait until ship:altitude - ship:geoposition:terrainheight < 50.
    // Locks throttle to target velocity of 5 m/s
    lock throttle to min(1, max(0, ((ship:airspeed - 5) / 0.83 + 1) * F_gravity(ship:mass*1000, kerbin:position):mag / (ship:maxthrust * 1000))).
    lock steering to up.
    wait until ship:altitude - ship:geoposition:terrainheight < 5.
    lock throttle to 0.
    wait 1.
    print "Landed!".
}

lock twr_throttle to min(1, max(0, tgt_twr * F_gravity(ship:mass * 1000, kerbin:position):mag / (ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)) * 1000))).  // A throttle ratio [0,1] that provides target TWR
lock T_max to ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude))*1000.  // Ship's current max thrust in N

//
//   Flight Program
//

countdownTimer(5).
// Launch
lock throttle to twr_throttle.
stage.
print "Liftoff!" at (0,1).
gravityTurn(2).
wait until ship:altitude > 70000.
unlock steering.
stage.  // Deploy payload fairing
orbitalInsertion().
deorbit().
rcs on.
lock steering to ship:srfretrograde.
wait until ship:altitude < 70000.
landingBurn().