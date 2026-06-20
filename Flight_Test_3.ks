// Initialize
wait until ship:unpacked.
ship:partstitled("Probodobodyne OKTO2")[0]:controlfrom().
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
set countdown to 5.  // Countdown timer in s
set tgt_altitude to 80000.  // Target orbital altitude in m
set tgt_twr to 1.1.  // Initial takeoff TWR
set initial_pitch to 2.  // Initial pitch from the vertical in degrees

// Lists
set booster_engines to list().  // List of booster engines
// Populate booster_engines list and measures max mass outflow rate
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
    parameter mass_arg.  // Mass in kg
    parameter position_arg.  // Position vector in m
    return kerbin:mu * mass_arg / (position_arg:mag)^2 * position_arg:normalized.
}

global function executeBurn {
    // Executes the given burn
    parameter burn_node.
    rcs on.
    lock steering to burn_node:deltav.
    
    // Calculate the burn time of the node using Tsiolkovsky's rocket equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    // m_final - m_initial / mass outflow rate = burn time in s
    set m_initial to ship:mass*1000.
    set m_final to m_initial * constant:e^(-1*burn_node:deltav:mag / (booster_engines[0]:visp * 9.81)).  // Expected mass at the end of the burn in kg
    set burn_time to (m_final - m_initial) / max_mass_outflow.  // Burn time in s
    print "burn_time = " + burn_time.

    // Wait for burn time, then execute
    wait until nextnode:eta < (burn_time / 2).
    lock throttle to 1.
    // Throttle down towards end of burn
    wait burn_time - 1.
    set remaining_burn to burn_node:deltav:mag.
    lock throttle to burn_node:deltav:mag / remaining_burn.
    wait until burn_node:deltav:mag < 2.
    lock throttle to 0.
    rcs off.
    unlock steering.
    remove burn_node.
    print "Burn complete.".
}

lock twr_throttle to min(1, max(0, tgt_twr * F_gravity(ship:mass * 1000, kerbin:position):mag / (ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)) * 1000))).  // A throttle ratio [0,1] that provides target TWR
lock T_max to ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude))*1000.  // Ship's current max thrust in N

//
//   Flight Program
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
until countdown = 0 {
    print "T - " + countdown + " " at (0,1).
    set countdown to countdown - 1.
    wait 1.
}

// Launch
lock throttle to twr_throttle.
stage.
print "Liftoff!" at (0,1).
wait until ship:altitude > 200.
lock steering to heading(90,90-initial_pitch).
wait until vang(ship:srfprograde:forevector, heading(90,90-initial_pitch):forevector) < 0.5.
lock steering to ship:srfprograde.

lock theta to vang(heading(90,0):forevector, ship:srfprograde:forevector).
lock tgt_theta to 88 * constant:e^(-0.05723 * ship:altitude / 1000).
lock throttle to min(1, max(0.05,-1/10 * (theta-tgt_theta) + 1)).

wait until ship:obt:apoapsis > tgt_altitude.
rcs on.
lock throttle to 0.

// Orbital insertion
clearscreen.
wait until ship:altitude > 70000.
stage.  // Deploy payload fairing
// Calculate velocity at apoapsis using conservation of energy
// v_final = sqrt(v_inital^2 + 2*kerbin:mu*(1/r_initial - 1/r_final))
set apoapsis_vel to sqrt(ship:velocity:orbit:mag^2 + 2*kerbin:mu*(1/(kerbin:radius+ship:obt:apoapsis) - 1/(kerbin:radius+ship:altitude))).
print "Apoapsis_vel = " + apoapsis_vel.
// Calculate orbital velocity at ship apoapsis
// v = sqrt(MU/r)
set tgt_obt_vel to sqrt(kerbin:mu / (kerbin:radius + ship:obt:apoapsis)).
print "tgt_obt_vel = " + tgt_obt_vel.
print "ETA TO APO: " + ship:obt:eta:apoapsis.
// Create a maneuver node at apoapsis with nesecary delta v in the prograde direction
set obt_insertion_burn to node(timespan(ship:obt:eta:apoapsis), 0, 0, tgt_obt_vel-apoapsis_vel).
add obt_insertion_burn.
print "Awaiting orbital insertion burn.".
executeBurn(obt_insertion_burn).