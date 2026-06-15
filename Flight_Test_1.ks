wait until ship:unpacked.
ship:partstitled("Probodobodyne OKTO2")[0]:controlfrom().
clearscreen.
Print "Beginning Flight Test 1".

// Control steering and thrust
sas off.
rcs off.
lock steering to heading(90,90).
lock throttle to 0.
stage.  // Start booster engines

//
//   VARIABLES 
//

// Launch parameters
set countdown to 10. // Launch countdown timer in s
set initial_TWR to 1.1.  // Inital TWR, dimensionless ratio
set tgt_altitude to 80000.  // Target orbital altitude in m
set hover_slam_alt to 500.  // Altitude in m at which the hoverslam will begin

set descent_drag_coefficient_measurements to list().  // List of measured drag coefficients during descent
set booster_engines to list().  // List of booster engines
// Populate booster_engines list and measures max mass outflow rate
set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
        set max_mass_outflow to max_mass_outflow - eng:maxmassflow * 1000.  // maxmassflow is in Mg/s
    }
}

set dmdt to 0.  // Maxium fuel outflow rate in kg/s
set dvdt to 0.  // Magnitude of acceleration in m/s^2
set descent_drag_coeff to 0. // Descent drag proortionality constant, dimensionless
set slisp to ship:engines[0]:slisp.  // Sea level ISP in seconds, assumes that all booster engines have the same slisp
set visp to ship:engines[0]:visp.  // Vacuum ISP in seconds, assumes that all booster engines have the same visp

lock ship_mass to ship:mass * 1000.  // Current ship mass is in kg, ship:mass is in Mg
lock temp to kerbin:atm:alttemp(ship:altitude).  // Predicited temperature in K
lock pressure to ship:sensors:pres * 1000.  // Atmospheric pressure in Pa, sensors:pres is in kPa
lock F_gravity to kerbin:mu * ship_mass / (ship:altitude + kerbin:radius)^2. // Magnitude of local gravity in N
lock F_thrust to ship:thrust * 1000.  // Magnitude of current ship thrust in N, ship:thrust is in kN
lock theta to vang(ship:up:forevector, ship:srfprograde:forevector).  // The angle between the ship's surface prograde and the vertical in degrees
lock TWR_throttle to min(1, max(0, tgt_TWR * F_gravity / (ship:maxthrustat(ship:altitude) * 1000))).  // Creates a throttle setting that locks to a specific TWR, will stay in the bounds [0,1], ship:maxthrust is in kN
lock vert_TWR_throttle to min(1, max(0, tgt_TWR * F_gravity / (ship:maxthrustat(ship:altitude) * 1000 * abs(cos(theta))))).  // Creates a throttle setting that locks to a TWR accounting only for the vertical component of thrust, will stay in the bounds [0,1], ship:maxthrust is in kN


//
//   FUNCTIONS
//

global function measureDmdt {
    // Measures current mass outflow

    set temp_var to 0.

    for eng in booster_engines {
        set temp_var to temp_var - eng:massflow * 1000.  // Sums mass outflow in kg/s, eng:massflow is in Mg/s
    }

    set dmdt to temp_var.
}

global function measureDvdt {
    // Measures the magnitude of acceleration 

    set ship_last_speed to ship:airspeed.
    set timer to time.
    wait 0. // Wait one physics tick
    set dvdt to (ship:airspeed - ship_last_speed) / (time-timer):seconds.
}

global function measureDragCoefficient {
    // Calculates the drag proportionality coefficient k in the drag equation F_drag = kPv^(2)/T where P is atmospheric pressure, v is velocity, and T is temperature. k has unites of K * s^2   
    // Descent profile
    // From Newton's second law and the drag equation, where theta represents the pitch angle from the vertical:
    // k = T/(PV^2) * (F_gravity*cos(theta) - m*dv/dt)
    // Note: use predicted temperature only and assume that it is always proportional to the real temperature

    // Measure acceleration
    measureDvdt().
    descent_drag_coefficient_measurements:add(temp /  (pressure * ship:airspeed^2) * (F_gravity*abs(cos(theta)) - ship_mass*dvdt)).  // KOS trig functions take inputs in degrees
}

global function averageDragCoefficient {
    // Averages current drag coefficient measurements
    for i in descent_drag_coefficient_measurements {
        set descent_drag_coeff to descent_drag_coeff + descent_drag_coefficient_measurements[i].  // Sum elements in list
    }
    set descent_drag_coeff to descent_drag_coeff / descent_drag_coefficient_measurements:length.  //Divides by the length to complete the average
}

global function executeBurn {
    // Executes the given burn
    parameter burn_node.
    lock steering to burn_node:deltav.
    
    // Calculate the burn time of the node using Tsiolkovsky's rocket equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    // m_final - m_initial / mass outflow rate = burn time in s
    set m_initial to ship_mass.
    set m_final to m_initial * constant:e^(-1*burn_node:deltav:mag / (visp * 9.81)).  // Expected mass at the end of the burn in kg
    set burn_time to (m_final - m_initial) / max_mass_outflow.  // Burn time in s
    print "burn_time = " + burn_time.

    // Wait for burn time, then execute
    wait until nextnode:eta < (burn_time / 2).
    lock throttle to 1.
    // Throttle down towards end of burn
    wait burn_time - 0.25.
    set remaining_burn to burn_node:deltav:mag.
    lock throttle to burn_node:deltav:mag / remaining_burn.
    wait until burn_node:deltav:mag < 0.4.
    lock throttle to 0.
    unlock steering.
    remove burn_node.
    print "Burn complete.".
}

global function landingBurn {
    // Calculates ideal landing burn and performs a hoverslam

    lock steering to ship:srfretrograde.
    // Calculate the burn time using Tsiolkovsky's rocket equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    // m_final - m_initial / mass outflow rate = burn time in s
    set m_initial to ship_mass.
    //set m_final to m_initial * constant:e^(-1*ship:airspeed / (slisp * 9.81)).  // Expected mass at the end of burn

    // Calculate time until hoverslam altitude using simple projectile motion, solves for t with the quadratic formula
    // Note: neglecting drag gives a built in buffer since actual acceleration will be slower than this formula predicts
    lock impact_time to (-1*ship:verticalspeed - sqrt((ship:verticalspeed)^2 + 2*9.81*(ship:altitude - hover_slam_alt))) / -9.81.

    // Calculate time to negate ship:airspeed at sea level
    lock landing_burn_time to ((m_initial * constant:e^(-1*ship:airspeed / (slisp * 9.81))) - m_initial) / max_mass_outflow.
    
    wait until ship:altitude < 30000.

    // Wait until landing burn then perform hoverslam
    until impact_time < landing_burn_time {
        measureDragCoefficient().
        wait 1.
    }
    lock throttle to 1.
    wait until ship:airspeed < 25.
    lock throttle to min(1, max(0, ((ship:airspeed - 20) / 4.2 + 1) * F_gravity / (ship:maxthrust * 1000))).
    wait until ship:altitude < 50.
    lock throttle to min(1, max(0, ((ship:airspeed - 5) / 0.83 + 1) * F_gravity / (ship:maxthrust * 1000))).
    lock steering to up.
    wait until ship:altitude < 5.
    lock throttle to 0.
    wait 1.
    print "Landed!".
    unlock impact_time.
    unlock landing_burn_time.
}

//
//   PRELAUNCH
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
until countdown = 0 {
    print "T - " + countdown + "   " at (0,1).
    set countdown to countdown - 1.
    wait 1.
}

//
//   INITIAL CLIMB
//

// Initial vertical climb
set initial_throttle to initial_TWR * F_gravity / (ship:maxthrust * 1000). 
set g_turn_throttle to initial_throttle.
lock throttle to g_turn_throttle.
stage.  // Release tower clamps
clearscreen.
print "Liftoff!".
wait until ship:altitude > 200.

//
//   ROLL PROGRAM and GRAVITY TURN
// 

// Crude gravity turn code, optimize in future
// Inital_theta = arccos(F_gravity * initial_TWR / F_maxthrust)  NOTE: this is the angle from the veritcal
set initial_theta to arccos(F_gravity * initial_TWR / (ship:maxthrust * 1000)).  // ship:maxthrust is in kN
print "Initial_theta = " + initial_theta.
lock steering to heading (90,85).
wait 1.
wait until vang(ship:srfprograde:forevector, up:forevector) < 1.
lock steering to ship:srfprograde.
set g_turn_throttle to initial_TWR * F_gravity / (ship:maxthrust * 1000). 
lock throttle to g_turn_throttle.
until vang(ship:srfprograde:forevector, up:forevector) > initial_theta {
    set g_turn_throttle to min(1, max(initial_throttle, 1 / (ship:srfprograde:pitch + initial_theta - 90))).
}
print " Velocity pitch = " + ship:srfprograde:pitch.
print "THROTTLE UP".
lock throttle to 1.
// Cut throttle when apoapsis reaches target orbital altitude
wait until ship:obt:apoapsis > tgt_altitude.
lock throttle to 0.
print "MECO.".

//
//   ORBITAL INSERTION
//

// Crude orbital insertion program, optimize in future
// Coast until ship is in space
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
// Payload deployment
lock steering to north.
wait until vang(ship:facing:forevector, north:forevector) < 1.  // Waits until the ship is facing within 1 degree of normal
stage.  // Deploy payload
print "Payload deployed.".
wait 5.

//
//   REENTRY
//

lock steering to ship:retrograde.
wait until vang(ship:facing:forevector,ship:retrograde:forevector) < 1.  // Wait until ship is facing within 1 degree of retrograde
// Deorbitting burn
lock throttle to 0.5.
wait until ship:obt:periapsis < 33000.
lock throttle to 0.
lock steering to ship:srfretrograde.
wait until ship:altitude < 70000.
landingBurn().
switch to 0.
log descent_drag_coefficient_measurements to "drag_coeff.txt".
print "Logged drag coefficients".