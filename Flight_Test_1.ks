wait until ship:unpacked.
ship:partsnamed("Booster Control Module")[0]:controlfrom().
clearscreen.
Print "Beginning Flight Test 1".

// Control steering and thrust
sas off.
lock steering to heading(90,90).
lock throttle to 0.
stage.  // Start booster engines

//
//   VARIABLES 
//

// Launch parameters
set countdown to 50. // Launch countdown timer in s
set initial_TWR to 1.2.  // Inital TWR, dimensionless ratio
set tgt_altitude to 80000.  // Target orbital altitude in m

set ascent_drag_coefficient_measurements to list().  // List of measured drag coefficients during ascent
set descent_drag_coefficient_measurements to list().  // List of measured drag coefficients during descent
set booster_engines to list().  // List of booster engines
// Populate booster_engines list
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
    }
}

set dmdt to 0.  // Maxium fuel outflow rate in kg/s
set dvdt to 0.  // Magnitude of acceleration in m/s^2
set ascent_drag_coeff to 0. // Ascent drag proportionality constant, dimensionless
set descent_drag_coeff to 0. // Descent drag proortionality constant, dimensionless

lock ship_mass to ship:mass * 1000.  // Current ship mass is in kg, ship:mass is in Mg
lock temp to kerbin:atm:alttemp(ship:altitude).  // Predicited temperature in K
lock pressure to ship:sensors:pres * 1000.  // Atmospheric pressure in Pa, sensors:pres is in kPa
lock F_gravity to kerbin:mu * ship_mass / (ship:altitude + kerbin:radius)^2. // Magnitude of local gravity in N
lock F_thrust to ship:thrust * 1000.  // Magnitude of current ship thrust in N, ship:thrust is in kN
lock theta to ship:prograde:pitch.  // Ship's pitch in degrees MIGHT NEED TO ADJUST FOR MEASUREMENT FROM VERTICAL

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
    
    // Ascent profile
    if ship:thrust > 0 {
        // From Newton's second law and the drag equation assuming a veritcal ascent:
        // k = T/(PV^2) * (F_thrust - F_gravity*cos(theta) - dm/dt*v - m*dv/dt)
        // Note: use predicted temperature only and assume that it is always proportional to the real temperature

        // Measure mass outflow and acceleration
        measureDmdt().
        measureDvdt().

        // Calculate k
        ascent_drag_coefficient_measurements:add(temp / (pressure * ship:airspeed^2) * (F_thrust - F_gravity*cos(theta) - dmdt*ship:airspeed - ship_mass*dvdt)).  // KOS trig functions take inputs in degrees
    }

    // Descent profile
    else {
        // From Newton's second law and the drag equation, where theta represents the pitch angle from the vertical:
        // k = T/(PV^2) * (F_gravity*cos(theta) - m*dv/dt)
        // Note: use predicted temperature only and assume that it is always proportional to the real temperature

        // Measure acceleration
        measureDvdt().
        descent_drag_coefficient_measurements:add(temp /  (pressure * ship:airspeed^2) * (F_gravity*cos(theta) - ship_mass*dvdt)).  // KOS trig functions take inputs in degrees
    }
}

global function averageDragCoefficient {
    // Averages current drag coefficient measurements
    parameter profile.
    
    if profile = "ascent" {
        for i in ascent_drag_coefficient_measurements {
            set ascent_drag_coeff to ascent_drag_coeff + ascent_drag_coefficient_measurements[i].  // Sums elements in list
        }
        set ascent_drag_coeff to ascent_drag_coeff / ascent_drag_coefficient_measurements:length.  // Divides by the length to complete the average
    }

    if profile = "descent" {
        for i in descent_drag_coefficient_measurements {
            set descent_drag_coeff to descent_drag_coeff + descent_drag_coefficient_measurements[i].  // Sum elements in list
        }
        set descent_drag_coeff to descent_drag_coeff / descent_drag_coefficient_measurements:length.  //Divides by the length to complete the average
    }
 }

 global function executeBurn {
    // Executes the given burn
    parameter burn_node.
    lock steering to burn_node:deltav.
    
    // Calculate the burn time of the node using Tsiolkovsky's rocket equation
    // m_final = m_initial * e^(-deltaV / (Isp * g0))
    // m_final - m_initial / mass outflow rate = burn time in s
    set m_initial to ship_mass.
    set visp to ship:engines[0]:visp.  // Vacuum ISP in seconds, assumes that all boosters have the same visp
    set mass_outflow_rate to 0.
    for i in booster_engines {
        set mass_outflow_rate to mass_outflow_rate + booster_engines[i]:maxmassflow * 1000.  // maxmassflow is in Mg/s
    }
    set m_final to m_initial * e^(-1*burn_node:deltav:mag / (visp * g0)).  // Expected mass at the end of the burn in kg
    set burn_time to (m_final - m_initial) / mass_outflow_rate.  // Burn time in s

    // Wait for burn time, then execute
    wait until ship:obt:eta:nextnode = (burn_time / 2).
    lock throttle to 1.
    // Throttle down towards end of burn
    wait until burn_node:deltav:mag * ship:maxthrust * 1000 / m_final.  // Waits until the remaining burn can be completed in 1 second at full thrust (roughly)
    set remaining_burn to burn_node:deltav:mag.
    lock throttle to burn_node:nodedeltav:mag / remaining_burn.
    wait until burn_node:nodedeltav:mag / remaining_burn < .05.  // Waits until burn is within 5% of the last second remaining (TWEAK THIS LATER)
    lock throttle to 0.
    unlock steering.
    print "Burn complete.".
 }

//
//   PRELAUNCH
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
until countdown = 0 {
    print "T - " + countdown + "   " at (0,1).
    print "" at (0,2).
    wait 1.
    set countdown to countdown - 1.
}

//
//   INITIAL CLIMB
//

clearscreen.
print "Liftoff!".
// Initial vertical climb
lock throttle to initial_TWR * F_gravity / (ship:maxthrust * 1000).  // Locks throttle to the desired TWR, ship:maxthrust is in kN
stage.  // Release tower clamps
// Make drag coefficient measurements periodically
until ship:altitude > 200 {
    measureDragCoefficient().
    wait 1.
}

//
//   ROLL PROGRAM and GRAVITY TURN
// 

// Crude gravity turn, optimize in future
// Pitch 10 degrees from the vertical
lock steering to heading(90,80).
// Lock to surface prograde when velocity catches up
wait until ship:srfprograde:pitch < 80.
lock steering to ship:srfprograde.
// Inital_theta = arccos(F_gravity * initial_TWR / F_maxthrust)  NOTE: this is the angle from the veritcal
set initial_theta to arccos(F_gravity * initial_TWR / (ship:maxthrust * 1000)).  // ship:maxthrust is in kN
// Wait until gravity turns to target pitch
wait until ship:srfprograde:pitch < 90 - initial_theta.
lock throttle to 1.
// Cut throttle when apoapsis reaches target orbital altitude
wait until ship:obt:apoapsis > tgt_altitude.
lock throttle to 0.

//
//   ORBITAL INSERTION
//

// Crude orbital insertion program, optimize in future
// Coast until ship is in space
wait until ship:altitude > 70000.
stage.  // Deploy payload fairing
// Calculate orbital velocity at apoapsis with the vis-viva equation (1/a is 0 for parabolas)
// v^2 = MU(2/r - 1/a) 
set apoapsis_vel to sqrt(kerbin:mu * 2 / (kerbin:radius + ship:altitude)).
// Calculate orbital velocity at ship apoapsis
// v = sqrt(MU/r)
set tgt_obt_vel to sqrt(kerbin:mu / (kerbin:radius + ship:obt:apoapsis)).
// Create a maneuver node at apoapsis with nesecary delta v in the prograde direction
set obt_insertion_burn to node(ship:obt:eta:apoapsis, 0, 0, tgt_obt_vel-apoapsis_vel).
add obt_insertion_burn.
print "Awaiting orbital insertion burn.".
executeBurn(obt_insertion_burn).
// Payload deployment
lock steering to north.
wait until vang(ship:facing:forevector, north:forevector) < 1.  // Waits until the ship is facing within 1 degree of normal
stage.  // Deploy payload
wait 5.

//
//   REENTRY
//

lock steering to ship:retrograde.
wait until vang(ship:facing:forevector,ship:retrograde:forevector) < 1.  // Wait until ship is facing within 1 degree of retrograde
// Deorbitting burn
lock throttle to 1.
wait until ship:obt:periapsis < 33000.
lock throttle to 0.
lock steering to ship:srfretrograde.
wait until ship:altitude < 70000.

// Measure drag coefficients on the way down and attempt a suicide burn