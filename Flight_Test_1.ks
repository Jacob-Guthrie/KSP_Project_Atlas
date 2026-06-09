wait until ship:unpacked.
clearscreen.
Print "Beginning Flight Test 1".

// Control steering and thrust
sas off.
lock steering to heading(90,90).
lock throttle to 0.
stage.

//
//  VARIABLES 
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
//  FUNCTIONS
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

//
//  PRELAUNCH
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
until countdown = 0 {
    print "T - " + countdown + "   " at (1,0).
    wait 1.
    set countdown to countdown - 1.
}

//
//  INITIAL CLIMB
//

clearscreen.
print "Liftoff!".

// Initial vertical climb
lock throttle to initial_TWR * F_gravity / (ship:maxthrust * 1000).  // Locks throttle to the desired TWR, ship:maxthrust is in kN

// Make drag coefficient measurements periodically
until ship:altitude > 200 {
    measureDragCoefficient().
    wait 1.
}

//
//  ROLL PROGRAM and GRAVITY TURN
// 

// Set inital pitch from the vertical
// Inital_theta = arccos(F_gravity * initial_TWR / F_maxthrust)
set initial_theta to 90 - arcCos(F_gravity * initial_TWR / ship:maxthrust * 1000).  // ship:maxthrust is in kN
lock steering to heading(90, initial_theta).  // Heading due east with desired pitch
lock throttle to 1.

// Lock steering to prograde once velocity has adjusted.
wait until ship:prograde:pitch <= initial_theta.
lock steering to ship:prograde.

// Cut throttle when apoapsis reaches target orbital altitude
wait until ship:obt:apoapsis > tgt_altitude.
lock throttle to 0.

//
//  ORBITAL INSERTION
//

// Coast until ship is in space
wait until ship:altitude > 70000.
