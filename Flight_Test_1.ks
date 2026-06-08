wait until ship:unpacked.
clearscreen.
Print "Beginning Flight Test 1".

// Control steering and thrust
sas off.
lock steering to up.
lock throttle to 0.0.
stage.

//
// INITALIZE VARIABLES
//

set countdown to 50. // Launch countdown timer in s
set dmdt to 0.  // Maxium fuel outflow rate in kg/s

lock ship_mass to ship:mass * 1000.  // Current ship mass in kg, ship:mass is in Mg
lock temp to kerbin:atm:alttemp(ship:altitude).  // Predicited temperature in K
lock pressure to ship:sensors:pres * 1000.  // Atmospheric pressure in Pa, sensors:pres is in kPa
lock F_gravity to kerbin:mu * ship_mass / (ship:altitude + kerbin:radius)^2. // Magnitude of local gravity in N
lock F_thrust to ship:thrust * 1000.  // Magnitude of current ship thrust in N, ship:thrust is in kN

set ascent_drag_coefficient_measurements to list().  // List of measured drag coeffcients during ascent
set booster_engines to list().  // List of booster engines

// Populate booster_engines list
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
    }
}

//
// INITIALIZE FUNCTIONS
//

global function calculateDragPropotionalityCoefficient {
    // Calculates the drag proportionality coefficient k in the drag equation F_drag = kPv^(2)/T where P is atmospheric pressure, v is velocity, and T is temperature. k has unites of K * s^2
    
    // Ascent profile
    if ship:thrust > 0 {
        // From Newton's second law and the drag equation assuming a veritcal ascent:
        // k = T/(PV^2) * (F_thrust - F_gravity - dm/dt*v - m*dv/dt)
        // Note: use predicted temperature only and assume that it is always proportional to the real temperature

        // Measure dv/dt
        set ship_last_vel to ship:airspeed.
        set timer to time.
        wait 0.  // Wait one physics tick
        set dvdt to (ship:airspeed - ship_last_vel) / (time-timer):seconds.

        // Measure current total dm/dt
        for eng in booster_engines {
            set dmdt to dmdt - eng:massflow * 1000
        }

        // Calculate k
        ascent_drag_coefficient_measurements:add( temp / (pressure * ship:airspeed^2) * (F_thrust - F_gravity - dmdt*ship:airspeed - ship_mass*dvdt)).

        // Reset dm/dt
        set dmdt to 0.
    }

    // Descent profile
    else {

    }
} 

//
// PRELAUNCH
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor

until countdown = 0 {
    print "T - " + countdown + "   " at (1,0).
    wait 1.
    set countdown to countdown - 1.
}

//
// INITAL CLIMB
//

lock throttle to 1.0.
clearscreen.
print "Liftoff!".

// Calculate trajectory by 2 km
until ship:altitude > 2000 {

}

//
// ROLL MANEUVER (Steer towards right heading by 2.5 km)
//



//
// ORBITAL INSERTION (Get to orbit)
//



//
// REENTRY (Measure drag proportionality constant,  Aim for ocean, Attempt a landing burn
//
