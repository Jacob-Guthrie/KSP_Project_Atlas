wait until ship:unpacked.

// INITIALIZE VARIABELS

set countdown to 20.  // launch countdown timer
set t to 0.  // mission time variablle

// Data collection lexicons. Pressure values are in kPA, acceleration values are in g's
set flight_dynamic_pressure to lexicon().
set descent_dynamic_pressure to lexicon().
set flight_pressure to lexicon().
set descent_pressure to lexicon().
set flight_acceleration to lexicon().
set descent_acceleration to lexicon().
set flight_expected_acceleration to lexicon().
set descent_expected_acceleration to lexicon().


// PRELAUNCH

// Control steering and throttle
sas off.
lock steering to up.
lock throttle to 0.0.

// Countdown
until countdown = 0 {
    print "t - " + countdown at(0,0).
    wait 1.
    set countdown to countdown - 1.
}

// Lock throttle to a value so that inital TWR = 1.5
// F_thrust = TWR * mg
lock throttle to 1.5 * ship:mass * 9.81 / ship:availablethrust.  // ship:mass is in Mg and ship:available thrust is in kN, so the unit conversions cancel eachother out
stage.
print "Liftoff!" at(0,0).

// FLIGHT

until ship:deltav:current < 5 {

    // Read pressure sensors
    flight_dynamic_pressure:add(t, ship:q * constant:atmtokpa).
    flight_pressure:add(t, ship:sensors:pres).

    // Read accelerometer
    flight_acceleration:add(t, ship:sensors:acc:mag).

    // Calculate expected acceleration if there is no drag:
    // a = 1/m * (F_thrust - F_gravity - dm/dt * v)
    flight_expected_acceleration:add(t, 1/(ship:mass * 1000) * (ship:engines[0]:thrust * 1000 - ship:sensors:grav:mag * 9.81 + ship:engines[0]:massflow * 1000 * ship:airspeed)).  //ship:mass is in Mg, ship:engines[0]:thrust is in kN, ship:sensors:grav is in g's, ship:massflow is in Mg/s

    wait 1.
    set t to t + 1.
} 

// DESCENT

lock throttle to 0.0.
stage.

until ship:status = "landed" or ship:status = "splashed" {

    // Read pressure sensors
    descent_dynamic_pressure:add(t, ship:q * constant:atmtokpa).
    descent_pressure:add(t, ship:sensors:pres).

    // Read accelerometer
    descent_acceleration:add(t, ship:sensors:acc:mag).

    // Calculate expected acceleration if there is no drag:
    // a = 1/m * (F_gravity - dm/dt * v)
    descent_expected_acceleration:add(t, 1/(ship:mass * 1000) * (ship:sensors:grav:mag * 9.81 + ship:engines[0]:massflow * 1000 * ship:airspeed)).  //ship:mass is in Mg, ship:sensors:grav is in g's, ship:massflow is in Mg/s

    wait 1.
    set t to t + 1.
}

print "Landed!".

// EXPORT DATA

switch to 0.
log flight_dynamic_pressure to "flight_dynamic_pressure.txt".
log flight_pressure to "flight_pressure.txt".
log flight_acceleration to "flight_acceleration.txt".
log flight_expected_acceleration to "flight_expected_acceleration.txt".
log descent_dynamic_pressure to "descent_dynamic_pressure.txt".
log descent_pressure to "descent_pressure.txt".
log descent_acceleration to "descent_acceleration.txt".
log descent_expected_acceleration to "descent_expected_acceleration.txt".

print "Flight data exported.".