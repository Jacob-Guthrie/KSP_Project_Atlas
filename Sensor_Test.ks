wait until ship:unpacked.

// INITIALIZE VARIABELS

set countdown to 40.  // launch countdown timer
set t to 0.  // mission time variablle

// Data collection lexicons. Altitude values are in m above sea level, pressure values are in kPA, acceleration values are in g's, temperature is not specified (assume K), thrust is in kN
set flight_altitude to lexicon().
set flight_temp to lexicon().
set predicted_temp to lexicon().
set flight_dynamic_pressure to lexicon().
set descent_dynamic_pressure to lexicon().
set flight_pressure to lexicon().
set descent_pressure to lexicon().
set flight_acceleration to lexicon().
set descent_acceleration to lexicon().
set flight_expected_acceleration to lexicon().
set descent_expected_acceleration to lexicon().
set detected_thrust to lexicon().
set predicted_thrust to lexicon().

// PRELAUNCH

// Control steering and throttle
sas off.
lock steering to up.
lock throttle to 0.0.

// Countdown
until countdown = 0 {
    clearscreen.
    print "t - " + countdown.
    wait 1.
    set countdown to countdown - 1.
}

// Lock throttle to a value so that TWR = 1.5
// F_thrust = TWR * mg
lock throttle to 1.5 * ship:mass * 9.81 / ship:availablethrust.  // ship:mass is in Mg and ship:available thrust is in kN, so the unit conversions cancel each other out
stage.
clearscreen.
print "Liftoff!".

// FLIGHT

until ship:deltav:current < 20 {
    clearscreen.
    print "t + " + t.

    // Log altitude
    flight_altitude:add(t, ship:altitude).

    // Log temperature
    flight_temp:add(t, ship:sensors:temp).  // KOS documentation does not specify the unit of temperature. The in game thermometer reads in K so this is the assumption

    // Calculate expected temperature
    predicted_temp:add(t, kerbin:atm:alttemp(ship:altitude)).

    // Read pressure sensors, values in kPA
    flight_dynamic_pressure:add(t, ship:q * constant:atmtokpa).
    flight_pressure:add(t, ship:sensors:pres).

    // Log thrust
    detected_thrust:add(t, ship:thrust).  // ship:thrust is in kN

    // Calculate expected thrust
    predicted_thrust:add(t, throttle * ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude))).  // ship:altitude is passed into altitudepressure method which gives predicted pressure in atm which is then passed into maxthrustat method

    // Read accelerometer
    flight_acceleration:add(t, ship:sensors:acc:mag * 9.81).  // Sensors:acc is in g's

    // Calculate expected acceleration as if there is no drag:
    // a = 1/m * (F_thrust - F_gravity - dm/dt * v)
    flight_expected_acceleration:add(t, 1/(ship:mass * 1000) * (ship:engines[0]:thrust * 1000 - ship:sensors:grav:mag * 9.81 + ship:engines[0]:massflow * 1000 * ship:airspeed)).  //ship:mass is in Mg, ship:engines[0]:thrust is in kN, ship:sensors:grav is in g's, ship:massflow is in Mg/s

    wait 1.
    set t to t + 1.
} 

// DESCENT

lock throttle to 0.0.
stage.

until ship:status = "landed" or ship:status = "splashed" {
    clearscreen.
    print "t + " + t.

    // Log altitude
    flight_altitude:add(t, ship:altitude).

    // Log temperature
    flight_temp:add(t, ship:sensors:temp).  // KOS documentation does not specify the unit of temperature. The in game thermometer reads in K so this is the assumption

    // Calculate expected temperature
    predicted_temp:add(t, kerbin:atm:alttemp(ship:altitude)).

    // Read pressure sensors, values in kPa
    descent_dynamic_pressure:add(t, ship:q * constant:atmtokpa).
    descent_pressure:add(t, ship:sensors:pres).

    // Read accelerometer
    descent_acceleration:add(t, ship:sensors:acc:mag * 9.81).  // Sensors:acc is in g's

    // Calculate expected acceleration as if there is no drag (mass is now constant):
    // a = -F_gravity / m
    descent_expected_acceleration:add(t, ship:sensors:grav:mag * -9.81 / (ship:mass * 1000)). //ship:mass is in Mg, ship:sensors:grav is in g's

    wait 1.
    set t to t + 1.
}

print "Landed!".

// EXPORT DATA

switch to 0.
log flight_altitude to "altitude.txt".
log flight_temp to "flight_temperature.txt".
log predicted_temp to "predicted_temperature.txt".
log flight_dynamic_pressure to "flight_dynamic_pressure.txt".
log flight_pressure to "flight_pressure.txt".
log flight_acceleration to "flight_acceleration.txt".
log flight_expected_acceleration to "flight_expected_acceleration.txt".
log descent_dynamic_pressure to "descent_dynamic_pressure.txt".
log descent_pressure to "descent_pressure.txt".
log descent_acceleration to "descent_acceleration.txt".
log descent_expected_acceleration to "descent_expected_acceleration.txt".
log detected_thrust to "detected_thrust.txt".
log predicted_thrust to "predicted_thrust.txt".

print "Flight data exported.".