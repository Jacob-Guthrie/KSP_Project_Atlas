// Initialize
wait until ship:unpacked.
ship:partstitled("Probodobodyne OKTO2")[0]:controlfrom().
clearscreen.
Print "Beginning Flight Test 2".
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
set countdown to 10.  // Countdown timer in s
set tgt_twr to 1.1.  // Target TWR, dimensionless ratio
set tgt_alt to 80000.  // Target orbital altitude in m

// Lists
set ascent_profile_drag_constants to list().  // List of measured drag constants for the ascent profile in K*kg/(atm*m)
set descent_profile_drag_constants to list().  // List of measrued drag constants for the descent profile in K*kg/(atm*m)

//
//   Functions
//

function F_thrust {
    // Returns the vector of thrust from a given throttle and position vector
    parameter throttle_arg.  // Throttle %
    parameter position_arg.  // Position vector in m
    return throttle_arg * ship:maxthrustat(kerbin:atm:altitudepressure(position_arg:mag-kerbin:radius)) * 1000 * ship:facing:forevector:normalized.  // Thrust vector in N
}

function F_gravity {
    // Returns the vector of gravity from a given mass and position vector using Newton's law of gravity
    parameter mass_arg.  // Mass in kg
    parameter position_arg.  // Position vector in m
    return kerbin:mu * mass_arg / (position_arg:mag)^2 * kerbin:postion:normalized.  // Gravity vector in N
}

function F_drag {
    // Returns the vector of drag based on the drag equation F_drag = kPv^2/T
    parameter k.  // Drag constant in K*kg/(atm*m)
    parameter position_arg.  // Position vector in m from which pressure(atm) and temperature(K) can be calculated.
    parameter velocity_arg.  // Velocity vector in m/s

    // Calculate pressure and temperature
    declare pressure to kerbin:atm:altitudepressure(position_arg:mag - kerbin:radius).  // Pressure in atm
    declare temperature to kerbin:atm:alttemp(position_arg:mag - kerbin:radius).  // Temperature in K

    return k * pressure * velocity_arg:mag^2 / temperature * ship:srfretrograde:forevector:normalized.  // Drag vector in N  
}

function momentum {
    // Returns the vector of momentum from a given mass and velocity
    parameter mass_arg.  // Mass in kg
    parameter velocity_arg.  // Velocity vector in m/s
    return mass_arg * velocity_arg.  // Momentum vector in kg*m/s
}

function measureDragConstant {
    // Measures the drag constant based on change in momentum
    // F_drag = dp/dt - F_thrust - F_gravity
    // F_drag = kPv^2/T
    parameter profile.

    // Measure dp/dt
    declare last_momentum to momentum(ship:mass*1000, ship:obt:velocity:surface).
    declare timer to time.
    wait 0.  // Wait 1 physics tick.
    declare dpdt to (momentum(ship:mass*1000, ship:obt:velocity:surface) - last_momentum) / (time - timer):seconds.

    // Measure drag and calculate drag constant
    declare measured_drag to dpdt - F_thrust(throttle, kerbin:position) - F_gravity(ship:mass * 1000, kerbin:positon).
    declare k to measured_drag:mag * kerbin:atm:alttemp(ship:altitude) / (kerbin:atm:altitudepressure(ship:altitude) * ship:airspeed^2).

    if profile = "ascent" {
        ascent_profile_drag_constants:add(k).
    }
    if profile = "descent" {
        descent_profile_drag_constants:add(k).
    }
}

lock twr_throttle to min(1, max(0, tgt_twr * F_gravity(ship:mass * 1000, ship:altitude) / ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)))).  // A throttle ratio [0,1] that provides target TWR

//
//   Flight Program
//


// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
until countdown = 0 {
    print "T - " + countdown + "   " at (0,1).
    set countdown to countdown - 1.
    wait 1.
}


// Launch
lock throttle to twr_throttle.
stage.
clearscreen.
print "Liftoff!".


// Vertical Ascent
until ship:altitude > 300 {
    measureDragConstant("ascent").
    wait 1.
}
// Average drag constants
set drag_constant to 0.
for i in ascent_profile_drag_constants {
    set drag_constant to drag_constant + ascent_profile_drag_constants[i].
}
set drag_constant to drag_constant / ascent_profile_drag_constants:length.
print "Drag constant: " + drag_constant.
// DEBUG SECTION
log ascent_profile_drag_constants to "ascent_drag_constants.txt".
print "Drag measurements logged.".
// END DEBUG SECTION


// Calculate flight trajectory using 4th order runge-kutta
// dr/dt = -v ; r(0) = kerbin:positon:mag
// dv/dt = 1/m * (F_net - dm/dt * v) ; v(0) = ship:alt
set step to 0.02.  // step size for runge-kutta method. KSP tries to do a physics tick every 0.02 seconds for a total of 50 times per second.
set pos_list to list().
set vel_list to list().

set pos to kerbin:position.
set vel to ship:obt:velocity:surface.
set vel:direction to heading(90,85).

pos_list.add(pos).
vel_list.add(vel).

until condition {
    // Calulate slopes
    set pos_k1 to -1*vel.
    set pos_k2 to -1*(vel + pos_k1 * step / 2).
    set pos_k3 to -1*(vel + pos_k2).

    // Step
    set pos_nextstep to pos + step / 6.
}

// Roll program
lock steering to heading(90,85).
wait until vang(heading(90,85):forevector, ship:srfprograde:forevector) < 0.5.
lock steering to ship:srfprograde.