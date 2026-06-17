// Initialize
wait until ship:unpacked.
ship:partstitled("Probodobodyne OKTO2")[0]:controlfrom().
clearscreen.
Print "Beginning Flight Test 2".
// Control steering and thrust
sas off.
rcs off.
lock throttle to 0.
lock steering to heading(90,90).
stage.  // Start booster engines

//
//   Functions
//

function F_thrust {
    // Returns the vector of thrust from a given throttle and position vector
    parameter throttle_arg.  // Throttle %
    parameter position_arg.  // Position vector in m
    return throttle_arg * ship:maxthrustat(kerbin:atm:altitudepressure(position_arg:mag-kerbin:radius)) * 1000 * ship:facing:forevector:normalized.  // Thrust in N
}

function F_gravity {
    // Returns the vector of gravity from a given mass and position vector using Newton's law of gravity
    parameter mass_arg.  // Mass in kg
    parameter position_arg.  // Position vector in m
    return kerbin:mu * mass_arg / (position_arg:mag)^2 * kerbin:postion:normalized.  // Gravity in N
}

function F_drag {
    // Returns the vector of drag based on the drag equation F_drag = kPv^2/T
    parameter drag_constant.  // Drag constant in K*kg/(atm*m)
    parameter position_arg.  // Position vector in m from which pressure(atm) and temperature(K) can be calculated.
    parameter velocity_arg.  // Velocity vector in m/s

    // Calculate pressure and temperature
    declare local pressure to kerbin:atm:altitudepressure(position_arg:mag - kerbin:radius).  // Pressure in atm
    declare local temperature to kerbin:atm:alttemp(position_arg:mag - kerbin:radius).  // Temperature in K

    return drag_constant*pressure*velocity_arg^2/temperature.  // Magnitude of drag in N
}

function momentum {
    // Returns the magnitude of momentum from a given mass and velocity
    parameter mass_arg.  // Mass in kg
    parameter velocity_arg.  // Velocity in m/s
    return mass_arg*velocity_arg.  // Magnitude of momentum in kg*m/s
}

function measureDragConstant {
    // Measures the drag constant based on change in momentum
    // F_drag = dp/dt - F_thrust - F_gravity
    // F_drag = kPv^2/T
    set last_momentum_vec to momentum(ship:mass*1000,ship:airspeed)*ship:obt:velocity:.
    set timer to time.
    wait 0.  // Wait 1 physics tick.
    set dpdt to (momentum(ship:mass*1000, ship:airspeed))
}


//
//   Variables
//

// Flight parameters
set countdown to 10.  // Countdown timer in s
set tgt_twr to 1.1.  // Target TWR, dimensionless ratio

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
// Note: this program will assume that the drag model works and integrate numerically backwards from a target destination
lock throttle to twr_throttle.
stage.
clearscreen.
print "Liftoff!".

// Measure drag constant during vertical ascent
until ship:altitude > 300 {
}