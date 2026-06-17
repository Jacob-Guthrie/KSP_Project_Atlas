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

set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s, calculated when booster_engine list is created

// Lists
set ascent_profile_drag_constants to list().  // List of measured drag constants for the ascent profile in K*kg/(atm*m)
set descent_profile_drag_constants to list().  // List of measrued drag constants for the descent profile in K*kg/(atm*m)
set booster_engines to list().  // List of booster engines
// Populate booster_engines list and measures max mass outflow rate
for eng in ship:engines {
    if eng:ignition {
        booster_engines:add(eng).
        set max_mass_outflow to max_mass_outflow - eng:maxmassflow * 1000.  // maxmassflow is in Mg/s
    }
}

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

function F_net {
    // Returns the net force vector from thrust, gravity, and drag
    parameter throttle_arg.  // Throttle %
    parameter position_arg.  // Position vector in m
    parameter velocity_arg.  // Velocity vector in m/s
    parameter mass_arg.  // Mass in kg
    parameter k.  // Drag constant in K*kg/(atm*m)
    return F_thrust(throttle_arg, position_arg) + F_gravity(mass_arg, position_arg) + F_drag(k, position_arg, velocity_arg).  // Net force vector in N
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

function integrateFlightPath {
    // Calculates flight trajectory using the 4th order runge-kutta method
    // dr/dt = -v ; r(0) = kerbin:positon:mag
    // dv/dt = 1/m * (F_net - dm/dt * v) ; v(0) = ship:alt
 
    function massFunc {
        // Models mass as a function of time
        parameter t.  // Time in s
        return try_throttle * max_mass_outflow * t + initial_mass.  // Mass in kg
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

    set try_throttle to 1.
    set pos_list to list().
    set vel_list to list().

    until condition {  // Until flight path predicts a flatish trajectory at 30000 m altitude

        pos_list:clear().
        vel_list:clear().

        // Inital conditions
        set n to 1.  // Index variable
        set step to .02.  // KSP tries to do a physics tick every .02 seconds for a total of 50 times a second
        set t to 0.  // Elapsed time in s
        set initial_mass to ship:mass * 1000.  // Mass in kg
        set dmdt to try_throttle * max_mass_outflow.  // Change in mass with respect to time in kg/s

        // Get the ship's surface velocity and change its direction
        set initial_vel to ship:obt:velocity:surface.
        set initial_vel:direction to heading(90,85).

        pos_list:add(kerbin:position).  // Position vector in m
        vel_list:add(initial_vel).  // Surface velocity vector in m/s

        until condition { // Until altitude reaches 30000 m
            // Iterate
            set pos_k1 to -1 * vel_list[n-1].
            set vel_k1 to (F_net(try_throttle, pos_list[n-1], vel_list[n-1], massFunc(t), drag_constant) - dmdt * vel_list[n-1]) / massFunc(t).

            set pos_k2 to -1 * vel_list[n-1]*step/2*vel_k1.
            set vel_k2 to (F_net(try_throttle, pos_list[n-1]*step/2*pos_k1, vel_list[n-1]*step/2*vel_k1, massFunc(t+step/2), drag_constant) - dmdt * vel_list[n-1]*step/2*vel_k1) / massFunc(t+step/2).

            set n to n + 1.
        }   

        if long {
            set try_throttle to try_throttle / 2. 
        }
        if short {
            set try_throttle to try_throttle + (1 - try_throttle) / 2.
        }
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

// Roll program
lock steering to heading(90,85).
wait until vang(heading(90,85):forevector, ship:srfprograde:forevector) < 0.5.
lock steering to ship:srfprograde.