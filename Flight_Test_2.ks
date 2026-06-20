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
set countdown to 5.  // Countdown timer in s
set tgt_twr to 1.1.  // Target TWR, dimensionless ratio
set tgt_alt to 80000.  // Target orbital altitude in m

set max_mass_outflow to 0.  // Maximum fuel outflow rate in kg/s, calculated when booster_engine list is created

// Lists
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
    // Returns the vector of thrust from a given throttle and position vector in the direction of velocity
    parameter throttle_arg.  // Throttle %
    parameter position_arg.  // Position vector in m
    parameter velocity_arg.  // Velocity vector in m/s
    return throttle_arg * ship:maxthrustat(kerbin:atm:altitudepressure(position_arg:mag-kerbin:radius)) * 1000 * velocity_arg:normalized.  // Thrust vector in N
}

function F_gravity {
    // Returns the vector of gravity from a given mass and position vector using Newton's law of gravity
    parameter mass_arg.  // Mass in kg
    parameter position_arg.  // Position vector in m
    return kerbin:mu * mass_arg / (position_arg:mag)^2 * position_arg:normalized.  // Gravity vector in N
}

function F_drag {
    // Returns the vector of drag based on the drag equation F_drag = 1/2 * rho * v^2 * Cd * A
    parameter position_arg.  // Position vector in m
    parameter velocity_arg.  // Velocity vector in m/s
    parameter C_drag.  // Drag coefficient, dimensionless
    parameter A.  // Cross sectional area in m^2

    // Calculate fluid density rho, in kg/m^3
    // rho = pressure / (287.053 * temperature)
    declare rho to (kerbin:atm:altitudepressure(position_arg:mag - kerbin:radius) * constant:atmtokpa * 1000) / (287.053 * kerbin:atm:alttemp(position_arg:mag - kerbin:radius)).

    return 1/2 * rho * velocity_arg:mag^2 * C_drag * A * -1*velocity_arg:normalized.  // Drag vector in N 
}

function F_net {
    // Returns the net force vector from thrust, gravity, and drag
    parameter throttle_arg.  // Throttle %
    parameter position_arg.  // Position vector in m
    parameter velocity_arg.  // Velocity vector in m/s
    parameter mass_arg.  // Mass in kg
    parameter C_drag.  // Drag coefficient, dimensionless
    parameter A.  // Cross sectional area in m^2
    return F_thrust(throttle_arg, position_arg, velocity_arg) + F_gravity(mass_arg, position_arg) + F_drag(position_arg, velocity_arg, C_drag, A).  // Net force vector in N
}

function momentum {
    // Returns the vector of momentum from a given mass and velocity
    parameter mass_arg.  // Mass in kg
    parameter velocity_arg.  // Velocity vector in m/s
    return mass_arg * velocity_arg.  // Momentum vector in kg*m/s
}

function integrateFlightPath {
    // Calculates flight trajectory using the 4th order Runge-Kutta method
 
    local function massFunc {
        // Models mass as a function of time
        parameter t_arg.  // Time in s
        return try_throttle * max_mass_outflow * t_arg + initial_mass.  // Mass in kg
    }

    local function drdt {
        // Differential equation modelling kerbin's position vector
        // dr/dt = -v ; r(0) = kerbin:position:mag
        parameter velocity_arg.
        return -1 * velocity_arg. 
    }

    local function dvdt {
        // Differential equation modelling the ship's velocity vector
        // dv/dt = (F_net - dm/dt * v) / m ; v(0) = ship:alt
        parameter t_arg.
        parameter position_arg.
        parameter velocity_arg.
        return (F_net(try_throttle, position_arg, velocity_arg, massFunc(t_arg), .0066, 21.694) - dmdt * velocity_arg) / massFunc(t_arg). 
    }

    declare try_throttle to 1.
    set pos_list to list().
    set vel_list to list().
    declare initial_mass to 0.
    declare dmdt to 0.
    clearscreen.

    until false {
        // This loop tries different throttle settings until it finds one that provides the desired trajectory
        pos_list:clear().
        vel_list:clear().

        // Parameters
        declare n to 0.  // Index
        declare t to 0.  // Elapsed time in s
        declare step to 0.001.  // KSP tries to do a physics tick every .02 seconds for a total of 50 times a second
        set initial_mass to ship:mass * 1000.  // Mass in kg
        set dmdt to try_throttle * max_mass_outflow.  // Change in mass with respect to time in kg/s

        // Inital conditions
        pos_list:add(kerbin:position+ 281*kerbin:position:normalized).
        vel_list:add(36.7 * heading(90,85):forevector).

        // Apply Runge-Kutta method for given conditions until the calculated trajectory reaches 30 km
        until pos_list[n]:mag - kerbin:radius > 30000 {
            print "Loop #: " + (n+1) at (0,0).
            // drdt = f(vel)
            // dvdt = f(t,pos,vel)
            declare pos_k1 to drdt(vel_list[n]).
            declare vel_k1 to dvdt(t, pos_list[n], vel_list[n]).

            declare pos_k2 to drdt(vel_list[n]+vel_k1*step/2).
            declare vel_k2 to dvdt(t+step/2, pos_list[n]+pos_k1*step/2, vel_list[n]+vel_k1*step/2).

            declare pos_k3 to drdt(vel_list[n]+vel_k2*step/2).
            declare vel_k3 to dvdt(t+step/2, pos_list[n]+pos_k2*step/2, vel_list[n]+vel_k2*step/2).

            declare pos_k4 to drdt(vel_list[n]+vel_k3*step).
            declare vel_k4 to dvdt(t+step, pos_list[n]+pos_k3*step, vel_list[n]+vel_k3*step).

            // Iterate
            pos_list:add(pos_list[n] + step/6 * (pos_k1 + 2*pos_k2 + 2*pos_k3 + pos_k4)).
            vel_list:add(vel_list[n] + step/6 * (vel_k1 + 2*vel_k2 + 2*vel_k3 + vel_k4)).
            print "Pos Mag: " + pos_list[n+1]:mag at(0,3).
            print "Vel Mag: " + vel_list[n+1]:mag at(0,4).
            print "Current Theta: " + vang(pos_list[n+1], vel_list[n+1]) at(0,5).
            print "Current Mass: " + massFunc(t) at(0,6).
            set t to t + step.
            set n to n + 1.

            // Break if altitude ever drops
            if pos_list[n]:mag < pos_list[n-1]:mag {
                print "Altitude dropped on loop: " + (n+1) at(0,10).
                break.
            }
        } 
        // Check that the trajectory ends with a pitch between 5 and 15 degrees (FINE TUNE LATER)
        declare theta to vang(pos_list[n], vel_list[n]).
        if theta > 95 AND theta < 105 {
            // Success! Return try_throttle and begin gravity turn
            clearscreen.
            print "Success!".
            print "Altitude: " + (pos_list[n]:mag - kerbin:radius).
            print "Theta: " + theta.
            print "Throttle: " + try_throttle.
            return try_throttle.  
        } else if theta > 105 {
            // Throttle too high
            print "Theta: " + theta at(0,2).
            set try_throttle to try_throttle / 2.
            print "Trying throttle: " + try_throttle at(0,1).
        } else {
            // Throttle too low
            print "Theta: " + theta at(0,2).
            set try_throttle to try_throttle + (1 - try_throttle) / 2.
            print "Trying throttle: " + try_throttle at(0,1).
        }
    }
}

lock twr_throttle to min(1, max(0, tgt_twr * F_gravity(ship:mass * 1000, kerbin:position):mag / (ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)) * 1000))).  // A throttle ratio [0,1] that provides target TWR

//
//   Flight Program
//

// Countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor
set tgt_throttle to integrateFlightPath().
//set tgt_throttle to 1.
until countdown = 0 {
    print "T - " + countdown + "   " at (0,1).
    set countdown to countdown - 1.
    wait 1.
}

// Launch and Vertical Ascent
lock throttle to twr_throttle.
stage.
clearscreen.
print "Liftoff!".
wait until ship:altitude > 200.

// Roll program and gravity turn
lock steering to heading(90,85).
lock throttle to tgt_throttle.
wait until vang(heading(90,85):forevector, ship:srfprograde:forevector) < 0.5.
lock steering to ship:srfprograde.

wait until ship:altitude > 30000.
lock throttle to 1.
wait until ship:obt:apoapsis > tgt_alt.
lock throttle to 0.