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
//   Variables
//

// Flight parameters
set countdown to 10.  // Countdown timer in s
set tgt_twr to 1.1.  // Target TWR, dimensionless ratio

lock F_gravity to kerbin:mu * ship:mass * 1000 / (kerbin:radius+ship:altitude)^2 * ship:body:position:normalized.  // A vector representing the current force of gravity in m/s^2
lock twr_throttle to min(1, max(0, tgt_twr * F_gravity:mag / ship:maxthrustat(kerbin:atm:altitudepressure(ship:altitude)))).  // A throttle ratio [0,1] that provides target TWR

//
//   Functions
//



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


