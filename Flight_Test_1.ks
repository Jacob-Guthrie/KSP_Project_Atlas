wait until ship:unpacked.
clearscreen.
Print "Running Flight Test 1".

//
// INITALIZE VARIABLES
//

set countdown to 50. // Launch countdown timer

//
// INITIALIZE FUNCTIONS
//

global function calculateDragPropotionalityCoefficient {
    // Calculates the drag proportionality coefficient k in the equation F_drag = kPv^(2)/T where P is atmospheric pressure, T is temperature, and v is velocity.
    // From Newton's Second law: F_drag = F_thrust - F_gravity - dp/dt where p is momentum.
} 

//
// PRELAUNCH
//

// Control steering and thrust
sas off.
lock steering to up.
lock throttle to 0.0.

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
stage.
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
