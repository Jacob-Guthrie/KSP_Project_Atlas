//
// INITALIZE VARIABLES
//

set countdown to 10. // launch countdown timer
set height to ship:altitude. // current altitude of the ship
set r to height + body:radius. // current distance from the center of the body

//
// INITIALIZE FUNCTIONS
//

global function calculateDragPropotionalityCoefficient {

    //Calculates the drag proportionality coefficient k in the equation F_drag = kPv^(2)/T where P is atmospheric pressure, T is temperature, and v is velocity.
    //From Newton's Second law during a vertical ascent, F_drag = F_thrust - F_gravity - dp/dt where p is momentum.
    //

    //predict 

} 

//
// PRELAUNCH
//

sas off.
lock steering to up.
lock throttle to 0.0.

//countdown
// Note: since KSP does not simulate mechanical features or failures, this countdown is purely for flavor

until countdown = 0 {
    print "t - " + countdown at(0,0).
    wait 1.
    set countdown to countdown - 1.
}

//
// INITAL CLIMB (Calculate trajectory by 2 km)
//

lock throttle to 1.0.
stage.
print "Liftoff!" at(0,0).

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
