function gravity_arm= GravityVector(q)
global R 
gravity_arm=R.gravload(q');