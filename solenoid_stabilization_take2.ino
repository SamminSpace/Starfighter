float z=0
//height from sea level, in KM
float currentVelocity=0
//rotational velocity, degrees/second. Counterclockwise=positive
solenoidCC=0
//solenoid, produce counterclockwise force
solenoidC=0
//solenoid, produce clockwise force

while(z>=2000){
  if (currentVelocity>8){
    solenoidC=1 //solenoid output = on
  }else if(currentVelocity<1){
    solenoidC=0 //solenoid output = off
  }else{
    solenoidC=0
  }
  if (currentVelocity<(0-8)){
    solenoidCC=1
  }else if(currentVelocity>(0-1)){
    solenoidCC=0
  }else{
    solenoidCC=0
  }
  
}