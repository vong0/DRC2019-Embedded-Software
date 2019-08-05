#ifdef STANLEY

// Libraries
#include "src/RobotBase.h"
#include "src/Stanley.h"
#include "src/Euler.h"

// Robot
RobotBase robot;

// Odometry
Pose pose;
Euler ode;

// Path tracker
Stanley stanley;
ControlInput speedInput;  // speed controls from stanley

// Create path
PathData path;
const float cx[] = {0,0,0,0,0,0,0,0,0,0,-0.0939900000000000,-0.355760000000000,-0.715990000000000,-1.11441000000000,-1.51584000000000,-1.95291000000000,-2.30725000000000,-2.69034000000000,-3.11523000000000,-3.52294000000000,-3.92484000000000,-4.35302000000000,-4.80599000000000,-5.23752000000000,-5.68613000000000,-6.14011000000000,-6.58308000000000,-7.03084000000000,-7.42222000000000,-7.80009000000000,-8.17372000000000,-8.38261000000000,-8.53322000000000,-8.75803000000000,-8.90885000000000,-8.92380000000000,-8.80642000000000,-8.56762000000000,-8.22196000000000,-7.96590000000000,-7.64840000000000,-7.27389000000000,-6.89043000000000,-6.55306000000000,-6.18079000000000,-5.76797000000000,-5.29634000000000,-4.80873000000000,-4.31869000000000,-3.85803000000000,-3.40417000000000,-2.93546000000000,-2.45775000000000,-1.97505000000000,-1.49552000000000,-1.03970000000000,-0.628400000000000,-0.351030000000000,-0.193390000000000,-0.187080000000000,-0.256450000000000,-0.274380000000000,-0.274380000000000,-0.274380000000000};
const float cy[] = {0,0,0,0,0,0,0,0,0,0,0.000540000000000000,0.00964000000000000,0.0428700000000000,0.0919800000000000,0.150100000000000,0.215090000000000,0.270000000000000,0.267870000000000,0.252260000000000,0.250220000000000,0.250690000000000,0.245100000000000,0.245320000000000,0.251120000000000,0.261730000000000,0.284090000000000,0.319350000000000,0.353830000000000,0.368560000000000,0.370220000000000,0.374200000000000,0.379350000000000,0.384220000000000,0.405070000000000,0.502530000000000,0.649500000000000,0.814200000000000,0.951090000000000,1.01568000000000,1.04601000000000,1.05779000000000,1.05984000000000,1.04522000000000,1.02386000000000,0.994990000000000,0.959040000000000,0.915000000000000,0.872140000000000,0.824740000000000,0.776810000000000,0.737940000000000,0.700960000000000,0.676530000000000,0.665290000000000,0.657000000000000,0.639260000000000,0.612770000000000,0.588960000000000,0.522160000000000,0.413250000000000,0.351820000000000,0.347850000000000,0.347850000000000,0.347850000000000};
const uint8_t N = 64;

void setup() {
  // Setup serial
  NeoSerial.begin(115200);

  // Setup robot
  robot.begin();
  robot.setSpeed(0, 0);

  // Copy path data points
  memcpy(&path.cx, &cx, sizeof(float)*N);
  memcpy(&path.cy, &cy, sizeof(float)*N);
  path.N = N;
  stanley.setPath(path);

  // Wait until serial input
  NeoSerial.println(F("Waiting"));
  while(!NeoSerial.available());
  NeoSerial.read();
  NeoSerial.println(F("Starting program"));
}

// State machine for motor controllers
uint8_t state;

void loop() {
  // Update pose and control speed
  if(robot.sampleData()) {
    // Control speed
    robot.drive();
    
    // Update position
    pose = ode.integrate(robot.getVcm(), robot.getYaw(), robot.getDeltaTime());
    
    // Print data
//    logTime();
    state++; 
  }

  // Update path tracker
  if(state %2 == 0) {
    // Run stanley controller
    speedInput = stanley.calcControlEffort(pose);
    
    // Set speed inputs
    robot.setSpeed(speedInput);

    // Reset state
    printData();
    state = 0; 
  }
}

// Prints data
void printData() {
  // Print position (m)
  NeoSerial.print(pose.x, 5); NeoSerial.print(", ");
  NeoSerial.print(pose.y, 5); NeoSerial.print(", ");
//  NeoSerial.print(pose.yaw); NeoSerial.print(", ");
//  NeoSerial.print(robot.getVcm(), 5); NeoSerial.print(", ");
//  NeoSerial.print(robot.getVd(), 5); NeoSerial.print(", ");
//  NeoSerial.print(robot.getWcm(), 5); NeoSerial.print(", ");
//  NeoSerial.print(robot.getWd(), 5); NeoSerial.print(", ");
  NeoSerial.print(stanley.getCurrTargetIdx()); NeoSerial.print(", ");
//  logTime();
  NeoSerial.println();  
}

#endif
