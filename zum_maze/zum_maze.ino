#include <Wire.h>
#include <ZumoShield.h>
ZumoMotors motors;
ZumoReflectanceSensorArray linesensors(QTR_NO_EMITTER_PIN);
ZumoBuzzer buzzer;
Pushbutton button(ZUMO_BUTTON);
const int MAX_PATH = 64;
char recorded_path[64] = {};
void setup() {

  char recorded_path_test[64] = {'L','S','L','L','U','L','U','S','L','R','L','L','L','U','S','R','S','R','R','L','S','L','R','U','L','L','U','L','U','L','L','L','R','$'};
  for(int i=0; i<64; i++){
    // $ will act as "junk" in the array, to represent the end of the sequence
    recorded_path[i] = '$';
    }
  int i;
  int spin_direction = 1;
  motors.setSpeeds(80*spin_direction, -80*spin_direction);
  for(i = 0; i<100; i++){
    linesensors.calibrate();
    if(i%50 == 25){ // every 50 loops, starting on loop 25...
      spin_direction = -spin_direction;
      motors.setSpeeds(80*spin_direction, -80*spin_direction);
    }
    delay(20);
  }
  motors.setSpeeds(0,0);
  delay(500);
}

unsigned int sensor_vals[6];
int num_turns;
int THRESHOLD = 400;
int BASE_SPEED = 200;
int line_position;
int turn_counter = 0;
bool finish_detected = false;

const int INTERSECTION_LEFT_TURN = 100;
const int INTERSECTION_RIGHT_TURN = 10;
const int INTERSECTION_T = 110;
const int INTERSECTION_CROSS = 111;
const int INTERSECTION_LEFT_T = 101;
const int INTERSECTION_RIGHT_T = 11;
const int FINISH = 99;
const int NO_INTERSECTION = 1;

void loop() {
  line_position = linesensors.readLine(sensor_vals);
  
  bool line_on_left = sensor_vals[0] > THRESHOLD;
  bool line_on_right = sensor_vals[5] > THRESHOLD;

  int intersection_type = NO_INTERSECTION;
  
  if(line_on_left || line_on_right) {
    motors.setSpeeds(BASE_SPEED, BASE_SPEED);
    intersection_type = get_intersection_type();
  }

  if(intersection_type == NO_INTERSECTION && sensor_vals[1] < THRESHOLD && sensor_vals[2] < THRESHOLD && sensor_vals[3] < THRESHOLD && sensor_vals[4] < THRESHOLD){
    recorded_path[turn_counter++] = 'U';
    u_turn();
  }
  
  switch(intersection_type){
    case INTERSECTION_LEFT_TURN:
    case INTERSECTION_LEFT_T:
    case INTERSECTION_CROSS:
    case INTERSECTION_T:
      turn_left();
      recorded_path[turn_counter++] = 'L';
      break;
    case INTERSECTION_RIGHT_TURN:
      recorded_path[turn_counter++] = 'R';
      turn_right();
      break;
    case INTERSECTION_RIGHT_T:
    recorded_path[turn_counter++] = 'S';
    case NO_INTERSECTION:
      follow_line();
      break;
    case FINISH:
      solved();
      break;
    default: follow_line();
    }
}
int turnValue(char turn){
  switch (turn){
    case 'R':
      return 90;
    case 'L':
      return 270;
      // we never actually feed this a u turn,
      // but just in case...
    case 'U':
      return 180;
    case 'S':
      return 0;
    }
    return -1;
  }

char* fully_reduce_path(char *input_path){
  char* path = input_path;
  char* reduced = reduce_path(path);
  // this is kind of dumb (and it is possible, I think, to have paths that take more iterations
  // to reduce fully) but it works for this application.
  for(int i=0; i<6; i++){
    path = reduce_path(path);
    }
  return path;
  }
char* reduce_path(char* path){
  //Serial.begin(9600);
  //Serial.print("Reducing path\n");
  // assumes path length at maximum is 64.
  for(int i=0;i<MAX_PATH-2;i++){
    // we will be accessing array elements i to i+2 at any given time,
    // so must we wary of the bounds
    if(path[i]=='$'||path[i+1]=='$'||path[i+2]=='$'){
      // if path has a $ in it, that means we have probably replaced something and should move forward.
      break;
      }
    int total_angle = 0;

    if(path[i+1] == 'U'){
      char turnToTake = '_';
      total_angle = (turnValue(path[i])+ 180 + turnValue(path[i+2])) % 360;
      if (total_angle == 90)  { turnToTake = 'R'; }
      if (total_angle == 270) { turnToTake = 'L'; }
      if (total_angle == 0)   { turnToTake = 'S'; }
      if (total_angle == 180) { turnToTake = 'U'; }
      //Serial.print(total_angle);
      //Serial.print("\n");
      path[i] = turnToTake;

      // move everything over by two
      for (int moveChar=i+1; moveChar<MAX_PATH-2;moveChar++){
        path[moveChar] = path[moveChar + 2];
        }
    }
    }
    return path;
  }
bool paths_equal(char* a, char* b){
  // very simple comparison of path arrays a and b
  for (int i=0; i < MAX_PATH; i++){
    if(a[i] != b[i]){
      return false;
      }
    }
    return true;
  }
int get_intersection_type() {
  // returns the type of intersection at the robot's current position.
  bool left = sensor_vals[0] > THRESHOLD;
  bool right = sensor_vals[5] > THRESHOLD;

  bool ever_right = right;
  bool ever_left = left;

  int timeout = 0;

  while(left || right){
    linesensors.read(sensor_vals);
    left = sensor_vals[0] > THRESHOLD;
    right = sensor_vals[5] > THRESHOLD;
    
    ever_left = ever_left || left;
    ever_right = ever_right || right;

    if(timeout++ > 75){
      buzzer.playNote(NOTE_E(2),100,15);
      return FINISH;
      }
    }
  bool line_straight = sensor_vals[2] > THRESHOLD || sensor_vals[3] > THRESHOLD;
  // enumerated values for turn types are set up specially so that this will work
  // could also use a bitmap to make this faster and save space, but that sounds miserable
  return (ever_left * 100) + (ever_right * 10) + line_straight;
}

void turn_left() {
  motors.setSpeeds(-BASE_SPEED, BASE_SPEED);
  while(sensor_vals[3] > THRESHOLD){
    linesensors.read(sensor_vals);
  }
  while(sensor_vals[3] < THRESHOLD){
    linesensors.read(sensor_vals);
  }
}

void turn_right() {
  motors.setSpeeds(BASE_SPEED, -BASE_SPEED);
  while(sensor_vals[2] > THRESHOLD){
    linesensors.read(sensor_vals);
  }
  while(sensor_vals[2] < THRESHOLD){
    linesensors.read(sensor_vals);
  }
}

void u_turn() {
  motors.setSpeeds(-BASE_SPEED, BASE_SPEED);
  while(sensor_vals[0] < THRESHOLD){
    line_position = linesensors.readLine(sensor_vals);
  }
  while(line_position > 3000 || line_position < 2000){
    line_position = linesensors.readLine(sensor_vals);
  }
}

void solved(){
  Serial.begin(9600);
  Serial.write(recorded_path,64);
  motors.setSpeeds(0,0);
  button.waitForButton();
  followRecordedPath(fully_reduce_path(recorded_path));
}

void followRecordedPath(char* path){
  int current_instruction = 0;
  while(1){
    line_position = linesensors.readLine(sensor_vals);
    
    bool line_on_left = sensor_vals[0] > THRESHOLD;
    bool line_on_right = sensor_vals[5] > THRESHOLD;
  
    int intersection_type = NO_INTERSECTION;
    
    if(line_on_left || line_on_right) {
      motors.setSpeeds(BASE_SPEED, BASE_SPEED);
      intersection_type = get_intersection_type();
    }
  
    if(intersection_type != NO_INTERSECTION){
      switch(path[current_instruction++]){
        case 'L':
        turn_left();
        break;
        case 'R':
        turn_right();
        break;
        case 'S':
        follow_line();
        break;
        case '$':
        buzzer.playNote(NOTE_E(2),100,15);
        solved();
      }
    }
    else{
      follow_line();
      }
  }
}


double PROPORTION_GAIN = 0.2;
double DERIVATIVE_GAIN = 3;
int last_error = 0;
void follow_line(){
  // follow line
  int error = line_position - 2500;
  int error_change = error - last_error;
  int left_speed = BASE_SPEED + PROPORTION_GAIN * error + DERIVATIVE_GAIN * error_change;
  int right_speed = BASE_SPEED + -PROPORTION_GAIN * error + -DERIVATIVE_GAIN * error_change;
  last_error = error;

  // set speed
  motors.setSpeeds(left_speed, right_speed);
}
