// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 6

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 20 < TEAM_NUMBER
#error "Team number must be within 1 and 20"
#endif

void setup() {
  Serial.begin(115200);

}

int up = 30;
int tilt = 0;
float Time = 0;
float old_time2 = 0;
//----------PID Variables----------------
const float target_value = 0.0;
const float Kp = 0.0003;
const float Ki = 0.0;
const float Kd = 0.0001;
float prev_error = 0.0;
float integral = 0.0;
bool PID_state = false;

void loop() {
  
  float old_time2 = 0;
  // Get the button states
  bool btnA = RR_buttonA();
  bool btnB = RR_buttonB();
  bool btnX = RR_buttonX();
  bool btnY = RR_buttonY();
  bool btnRB = RR_buttonRB();
  bool btnLB = RR_buttonLB();
  bool btnStart = RR_buttonStart();

  control();
  control_servos(btnRB, btnLB);
  

  if(btnStart){
    PID_state = true; 
    old_time2 = Time;
    delay(100);
  }
  
  while (PID_state){
    Time = millis();
    btnStart = RR_buttonStart();
    // we also have RR_setServo3 and RR_setServo4 available
         
    // read the ultrasonic sensors
    //
    //  Serial.print("Ultrasonic=");
    //  Serial.print(RR_getUltrasonic());
    //  Serial.print(" ;; ");
    int sensors[6];
    RR_getLineSensors(sensors);
        
    float output = PID(sensors);
    Serial.print(old_time);
    Serial.print("   ");
    Serial.print(old_time2);
    Serial.print("   ");
    Serial.println(Time);
    // This is important - it sleeps for 0.02 seconds (= 50 times / second)
    // Running the code too fast will overwhelm the microcontroller and peripherals
//    if (Time - old_time > 19000){
//        RR_setServo1(0);
//        RR_setServo4(180);
//        old_time = Time;
//        }
        
    if(btnStart || Time - old_time2 > 20000){
      PID_state = false;
      old_time2 = Time;
      delay(100);   
    }
    
    delay(20);
  }

  delay(20);
}

void control(){
    // Read the four joystick axes
  // These will be in the range [-1.0, 1.0]
  float rightX = RR_axisRX();
  float rightY = RR_axisRY();
  float leftX  = RR_axisLX();
  float leftY  = RR_axisLY();
  
  //---------------controller curve----------
  if (rightX > 0){
    rightX = sq(rightX);
  }
  else{
    rightX = -sq(rightX);
  }
  //-------------------------------------------
    // Arcade-drive scheme
  // Left Y-axis = throttle
  // Right X-axis = steering
  RR_setMotor1(leftY + rightX);
  RR_setMotor2(leftY - rightX);
}
// vim: tabstop=2 shiftwidth=2 expandtab

float PID(int sensors[]){

  float leftY = 0.21;
  
  int weighted_sum = 0;
  int total_weight = 0;
  int weight;
  
  for(int i=0; i<6; i++){
    switch(i){
      case 0:
        weight = 1.0;
        break;
      case 1:
        weight = 1.0;
        break;
      case 2:
        weight = 0.5;
        break;
      case 3:
        weight = -0.5;
        break;
      case 4:
        weight = -1.0;
        break;
      case 5:
        weight = -1.0;
        break;
    }
    weighted_sum += sensors[i] * weight;
  }
  float error = target_value - (float)weighted_sum / 4;

  integral += error;
  float difference = error - prev_error;
  float control_output = Kp * error + Ki * integral + Kd * difference;
//  Serial.print(error);
//  Serial.print("   ");
//  Serial.print(Kp * error);
//  Serial.print("   ");
//  Serial.println(Kd * difference);
  prev_error = error;

  RR_setMotor1(leftY + control_output);
  RR_setMotor2(leftY - control_output);
  return control_output;
}

void control_servos(bool btnRB, bool btnLB){
  // Control servo 1 using the dpad
  // 6 = left, 2 = right, 0 = up, 4 = down, 8 = center
  // (note that you will also see the value 0 if the controller
  //  is disconnected)

  
  if (RR_dpad() == 0) { // left
    // we can't move a servo less than 0 degrees
    if (up <180) up += 10;
  }
  
  else if (RR_dpad() == 4) { // right
    // we can't move a servo past 180 degrees
    // for continuous rotation, try using a DC motor
    if (up >30) up -= 7;
  }
  
  RR_setServo1(180-up);
  RR_setServo4(up);
  Serial.println(up);
//-------------------------------------------------
    if (RR_dpad() == 6) { // left
    // we can't move a servo less than 0 degrees
    if (up > 0) tilt -= 10;
  }
  
  else if (RR_dpad() == 2) { // right
    // we can't move a servo past 180 degrees
    // for continuous rotation, try using a DC motor
    if (up < 180) tilt += 10;
  }
  
  RR_setServo1(180-up);
  RR_setServo4(up);
  Serial.println(up);

  RR_setServo2(180-tilt);
  RR_setServo3(tilt);
  // Control servo 2 using the shoulder buttons
  // This example moves the servo to fixed points
  // You can change the angles based on your mechanism
  // (this is great for a mechanism that only has 2 states,
  //  such as a grabber or hook)
  if (btnRB) {
    RR_setServo2(180);
  }
  else if (btnLB) {
    RR_setServo2(0);
  }
  
}
