//Use Arduino Mega 2560 and RAMPS 1.6
//speed version
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
//12V Output--------------------------------------------------------------------------------------------
  #define output_12V  8
//Button------------------------------------------------------------------------------------------------
  #define start_button        3 // X-MIN (X-)
  #define LAP2_button         18 // Z-MIN (Z-)
  #define front_micro_switch  19 // Z-MAX (Z+)
//LEDs--------------------------------------------------------------------------------------------------
  #define start_LED_red           2 // X-MAX (X+)
  #define openloop_LED_yellow     14 // Y-MIN (Y-)
  #define follow_line_LED_green   15 // Y-MAX (Y+)
//Step Motor (TMC2208 + RAMPS 1.6 + AccelStepper)-------------------------------------------------------
  #define Rmotor_E0_STEP_PIN    26
  #define Rmotor_E0_DIR_PIN     28
  #define Rmotor_E0_ENABLE_PIN  24
  #define Lmotor_E1_STEP_PIN    36
  #define Lmotor_E1_DIR_PIN     34
  #define Lmotor_E1_ENABLE_PIN  30
  #define stepsPerRevolution 200
  #define mode 2
  #define wheel_radius 30 // (mm)
  #define car_radius 99.3 // (mm) 98
  //STEPS = stepsPerRevolution * mode * distance / (2 * PI * wheel_radius)
  double distance_to_STEPS_constant = stepsPerRevolution * mode / (2 * PI * wheel_radius);
  double steps_per_second_constant = stepsPerRevolution * mode / (2 * PI * wheel_radius);
  AccelStepper stepper_R(AccelStepper::DRIVER, Rmotor_E0_STEP_PIN, Rmotor_E0_DIR_PIN);
  AccelStepper stepper_L(AccelStepper::DRIVER, Lmotor_E1_STEP_PIN, Lmotor_E1_DIR_PIN);
  unsigned long TmWheel = 0;
//Servos------------------------------------------------------------------------------------------------
  #define servoFrontDoorPin 4
  #define servoRisePin 5
  #define servoGrabPin 6
  Servo servoFrontDoor; //door
  Servo servoRise; //arm
  Servo servoGrab; //clamp
  /*
  * door servo goes from 2380us(closed) to 620us(opened)
  * arm servo goes from 620us(lowered) to 2360us(raised)
  * clamp servo goes from 2500us(released) to 700us(grabbed)
  */
 int servoFrontDoor_close = 2380, servoFrontDoor_open = 620;
 int servoRise_down = 620, servoRise_up = 2360;
 int servoGrab_release = 2500, servoGrab_grab = 700;
//K63 digital IR sensor----------------------------------------------------------------------------------
  #define Grab_IR_sensor 32
//TCS34725 color sensor----------------------------------------------------------------------------------
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
  unsigned long Tmcolor = 0;
  int sample_per_second = 100;
  int sample_rate = 1000 / sample_per_second;
  /////////////////////Need Calibration/////////////////////
  double red_R = 150, red_G = 60, red_B = 60;
  double green_R = 63, green_G = 100, green_B = 85;
  double blue_R = 63, blue_G = 103, blue_B = 90;
  double yellow_R = 85, yellow_G = 100, yellow_B = 50;
  /////////////////////Need Calibration/////////////////////
//Line_Follow (CNY70 IR sensor * 5)----------------------------------------------------------------------
  // Analog Sensor Pins
  const byte pSen[5] = {A5, A9, A10, A11, A12}; // left to right
  // Speeds Motors Base 
  double baseSpeed = 300;
  const byte SpFWD = 260;
  const byte SpREV = 310;
  double steps_per_second_L = 0;
  double steps_per_second_R = 0;
  // CalSnX_omega
  double CalSnX_omega = 1.5;
  double CalSnX_steps_per_second = car_radius * CalSnX_omega * steps_per_second_constant;
  // Line PID constants
  float Kp = 0.22;  //0.20;
  float Ki = 0.00;  //0.00;
  float Kd = 0.00;  //0.00;
  long P=0, I=0, D=0, PIDv=0, pErr=0;
  // Sensor Position
  unsigned long PosX = 0;
  // Sensor Values
  int SenX[5];
  //////////////Need Calibration//////////////
  // Min Sensor Samples
  int MinX[5] = {398, 340, 352, 562, 524};
  // Max Sensor Samples
  int MaxX[5] = {979, 910, 972, 988, 987};
  //////////////Need Calibration//////////////
  // On Line Status
  bool detLe = false;
  // Running Status
  bool OnRun = false;
  // Timer Counters
  unsigned long Tm0 = 0;
  unsigned long Tm1 = 0;
//Constants----------------------------------------------------------------------------------------------
  double open_loop_baseSpeed = 300;
  double open_loop_baseOmega = 9;
  double speed = 0;
  long L_stepper_past_pos = 0;
  long R_stepper_past_pos = 0;
  long L_stepper_pos = 0;
  long R_stepper_pos = 0;
  long R_DIFF = 0;
  long L_DIFF = 0;
  long DIFF = 0;
//FUNCTIONS----------------------------------------------------------------------------------------------
void MAIN();
void Step_motor_move(char DIR='F', double distance=200, double Speed=100);
void Step_motor_turn(char DIR='R', double turn_degree=90, double omega=3);
void line_follow();
void CalSnX();
void EstSnX(); 
void CalPID();
void MoCTRL(double speed=300);
char get_color();
void line_follow(double speed=300);
void grab_LEGO_ball();
void green_to_red_FOLLOWING();
void drop_LEGO_ball();
void drop_ping_pong_ball();
void back_to_home();
//----------Arduino Code----------------------------------------------------------------------------------
void setup()
{ 
  Serial.begin(9600);
  //12V Output
  pinMode(output_12V, OUTPUT);
  digitalWrite(output_12V, HIGH); // !!! be careful !!!
  //Buttons
  pinMode(start_button, INPUT_PULLUP);
  pinMode(LAP2_button, INPUT_PULLUP);
  pinMode(front_micro_switch, INPUT_PULLUP);  
  //LEDs
  pinMode(start_LED_red, OUTPUT);
  pinMode(openloop_LED_yellow, OUTPUT);
  pinMode(follow_line_LED_green, OUTPUT);
  digitalWrite(openloop_LED_yellow, LOW);
  digitalWrite(follow_line_LED_green, LOW);
  //Step Motor Using TMC2208
  stepper_R.setEnablePin(Rmotor_E0_ENABLE_PIN);
  stepper_R.disableOutputs(); 
  stepper_L.setEnablePin(Lmotor_E1_ENABLE_PIN);
  stepper_L.disableOutputs(); 
  stepper_R.setAcceleration(300);
  stepper_L.setAcceleration(300);
  stepper_R.setMaxSpeed(1000);
  stepper_L.setMaxSpeed(1000);
  //Servos
  servoFrontDoor.attach(servoFrontDoorPin,620,2380);
  servoRise.attach(servoRisePin,620,2380);
  servoGrab.attach(servoGrabPin,660,2500);
  servoFrontDoor.writeMicroseconds(servoFrontDoor_close); //door initial state(closed)
  servoRise.writeMicroseconds(servoRise_down); //arm initial state(lowered)
  servoGrab.writeMicroseconds(servoGrab_release); //clamp initial state(released)
  //K63 digital IR sensor
  pinMode(Grab_IR_sensor, INPUT);
  //TCS34725 color sensor
  tcs.begin();
  //Line_Follow (CNY70 IR sensor * 5)
  for(int i=0; i<5; i++){
    pinMode(pSen[i], INPUT);
  }
  // delay(1000);
  // // Calibration Init
  // CalSnX();
  // // Calibration End
  // delay(1000);  
}

void loop()
{
  digitalWrite(start_LED_red, HIGH);
  if(digitalRead(start_button) == LOW){
    digitalWrite(start_LED_red, LOW);
    MAIN();
  }
}

//----------Function----------------------------------------------------------------------
void MAIN(){
  Step_motor_move('F', 150, open_loop_baseSpeed);
  digitalWrite(follow_line_LED_green, HIGH);
  L_stepper_past_pos = stepper_L.currentPosition(); // Reset steps count
  ///////////////////////////////////////////////////////////
  while(L_DIFF != 2500){
    if(speed != baseSpeed) speed+=0.5; //acceleration
    line_follow(speed);
    L_stepper_pos = stepper_L.currentPosition();
    L_DIFF = L_stepper_pos - L_stepper_past_pos;
  }
  //////////////////////////////////////////////////////////
  Tmcolor = millis();
  while(1){
    if(millis() - Tmcolor >= sample_rate){
      if(get_color() == 'B'){
        break;
      }
      Tmcolor = millis();
    }
    if(speed != baseSpeed) speed+=0.5; //acceleration
    line_follow(speed);
  }
  delay(500);
  digitalWrite(follow_line_LED_green, LOW);
  ///////////////////////////////////////////////
  Step_motor_move('F', 240, open_loop_baseSpeed); // Blue Floor
  Step_motor_turn('R', 90, open_loop_baseOmega);
  Step_motor_move('F', 80, open_loop_baseSpeed);
  ///////////////////////////////////////////////
  digitalWrite(follow_line_LED_green, HIGH);
  speed = 0;
  Tmcolor = millis();
  while(1){
    if(millis() - Tmcolor >= sample_rate){
      if(get_color() == 'G'){
        break;
      }
      Tmcolor = millis();
    }
    if(speed != baseSpeed) speed+=0.5; //acceleration
    line_follow(speed);
  }
  digitalWrite(follow_line_LED_green, LOW);
  delay(500);
  ////////////////// GREEN FLOOR //////////////////
  if(digitalRead(LAP2_button) == HIGH){ // NOT LAP 2
    grab_LEGO_ball(); // Green Floor
  }
  else{
    Step_motor_move('F', 350, open_loop_baseSpeed);
  }
  /////////////////////////////////////////////////
  green_to_red_FOLLOWING();
  ////////////////// RED FLOOR ////////////////////
  if(digitalRead(LAP2_button) == HIGH){ // NOT LAP 2
    drop_LEGO_ball(); // Red Floor
    drop_ping_pong_ball();
  }
  else{
    Step_motor_move('F', 300, open_loop_baseSpeed);
  }
  /////////////////////////////////////////////////
  digitalWrite(follow_line_LED_green, HIGH);
  speed = 0;
  Tmcolor = millis();
  while(1){
    if(millis() - Tmcolor >= sample_rate){
      if(get_color() == 'Y'){
        break;
      }
      Tmcolor = millis();
    }
    if(speed != baseSpeed) speed+=0.5; //acceleration
    line_follow(speed);
  }
  digitalWrite(follow_line_LED_green, LOW);
  delay(500);
  //////////////////////////////////////////////
  back_to_home(); // yellow Floor
  //////////////////////////////////////////////
}

// AccelStepper Movement /////////////////////////////////////////////////////////////////
void Step_motor_move(char DIR='F', double distance=200, double Speed=100){
  digitalWrite(openloop_LED_yellow, HIGH);
  //distance(mm), Speed(mm/s)
  double STEPS = distance * distance_to_STEPS_constant;
  double steps_per_second = Speed * steps_per_second_constant;
  stepper_R.setMaxSpeed(steps_per_second);
  stepper_L.setMaxSpeed(steps_per_second);
  if(DIR=='F'){ //Forward
    stepper_R.move(-STEPS);
    stepper_L.move(STEPS);
  }
  else if(DIR=='B'){ //Backward
    stepper_R.move(STEPS);
    stepper_L.move(-STEPS);
  }
  while(stepper_R.distanceToGo() != 0 || stepper_L.distanceToGo() != 0){
    stepper_R.run();
    stepper_L.run();
  }
  digitalWrite(openloop_LED_yellow, LOW);
}
//------------------------------------------------------------------------------------------------------
void Step_motor_turn(char DIR='R', double turn_degree=90, double omega=3){
  digitalWrite(openloop_LED_yellow, HIGH);
  //turn_degree(degrees), omega(rad/s)
  //STEPS = stepsPerRevolution * mode * distance / (2 * PI * wheel_radius)
  double distance = car_radius * turn_degree * PI / 180;//***turn_degree to distance***//
  double STEPS = distance * distance_to_STEPS_constant;
  double Speed = car_radius * omega; //***omega to Speed***//
  double steps_per_second = Speed * steps_per_second_constant;
  stepper_R.setMaxSpeed(steps_per_second);
  stepper_L.setMaxSpeed(steps_per_second);
  if(DIR=='R'){ //Right Turn
    stepper_R.move(STEPS);
    stepper_L.move(STEPS);
  }
  else if(DIR=='L'){ //Left Turn
    stepper_R.move(-STEPS);
    stepper_L.move(-STEPS);
  }
  while(stepper_R.distanceToGo() != 0 || stepper_L.distanceToGo() != 0){
    stepper_R.run();
    stepper_L.run();
  }
  digitalWrite(openloop_LED_yellow, LOW);
}

// Line Follow Functions /////////////////////////////////////////////////////////////////
void line_follow(double speed=300){
  EstSnX();
  CalPID();
  MoCTRL(speed);
}
//-------------------------------------------------------------------
void CalSnX(){ //for calibration, get max and min value of each CNY70 sensor
  stepper_L.setSpeed(CalSnX_steps_per_second);
  stepper_R.setSpeed(CalSnX_steps_per_second); 
  Tm0 = millis();
  Tm1 = Tm0;
  for(byte i=0; i<5; i++){
    SenX[i]=analogRead(pSen[i]);
    MinX[i]=SenX[i];
    MaxX[i]=SenX[i];
  }
  while((millis()-Tm0)<=10000){
    for(byte i=0; i<5; i++){
      SenX[i]=analogRead(pSen[i]);
      if(SenX[i]<MinX[i]) MinX[i]=SenX[i];
      if(SenX[i]>MaxX[i]) MaxX[i]=SenX[i];
    }
    stepper_L.runSpeed();
    stepper_R.runSpeed();
  }
/*
  for(byte i=0; i<5; i++){
    Serial.print(MinX[i]);
    Serial.print("  ");
  }
  Serial.println();
  for(byte i=0; i<5; i++){
    Serial.print(MaxX[i]);
    Serial.print("  ");
  }
*/
}
//-------------------------------------------------------------------------
void EstSnX(){ //
  unsigned long TmE = millis();
  if ((TmE-Tm0)>10){
    detLe = false;
    unsigned long avgS = 0;
    unsigned int sumS = 0;
    
    for(byte i=0; i<5; i++){
      SenX[i] = analogRead(pSen[i]);
      SenX[i] = map(SenX[i], MinX[i], MaxX[i], 1000, 0);
      SenX[i] = constrain(SenX[i], 0, 1000);
      if(SenX[i]>200)detLe = true;
      if(SenX[i]>50){
        avgS += (long)SenX[i]*(i*1000);
        sumS += SenX[i];
      }
    }
    if(detLe)PosX = avgS/sumS;
    else if(PosX < 2000)PosX = 0;
    else PosX = 4000;
    /*
    char DataX[60];
    sprintf(DataX,"%4d  %4d  %4d  %4d  %4d  %4d  ", SenX[0], SenX[1], SenX[2], SenX[3], SenX[4], PosX);
    Serial.print(DataX);Serial.print("||");
    */
    Tm0 = TmE;
  }
}
//------------------------------------------------------------------------
void CalPID(){
  P = PosX - 2000;
  I = I + P;
  D = P - pErr;
  
  PIDv = (Kp*P) + (Ki*I) + (Kd*D);
  pErr = P;
}
//-------------------------------------------------------------------------
void MoCTRL(double speed=300){
  if(detLe){
    steps_per_second_L = (speed + PIDv) * steps_per_second_constant;
    steps_per_second_R = (speed - PIDv) * steps_per_second_constant;

    steps_per_second_L=constrain(steps_per_second_L, 0, 1000);
    steps_per_second_R=constrain(steps_per_second_R, 0, 1000);
    
    stepper_L.setSpeed(steps_per_second_L);
    stepper_R.setSpeed(-steps_per_second_R); 
  }
  else{
    if(P==-2000){
      steps_per_second_L = SpREV * steps_per_second_constant;
      steps_per_second_R = SpFWD * steps_per_second_constant; 
      steps_per_second_L=constrain(steps_per_second_L, 0, 1000);
      steps_per_second_R=constrain(steps_per_second_R, 0, 1000); 
      
      stepper_L.setSpeed(-steps_per_second_L);
      stepper_R.setSpeed(-steps_per_second_R); 
    }
    else if(P==2000){
      steps_per_second_L = SpFWD * steps_per_second_constant;
      steps_per_second_R = SpREV * steps_per_second_constant;  
      steps_per_second_L=constrain(steps_per_second_L, 0, 1000);
      steps_per_second_R=constrain(steps_per_second_R, 0, 1000);
      
      stepper_L.setSpeed(steps_per_second_L);
      stepper_R.setSpeed(steps_per_second_R); 
    }
  }
/*
  char vals[28]; 
  sprintf(vals,"  L %i  R %i", MoSpL, MoSpR);
  Serial.print(P);
  Serial.println(vals);
*/
  // Serial.print(P);Serial.print("||");
  // Serial.print(baseSpeed + PIDv);Serial.print(" ");
  // Serial.print(baseSpeed - PIDv);Serial.print("||");
  // Serial.print(steps_per_second_L);Serial.print(" ");
  // Serial.print(steps_per_second_R);Serial.println(" ");
  stepper_L.runSpeed();
  stepper_R.runSpeed();
}
///////////////////////////////////////////////////////////////////////////////////
char get_color(){  //TCS34725 color sensor
  char color = 'W';
  float r, g, b;
  tcs.getRGB(&r, &g, &b);
  // Serial.print("R: "); Serial.print(r); Serial.print(" ");
  // Serial.print("G: "); Serial.print(g); Serial.print(" ");
  // Serial.print("B: "); Serial.print(b); Serial.print(" ");
  // Serial.println(" ");
  if((r >= red_R) && (g <= red_G) && (b <= red_B)){
    //Serial.println("red");
    color = 'R';
  }
  if((r <= green_R) && (g >= green_G) && (b <= green_B)){
    //Serial.println("green");
    color = 'G';
  }
  if((r <= blue_R) && (g <= blue_G) && (b >= blue_B)){
    //Serial.println("blue");
    color = 'B';
  }
  if((r >= yellow_R) && (g >= yellow_G) && (b <= yellow_B)){
    //Serial.println("yellow");
    color = 'Y';
  }
  return(color);
}
//-------------------------------------------------------------------------
void grab_LEGO_ball(){
  digitalWrite(openloop_LED_yellow, HIGH);
  double STEPS = 90 * distance_to_STEPS_constant; // distance = 90 mm
  double steps_per_second = 200 * steps_per_second_constant; // Speed = 200 mm/s
  stepper_R.setMaxSpeed(steps_per_second);
  stepper_L.setMaxSpeed(steps_per_second);
  stepper_R.move(-STEPS);
  stepper_L.move(STEPS);
  while(digitalRead(Grab_IR_sensor) != 0 && (stepper_R.distanceToGo() != 0 || stepper_L.distanceToGo() != 0)){
    stepper_R.run();
    stepper_L.run();
  }
  delay(500);
  servoGrab.writeMicroseconds(servoGrab_grab); //clamp closed, grabs the ball
  delay(500);
  servoRise.writeMicroseconds(servoRise_up); //arm raised, lifts the ball
  delay(500);
  Step_motor_move('F', 280, open_loop_baseSpeed);
}
//-------------------------------------------------------------------------
void green_to_red_FOLLOWING(){
  digitalWrite(follow_line_LED_green, HIGH);
  speed = 0;
  Tmcolor = millis();
  TmWheel = millis();
  int status1 = 0;
  while(1){
    if(millis() - TmWheel >= 1200 && status1 == 0){
      L_stepper_past_pos = stepper_L.currentPosition(); // Reset steps count
      R_stepper_past_pos = abs(stepper_R.currentPosition());
      status1++;
    }
    if(millis() - Tmcolor >= sample_rate){
      if(get_color() == 'R'){
        break;
      }
      Tmcolor = millis();
    }
    if(speed != baseSpeed) speed+=0.5; //acceleration
    line_follow(speed);
//    L_stepper_pos = stepper_L.currentPosition();
//    R_stepper_pos = abs(stepper_R.currentPosition());
//    R_DIFF = R_stepper_pos - R_stepper_past_pos;
//    L_DIFF = L_stepper_pos - L_stepper_past_pos;
//    DIFF = R_DIFF - L_DIFF;
//    Serial.print(L_DIFF);Serial.print(" ");
//    Serial.print(R_DIFF);Serial.print(" ");
//    Serial.println(DIFF);
  }
  digitalWrite(follow_line_LED_green, LOW);
  L_stepper_pos = stepper_L.currentPosition();
  R_stepper_pos = abs(stepper_R.currentPosition());
  R_DIFF = R_stepper_pos - R_stepper_past_pos;
  L_DIFF = L_stepper_pos - L_stepper_past_pos;
  DIFF = R_DIFF - L_DIFF;
  Serial.print(L_DIFF);Serial.print(" ");
  Serial.print(R_DIFF);Serial.print(" ");
  Serial.println(DIFF);
  Step_motor_move('F', 60, open_loop_baseSpeed);
  stepper_L.setMaxSpeed(open_loop_baseSpeed * steps_per_second_constant);
  stepper_L.move(DIFF);
  while(stepper_L.distanceToGo() != 0){
    stepper_L.run();
  }
  delay(500);
}
//-------------------------------------------------------------------------
void drop_LEGO_ball(){
  Step_motor_move('F', 150, open_loop_baseSpeed);
  Step_motor_turn('R', 90, open_loop_baseOmega);
  digitalWrite(openloop_LED_yellow, HIGH);
  double STEPS = 50 * distance_to_STEPS_constant; // distance = 50 mm
  double steps_per_second = 200 * steps_per_second_constant; // Speed = 200 mm/s
  stepper_R.setMaxSpeed(steps_per_second);
  stepper_L.setMaxSpeed(steps_per_second);
  stepper_R.move(-STEPS);
  stepper_L.move(STEPS);
  while(digitalRead(front_micro_switch) != 0 && (stepper_R.distanceToGo() != 0 || stepper_L.distanceToGo() != 0)){
    stepper_R.run();
    stepper_L.run();
  }
  //////////////////////////////////////////
  stepper_R.move(1);
  stepper_L.move(-1);
  while(stepper_R.distanceToGo() != 0 || stepper_L.distanceToGo() != 0){
    stepper_R.run();
    stepper_L.run();
  }
  //////////////////////////////////////////
  delay(500);
  servoGrab.writeMicroseconds(servoGrab_release); //clamp opened, releases the ball
  delay(500);
  Step_motor_move('B', 50, open_loop_baseSpeed);
  delay(500);
}
//-------------------------------------------------------------------------
void drop_ping_pong_ball(){
  Step_motor_turn('L', 180, open_loop_baseOmega);
  Step_motor_move('B', 70, open_loop_baseSpeed);
  servoFrontDoor.writeMicroseconds(servoFrontDoor_open); //door opened
  delay(2000);
  Step_motor_move('F', 100, open_loop_baseSpeed);
  Step_motor_turn('R', 100, open_loop_baseOmega);
  servoFrontDoor.writeMicroseconds(servoFrontDoor_close); //door closeed
  Step_motor_move('F', 100, open_loop_baseSpeed); // back to line
}
//-------------------------------------------------------------------------
void back_to_home(){
  Step_motor_move('F', 250, open_loop_baseSpeed);
  Step_motor_turn('L', 90, open_loop_baseOmega);
  Step_motor_move('B', 50, open_loop_baseSpeed);
  servoFrontDoor.writeMicroseconds(servoFrontDoor_close); //door initial state(closed)
  servoRise.writeMicroseconds(servoRise_down); //arm initial state(lowered)
  servoGrab.writeMicroseconds(servoGrab_release); //clamp initial state(released)
}
