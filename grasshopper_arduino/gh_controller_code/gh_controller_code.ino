/* 
Code information &/ legal
*/

// Motor 1 pins
#define M1_PWM 4  // PWM Speed
#define M1_DIR 5  // Direction
#define M1_ENC 18 // Encoder feedback
//Motor 2 pins
#define M2_PWM 6  // PWM Speed
#define M2_DIR 7  // Direction
#define M2_ENC 19 // Encoder feedback
//Motor 3 pins
#define M3_PWM 8  // PWM Speed
#define M3_DIR 9  // Direction
#define M3_ENC 20 // Encoder feedback
//Motor 4 pins
#define M4_PWM 10  // PWM Speed
#define M4_DIR 11  // Direction
#define M4_ENC 21 // Encoder feedback

// Define RS485 module control pins
#define DE_RE_PIN 22  // DE and RE tied together for simplified control

//Create global variables
float deg2rad = 0.0174533;  //radian per degree (0.0174533rad/degree)

//robot dimensions
float B = 0.4;//robot base length between steer pivot points [m]
float L = 0.42; //robot wheelbase (width) [m]

//Control Feedback
//float pulse_to_dist = 0.0001122; //distance in meters per pulse on encoder (mixed signal rise and fall, 64 pulses per motor rev, 0.16m dia tires
float motor_velocity_max = 1.0;   //[m/s] motor / wheel maximum output velocity saturation limit (maximum no-load velocity is 1.25m/s)
float velocity_command_max = 0.5*motor_velocity_max;   //maximum command velocity, overhead is permitted for control to correct
float motor_current_max = 2.0;   //[A] motor / wheel maximum output velocity saturation limit (maximum no-load velocity is 1.25m/s) ---------------------------------TODO CURRENT LIMIT!
float steer_angle_max = 45*deg2rad; //maximum allowed steering angle

float steer_kp = 0.5;              //steer angle control proportionate response (0.5 captured)
float steer_ki = 0.000;              //steer angle control integral response     (0.02 captured)
float steer_kd = 0.00;              //steer angle control differential response (0.8 captured)

//get current time in microseconds, time record (in miliseconds) from previous interrupt for differentiation and integration
long time_previous = micros(); //for motor speeds
long time_previous_anglePID_front= micros(); //for angle speeds 
long time_previous_anglePID_rear= micros(); //for angle speeds 

//Variables for steer angle error feedback intergration and differentiation ---------------------------------TODO
float angle_front_error_previous = 0;       //record of previous error signal
float angle_front_error_integral = 0;       //integral storage variable
float angle_rear_error_previous = 0;       //record of previous error signal
float angle_rear_error_integral = 0;       //integral storage variable
float angle = 0;                    // placeholder 
float angle_front = 0;              //angle read from front Encoder
float angle_rear = 0;              //angle read from rear Encoder
float angle_ff = 0;              //set angle from feedforward

//Variables for motor velocity setpoints [m/s] -- this is the sum of feedforward and steer feedback
float M1_vel_set = 0; //motor 1
float M2_vel_set = 0; //motor 2
float M3_vel_set = 0; //motor 3
float M4_vel_set = 0; //motor 4

//Variables for motor velocity feedfoward setpoints [m/s] -- this is the  feedforward setpoints
float M1_vel_ff = 0; //motor 1
float M2_vel_ff = 0; //motor 2
float M3_vel_ff = 0; //motor 3
float M4_vel_ff = 0; //motor 4

// Control commands
float command_vel = 0.0; //command velocity
float command_gamma = 0; //command steer angle in degrees  
float last_command_vel = 0.0; //command velocity
float last_command_gamma = 0; //command steer angle in degrees  
unsigned long lastSerialTime = 0;  // Variable to track the last time serial data was received
unsigned long dropoutTimer = 0;
unsigned long timeout = 1000;  // Timeout duration (1 second)
bool connection_check = true;
bool motors = true;
bool calibration = true;
bool Debug = false;
bool cmd_stop = false;
bool cmd_cal = false;

// Commuincation variables
// Define buffer to hold the incoming string
char cmd_string[64];  // Adjust size as necessary
// Array to store arguments after splitting the string
char *argv[4]; // Expecting 4 arguments (cmd_vel, cmd_steer, cmd_stop, cmd_cal)
int arg = 0;    // To keep track of the current argument
int index = 0;  // To track the index of the current argument

// Calibration Variables
float maxFront = 0;
float minFront = 0;
float maxRear = 0;
float minRear = 0;
int MotorLeft = 0;
int MotorRight = 0;
const float motorSpeed = 0.05; 
float currentEncoderValue = 0;
const float tolerance = 0.5;   

// PID Variables
float dt; //change in time
float velocity_error; //velocity error
float dedt; //derivative of error
float controller_feedback; //feedback term from controller

// Define states
typedef enum {
    STATE_TIMEOUT,
    STATE_CALIBRATION,
    STATE_NORMAL_OPERATION
} SystemState;

SystemState currentState = STATE_NORMAL_OPERATION;  // Initial state


void setup() {
  // Initialize the motor control pins
  pinMode(M1_PWM, OUTPUT); // Set PWM pin as output
  pinMode(M1_DIR, OUTPUT); // Set direction pin as output
  pinMode(M1_ENC, INPUT);  // Set encoder pin as input

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_ENC, INPUT);

  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_ENC, INPUT);

  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_ENC, INPUT);

  // Set initial motor direction
  digitalWrite(M1_DIR, HIGH); // Set direction (HIGH or LOW)
  digitalWrite(M2_DIR, HIGH); // Set direction (HIGH or LOW)
  digitalWrite(M3_DIR, HIGH); // Set direction (HIGH or LOW)
  digitalWrite(M4_DIR, HIGH); // Set direction (HIGH or LOW)

  // Set initial motor speed
  analogWrite(M1_PWM, 0); // Initially set speed to 0
  analogWrite(M2_PWM, 0); // Initially set speed to 0
  analogWrite(M3_PWM, 0); // Initially set speed to 0
  analogWrite(M4_PWM, 0); // Initially set speed to 0

    // Initialize Serial for Mega - RasPi
  Serial.begin(57600);
  // Initialize Serial for Mega - RS485
  Serial3.begin(38400);
  
  // Set DE and RE pin as an output
  pinMode(DE_RE_PIN, OUTPUT);  

  // SetUp completion Message
  Serial.println("Setup Done");
  Serial.flush();
}


void loop() {
  dropoutTimer = millis();
  communication();
  
  if (dropoutTimer - lastSerialTime > timeout) {
      currentState = STATE_TIMEOUT;  // Transition to timeout state
  }
   else if (!calibration) {
      currentState = STATE_CALIBRATION;  // Transition to calibration state
  } else{
    currentState = STATE_NORMAL_OPERATION;
  }

  // State-specific behavior 
  switch (currentState) {
      case STATE_TIMEOUT:
          handleTimeout();
          break;

      case STATE_CALIBRATION:
          setAllMotorVelocities(0.0);
          delay(1000); 
          calibrateSteering("front");
          delay(1000);
          calibrateSteering("rear");
          calibration = true;
          break;

      case STATE_NORMAL_OPERATION:
          processNormalOperation();
          break;
  }
}

// Function to handle commuincation     
void communication() {
    char cmd_string[100];       // Temporary buffer for reading the command
    char argv[4][25];           // Array of strings for arguments (4 arguments, each up to 25 characters)
    int arg = 0;                // Argument counter
    int index = 0;              // Index for cmd_string

    while (Serial.available()) {
        // Reset the timer when data is received
        lastSerialTime = millis();
        connection_check = true;

        // Read the next character
        char chr = Serial.read();

        // Terminate the command with a carriage return
        if (chr == '\r') { // 13 corresponds to '\r' (Carriage Return)
            // Null-terminate and save the last argument
            if (arg < 4 && index > 0) {
                cmd_string[index] = '\0';
                strcpy(argv[arg], cmd_string); // Copy cmd_string to argv[arg]
                arg++;
            }

            // Print received arguments for debugging
            for (int i = 0; i < arg; i++) {
                Serial.print("Arg ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(argv[i]);
            }

            // Call the function to run the command
            runCommand(argv);

            // Reset counters for the next command
            arg = 0;
            index = 0;
        } 
        else if (chr == ',') { // Use commas to delimit parts of the command
            if (arg < 4) {
                cmd_string[index] = '\0';  // Null-terminate the current argument
                strcpy(argv[arg], cmd_string); // Copy cmd_string to argv[arg]
                arg++;
                index = 0; // Reset index for the next argument
            }
        } 
        else {
            // Append the character to cmd_string
            if (index < sizeof(cmd_string) - 1) {
                cmd_string[index++] = chr;
            }
        }
    }
}


// Function to process the command once it is received
void runCommand(char argv[4][25]) {
    float command_vel = atof(argv[0]);      // Convert first argument to float
    float command_gamma = atof(argv[1]);   // Convert second argument to float
    String cmd_stop = argv[2];
    String cmd_cal = argv[3];

    // Your logic to handle the commands
      Serial.print("State,");
      Serial.print(command_vel);
      Serial.print(",");
      Serial.print(command_gamma);
      Serial.print(",");
      Serial.print(cmd_stop);
      Serial.print(",");
      Serial.print(cmd_cal);
      Serial.print(",");
      Serial.print(M3_vel_set);
      Serial.print(",");
      Serial.print(M4_vel_set);
      Serial.println("\r");
      Serial.flush(); // Ensure the message is sent


}


// Reset the command buffer for the next input
void resetCommand() {
    for (int i = 0; i < 4; i++) {
        argv[i] = NULL;
    }
}


// Function to handle timeout scenario
void handleTimeout() {
  if (connection_check) {
    Serial.println("Lack of device Input");
    connection_check = false;  // Only print once until reset
  }
  // Set all motors to idle (optional: could ramp down instead)
  setAllMotorVelocities(0);  
}


// Function to process normal operation (motor control)
void processNormalOperation() {
    feedforward_update(command_vel, command_gamma);  // Update control parameters
    GetEncoderInfo("front");  // Get front encoder data
    GetEncoderInfo("rear");   // Get rear encoder data
    front_angle_PID();  
    rear_angle_PID();     
  
    // Update motor velocities based on PID output
    setMotorVelocities(M1_vel_set, M2_vel_set, M3_vel_set, M4_vel_set);
}
  

// Function to set all motor velocities at once
void setAllMotorVelocities(float velocity) {
  setMotorVel(1, velocity);
  setMotorVel(2, velocity);
  setMotorVel(3, velocity);
  setMotorVel(4, velocity);
}
  

// Function to set individual motor velocities
void setMotorVelocities(float M1, float M2, float M3, float M4) {
  setMotorVel(1, M1);
  setMotorVel(2, M2);
  setMotorVel(3, M3);
  setMotorVel(4, M4);
}


// Function to set Steering Encoders center points
void SetEncoderZeroPoint(String encoder) {
  // Clear the RS485 buffer
  while (Serial3.available()) Serial3.read();
  
  // Set RS485 transceiver to transmit mode
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(10);

  // Define the command to send based on the encoder type
  byte command;
  if (encoder == "rear") {
    command = 0x24; // Command for the rear encoder
  } else if (encoder == "front") {
    command = 0x54; // Command for the front encoder
  } else {
    // Handle invalid encoder input
    Serial.println("Invalid encoder specified");
    return;
  }

  // Add the offset to the command and send it
  Serial3.write(command + 2); // Add offset to the base command
  Serial3.write(0x5E);        // Send the second byte of the command

  // Add a delay to allow the transmission to complete
  delay(5);

  // Return RS485 transceiver to receive mode
  digitalWrite(DE_RE_PIN, LOW);
}


// Function to gte steering encoder information 
void GetEncoderInfo(String encoder) {
  // Clear the RS485 buffer
  while (Serial3.available()) Serial3.read();
  
  // Set RS485 transceiver to transmit mode
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(10);

  if (encoder == "rear") {
    // 0x24: Command for the front encoder
    Serial3.write(0x24);
  } 
  else if (encoder == "front") {  
    // 0x54: Command for the rear encoder
    Serial3.write(0x54);
  } 
  else {
    // Handle invalid encoder input
    Serial.println("Invalid encoder specified");
    return;
  }
  
  Serial3.flush();  
  
  digitalWrite(DE_RE_PIN, LOW);
  delay(10);

  int bytes_received = Serial3.available();
  if (bytes_received == 2) {
      uint16_t currentPosition = Serial3.read(); //low byte comes first
      currentPosition |= Serial3.read() << 8;    //high byte next, OR it into our 16 bit holder but get the high bit into the proper placeholder
 
    if (verifyChecksumRS485(currentPosition))
      {
        //we got back a good position, so just mask away the checkbits
        currentPosition &= 0x3FFF;
        currentPosition = currentPosition * 0.02197265625;
        angle = currentPosition;
        if (angle > 300){
         angle = angle - 360;
          }
        //Serial.print("Degree:");
        //Serial.println(angle);
        if(encoder =="front"){
          angle_front = angle;
        } else if( encoder == "rear"){
          angle_rear = angle;
        }
      }
      else
      {
        Serial.println(" error: Invalid checksum.");
      }
    }
    else
    {
      Serial.print(" error: Expected to receive 2 bytes. Actually received ");
      Serial.print(bytes_received, DEC);
      Serial.println(" bytes.");
    }

  //flush the received serial buffer just in case anything extra got in there
  while (Serial3.available()) Serial3.read(); 
  }


// Function to verify RS485 steering encoder checksum
bool verifyChecksumRS485(uint16_t message)
{
  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent.
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}


//MOTOR SET SPEED AND DIRECTION FUNCTION (motor id 1-4,velocity [-1:0:1])
void setMotorVel(int motor_ID, float vel) {
  // Check for Motor emergency stop, disable motors if active
  if (motors == false) {
    vel = 0;  // Stop motor if emergency stop is activated   
  }

  // Convert velocity to PWM range 0-255 with a linear model of the system
  int PWM_set = round(abs(vel) * 255);  // Convert vel to PWM signal [0-255]
  PWM_set = constrain(PWM_set, 0, 255); // Ensure PWM remains in the 0-255 range

  // Direction Control: set 1 for forward, 0 for backward
  int vel_pos = (vel > 0) ? 1 : 0;

  // Assign settings to the correct motors
  // Motors 2 and 4 are opposite in direction to 1 and 3, so use ! for reversal
  if (motor_ID == 1) {
    analogWrite(M1_PWM, PWM_set);         // Motor 1 Speed
    digitalWrite(M1_DIR, !vel_pos);      // Motor 1 direction - reverse for opposite sides
  }
  else if (motor_ID == 2) {
    analogWrite(M2_PWM, PWM_set);         // Motor 2 Speed
    digitalWrite(M2_DIR, vel_pos);       // Motor 2 direction - normal
  }
  else if (motor_ID == 3) {
    analogWrite(M3_PWM, PWM_set);         // Motor 3 Speed
    digitalWrite(M3_DIR, !vel_pos);      // Motor 3 direction - reverse for opposite sides
  }
  else if (motor_ID == 4) {
    analogWrite(M4_PWM, PWM_set);         // Motor 4 Speed
    digitalWrite(M4_DIR, vel_pos);       // Motor 4 direction - normal
  }
} // End of Motor set velocity


// Calibration Section
void calibrateSteering(const char* position) {
  int maxVal = findEncoderLimit(position, true);
  int minVal = findEncoderLimit(position, false);
  float midPoint = (maxVal + minVal) / 2;  
  
  moveToEncoderValue(position, midPoint);
  SetEncoderZeroPoint(position);
}


// Fucntion to assist findEncoder Limit
float findEncoderLimitHelper(float& encoderAngle, int motorRight, int motorLeft, bool findMax, const char* encoder) {
  float pastEncoder = 0;
  int endCheck = 0;
  const int waitThreshold = 40;
  float limit = 0;
  bool working = true;
  const float motorSpeed = 0.08; 

  // Set motor speed and direction 
  while (working) {
      setMotorVel(motorRight, findMax ? motorSpeed : -motorSpeed);
      setMotorVel(motorLeft, findMax ? -motorSpeed : motorSpeed);
      delay(10);
      GetEncoderInfo(encoder); 

      if (encoderAngle == pastEncoder) {
        endCheck++;
        if (endCheck >= waitThreshold) {
          setMotorVel(motorRight, 0);
          setMotorVel(motorLeft, 0);
          limit = encoderAngle;
          working = false;
        }
      } else {
        endCheck = 0;
        pastEncoder = encoderAngle;
      }
  }
  return limit;
}


// Function to find encoder limit
float findEncoderLimit(const char* encoder, bool findMax) {
    if (strcmp(encoder, "front") == 0) {
        return findEncoderLimitHelper(angle_front, 2, 1, findMax,encoder);
    } else if (strcmp(encoder, "rear") == 0) {
        return findEncoderLimitHelper(angle_rear, 4, 3, findMax, encoder);
    } else {
        Serial.println("Invalid encoder specified!");
        return -1;
    }
}


// Function to drive wheels such that steering is aligned 
void moveToEncoderValue(const char* encoder, float targetValue) {

  // Determine motors based on encoder
  if (strcmp(encoder, "front") == 0) {
      MotorRight = 2;
      MotorLeft = 1;
  } else if (strcmp(encoder, "rear") == 0) {
      MotorRight = 4;
      MotorLeft = 3;
  } else {
      Serial.println("Invalid encoder specified.");
      return; // Exit the function if the encoder is invalid
  }

  while (true) {
    GetEncoderInfo(encoder);
    // Retrieve current encoder value
    if (strcmp(encoder, "front") == 0) {
        currentEncoderValue = angle_front; 
    } else if (strcmp(encoder, "rear") == 0) {
        currentEncoderValue = angle_rear;
    }

    float error = targetValue - currentEncoderValue;

    if (!Debug){
      Serial.println("Error: " + String(error));
    }

    // Check if the position is within tolerance
    if (fabs(error) <= tolerance) {
        setMotorVel(MotorRight, 0); // Stop the motors
        setMotorVel(MotorLeft, 0);
        break;
    }

    // Adjust motor speeds for turning
    float rightSpeedAdjustment = (error < 0) ? motorSpeed : -motorSpeed;
    float leftSpeedAdjustment = (error < 0) ? -motorSpeed : motorSpeed;    
    setMotorVel(MotorRight, rightSpeedAdjustment);
    setMotorVel(MotorLeft, leftSpeedAdjustment);
    delay(10); 
  }
}


//Calculate feedforward setpoints from input steering angle and radius
// cmd_vel [-1:0:1] cmd_gamma [-30:0:30] *degrees
void feedforward_update(float cmd_vel, float cmd_gamma){
  
  cmd_gamma = cmd_gamma * deg2rad;

  //safetly limit - ensure target angle (gamma) remains within the steering limits ** Could be move into commuincation like vel 
  cmd_gamma = constrain(cmd_gamma, -steer_angle_max, steer_angle_max);
 
  //update feedforward angle following saturation limit checks
  angle_ff = cmd_gamma/2; //feedforward angle is half the steer angle 
  
  //Condition if angle gamma set to zero, do not perform calculations and set all velocities to input velocity (straight ahead)
  if (cmd_gamma == 0){
    M1_vel_ff = cmd_vel;
    M2_vel_ff = cmd_vel;
    M3_vel_ff = cmd_vel;
    M4_vel_ff = cmd_vel;
  } 

  else {
    //feedforward calculations  
    float steer_radius = L / (2*tan(cmd_gamma/2))     ;    
    float axel_steer_radius = L / (2*sin(cmd_gamma/2)) ;   
    float left_arc_radius= axel_steer_radius - 0.5*B ;     
    float right_arc_radius = axel_steer_radius + 0.5*B ;   
    float omega = cmd_vel/steer_radius;                    //robot rotation speed around Instantaneous Centre of Rotation (ICR)

    // calculate requred motor velocities
    float left_wheel_vel = omega*left_arc_radius; 
    float right_wheel_vel = omega*right_arc_radius; 
    M1_vel_ff = right_wheel_vel;
    M2_vel_ff = left_wheel_vel;
    M3_vel_ff = right_wheel_vel;
    M4_vel_ff = left_wheel_vel;
    }
}

 
//Function for Front ANGLE ERROR PID CONTROLLER AND SUMMATION OF FEEDFORWARD
void front_angle_PID(){
    //Time update - calculate tie since last poll
    float time_current = micros(); //get current time in microseconds
    dt = ((float)(time_current - time_previous_anglePID_front))/1.0e6; //calculate delta time and convert to seconds
    time_previous_anglePID_front = time_current;  //update previous time in record to current time

    //FRONT steer angle S01 sensor(motors M01 & M02) - note left motor is M02
    angle_front = angle_front* deg2rad;
    velocity_error = angle_ff - angle_front;                //calculate error from feed forward angle and feedback angle
    dedt = (velocity_error - angle_front_error_previous)/dt;  // calculate error derivative
    angle_front_error_integral += velocity_error*dt;          //update integration for time step

    //PID Controller
    controller_feedback = 
      steer_kp*velocity_error + 
      steer_ki*angle_front_error_integral + 
      steer_kd*dedt;  

    M2_vel_set = M2_vel_ff - controller_feedback; //left motor feedback, slows with positive angle error
    M1_vel_set = M1_vel_ff + controller_feedback; //lright motor feedback, speeds with positive angle error
}


//Function for Rear ANGLE ERROR PID CONTROLLER AND SUMMATION OF FEEDFORWARD
void rear_angle_PID(){  
    //Time update - calculate tie since last poll
    float time_current = micros(); //get current time in microseconds
    dt = ((float)(time_current - time_previous_anglePID_rear))/1.0e6; //calculate delta time and convert to seconds
    time_previous_anglePID_rear = time_current;  //update previous time in record to current time
    
    //REAR steer angle S02 sensor(motors M03 & M04) - note left motor is M04, negative angle_ff due to rear facing
    angle_rear = angle_rear* deg2rad;
    velocity_error = -angle_ff - angle_rear;                //calculate error from feed forward angle and feedback angle
    dedt = (velocity_error - angle_rear_error_previous)/dt;  // calculate error derivative
    angle_rear_error_integral += velocity_error*dt;          //update integration for time step

    //PID Controller
    controller_feedback = 
      steer_kp*velocity_error + 
      steer_ki*angle_rear_error_integral + 
      steer_kd*dedt; 

    M4_vel_set = M4_vel_ff - controller_feedback; //left motor feedback, slows with positive angle error
    M3_vel_set = M3_vel_ff + controller_feedback; //lright motor feedback, speeds with positive angle error
} 


// CODE GRAVEYARD / WAITING ROOM -----------------------------------------------------------------------------------------------------------------------------------------------

// Old Functions for commuincation
/*// Velocity Packet Processing Function
void processPacketB(String content) {
  float msg = content.toFloat();  // Convert String to float
  msg = velocity_command_max * msg;  // Scale the message for max power
  
  // Calibration check
  if ((msg / velocity_command_max) == 4.0 && calibration == true && msg != last_msg) {
    last_msg = msg;  // Update last_msg after calibration
    calibration = false;
    return;
  }
  last_msg = msg;
  
  // Brake engagement check
  if ((msg / velocity_command_max) == 2.0 && motors == true) {    
    Serial.println("Brake Engaged");
    motors = false;  // Disable motors
    
    return;
  }

  // Brake release check when velocity is 0 and motors are off
  else if (msg == 0 && !motors) {
    motors = true;  // Engage motors
    Serial.println("Brake released");
    return;
  }

  // If the new command is the same as the current one, do nothing
  else if (command_v == msg) {
    return;
  }

  // Otherwise, update the velocity command
  else {
    command_v = msg;
//    Serial.print("Velocity: ");
//    Serial.println(msg);
    return;
  }
}

// Steering Packet Processing Function
void processPacketC(String content) {
  float msg = content.toFloat();  // Convert String to float
  msg = msg *45;
  if (command_gamma == msg) {
    // Do nothing if the message is the same as the current command_v
    return;
  } else {
     // Convert Steering input into gamma
    // Steering 45 Degress == 1.0
    // Left Positve, Right Negative [1:-1]    
    command_gamma = msg;
    //Serial.print("Steering:");
    //Serial.println(command_gamma);
  }
}
*/

// void Calibration(){
//     Serial.println("Front Calibration");
//   maxFront = findEncoderLimit("front", true);
//   minFront = findEncoderLimit("front", false);
//   float midPoint = (maxFront + minFront) / 2;
//   Serial.print("Moving wheels to midpoint: ");
//   Serial.println(midPoint);
//   moveToEncoderValue("front", midPoint);
//   SetEncoderZeroPoint("front");

//   delay(1000);
//   Serial.println("Rear Calibration");
//   maxFront = findEncoderLimit("rear", true);
//   minFront = findEncoderLimit("rear", false);
//   midPoint = (maxFront + minFront) / 2;
//   Serial.print("Moving wheels to midpoint: ");
//   Serial.println(midPoint);
//   moveToEncoderValue("rear", midPoint);
//   SetEncoderZeroPoint("rear");
// }
