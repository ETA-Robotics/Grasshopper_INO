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

struct DataPacket {
  int value1;
  int value2;
};
//Create global variables
//useful conversion
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

float motor_kp = 0.5;               //motor speed control proportionate response
float motor_ki = 0.05;             //motor speed control integral response
float motor_kd = 0.05 ;            //motor speed control differential response

float steer_kp = 0.5;              //steer angle control proportionate response (0.5 captured)
float steer_ki = 0.02;              //steer angle control integral response     (0.02 captured)
float steer_kd = 0.08;              //steer angle control differential response (0.8 captured)

//get current time in microseconds, time record (in miliseconds) from previous interrupt for differentiation and integration
long time_previous = micros(); //for motor speeds
long time_previous_anglePID= micros(); //for angle speeds 

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

void setup() {
  // Initialize Serial for Mega - RasPi
  Serial.begin(38400);
  // Initialize Serial for Mega - RS485
  Serial3.begin(38400);

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
  digitalWrite(M4_DIR, HIGH); // Set direction (HIGH or LOW)

  // Set initial motor speed
  analogWrite(M1_PWM, 0); // Initially set speed to 0
  analogWrite(M2_PWM, 0); // Initially set speed to 0
  analogWrite(M3_PWM, 0); // Initially set speed to 0
  analogWrite(M4_PWM, 0); // Initially set speed to 0
  
  // Set DE and RE pin as an output
  pinMode(DE_RE_PIN, OUTPUT);  

  // SetUp completion Message
  Serial.println("Setup Done");
  Serial.flush();
}

float command_v = 0.0; //command velocity
float command_gamma = 0; //command steer angle in degrees  
unsigned long lastSerialTime = 0;  // Variable to track the last time serial data was received
unsigned long timeout = 1000;  // Timeout duration (1 second)
bool connection_check = true;
bool motors = true;
bool calibration = true;


void loop() {
  if (Serial.available()) {
    // Reset the timer when data is received
    lastSerialTime = millis();
    connection_check = true;
    
    String data = Serial.readStringUntil('\n');
    int delimiterIndex = data.indexOf(',');
    char packet_id = data.charAt(0);
    String content = data.substring(delimiterIndex + 1);
    Serial.flush();
    
    // Process the packet based on the identifier
    switch(packet_id) {
      case 'A':
        processPacketA(content);
        break;
      case 'B':
        processPacketB(content);
        break;
      case 'C':
        processPacketC(content);
        break;
      default:
        Serial.println("Unknown packet type");
        Serial.println(packet_id);
    }
  }
  // calibration step
  if (calibration == false){
    // Run Calibration Test;
    Calibration();    
    calibration = true; 
    Serial.print("Calibration Complete");
  }

  // Check if no serial data has been received for 1 second
  if (millis() - lastSerialTime > timeout) {
    if (connection_check == true){
      Serial.println("Lack of device Input");
      connection_check = false;
      }
    command_gamma = 0;
    command_v = 0;
  }
 
  if (motors == false){
    command_gamma = 0;
    command_v = 0;
  }
  
  if (command_gamma == 0 || command_v == 0) {
    setMotorVel(1, command_v);
    setMotorVel(2, command_v);
    setMotorVel(3, command_v);
    setMotorVel(4, command_v);
  } else {
    GetEncoderInfo("front");
    angle_PID();
    setMotorVel(1, M1_vel_set);
    setMotorVel(2, M2_vel_set);
    setMotorVel(3, M3_vel_set);
    setMotorVel(4, M4_vel_set);
    Serial.println(M1_vel_set);
    Serial.println(M2_vel_set);
    //Serial.println(M3_vel_set);
    //Serial.println(M4_vel_set);
  }
  
//  // Function Outputs Encoder information (Degree:**)
//  GetEncoderInfo("rear");
}

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


// Encoder Function ------
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
// End of GetEncoderInfo


// CheckSum Function ----
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
// End of CheckSum Function


// Default Packet Process
void processPacketA(String content) {
  int delimiterIndex = content.indexOf(',');
  String value1 = content.substring(0, delimiterIndex);
  String value2 = content.substring(delimiterIndex + 1);

  // Use value1 and value2 for packet type A
  Serial.print("Packet A - Value 1: ");
  Serial.println(value1);
  Serial.print("Packet A - Value 2: ");
  Serial.println(value2);
}

float last_msg = 0;
// Velocity Packet Processing Function
void processPacketB(String content) {
  float msg = content.toFloat();  // Convert String to float
  msg = velocity_command_max * msg;  // Scale the message for max power

  // Calibration check
  if ((msg / velocity_command_max) == 4.0 && calibration == true && msg != last_msg) {
    Serial.print("cali");
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
    feedforward_update(command_v, command_gamma);
    //Serial.print("Steering:");
    //Serial.println(command_gamma);
  }
}


//MOTOR SET SPEED AND DIRECTION FUNCTION (motor id 1-4,velocity [-1:0:1])
void setMotorVel(int motor_ID,float vel){
  //Convert velocity to PWM range 0-255 with linear model of system  
  int PWM_set = round(abs(vel * 255)); // convert vel to PWM signal [0-255]
  PWM_set = constrain(PWM_set, 0,255); //Ensure PWM remains in 0-255 range
  int left_dir;  
  int right_dir; 
  // Direction Control
  if (vel < 0){
    left_dir = 0; 
    right_dir = 1;
  } else {
    left_dir = 1; 
    right_dir = 0;
  }

 //Assign settings to correct motors, note motors 2 and 4 are opposite rotation to 1 and 3, ! is used to reverse direction
   if (motor_ID == 1){
    analogWrite(M1_PWM, PWM_set);         //Motor 1 Speed
    digitalWrite(M1_DIR, left_dir);      //Motor 1 direction - use not! for opposite sides
    }
   else if(motor_ID == 2){
    analogWrite(M2_PWM, PWM_set);         //Motor 2 Speed
    digitalWrite(M2_DIR, right_dir);      //Motor 2 direction - use not! for opposite sides    
   }
   else if(motor_ID == 3){
    analogWrite(M3_PWM, PWM_set);         //Motor 3 Speed
    digitalWrite(M3_DIR, left_dir);      //Motor 3 direction - use not! for opposite sides    
   }
   else if(motor_ID == 4){
    analogWrite(M4_PWM, PWM_set);         //Motor 4 Speed
    digitalWrite(M4_DIR, right_dir);      //Motor 4 direction - use not! for opposite sides    
   }
} //End Motor set velocity

// Calibration Section
void Calibration() {
  Serial.println("Starting Calibration | Please Stand Clear");
  delay(1000); // Sleep for 10 seconds
  
  float maxFront = 0;
  float minFront = 0;
  float maxRear = 0;
  float minRear = 0;
  int MotorLeft = 0;
  int MotorRight = 0;
  const float motorSpeed = 0.05; // Speed of the motors

  
  Serial.println("Front Calibration");
  maxFront = findEncoderLimit("front", true);
  minFront = findEncoderLimit("front", false);
  float midPoint = (maxFront + minFront) / 2;
  Serial.print("Moving wheels to midpoint: ");
  Serial.println(midPoint);
  moveToEncoderValue("front", midPoint);
  SetEncoderZeroPoint("front");

  delay(1000);
  Serial.println("Rear Calibration");
  maxFront = findEncoderLimit("rear", true);
  minFront = findEncoderLimit("rear", false);
  midPoint = (maxFront + minFront) / 2;
  Serial.print("Moving wheels to midpoint: ");
  Serial.println(midPoint);
  moveToEncoderValue("rear", midPoint);
  SetEncoderZeroPoint("rear");
}

float findEncoderLimitHelper(float& encoderAngle, int motorRight, int motorLeft, bool findMax, const char* encoder) {
    Serial.println(findMax ? "Finding Maximum..." : "Finding Minimum...");
    float pastEncoder = 0;
    int endCheck = 0;
    const int waitThreshold = 40;
    float limit = 0;
    bool working = true;
    const float motorSpeed = 0.08; // Speed of the motors


    while (working) {
        setMotorVel(motorRight, findMax ? motorSpeed : -motorSpeed);
        setMotorVel(motorLeft, findMax ? -motorSpeed : motorSpeed);

        delay(10);
        GetEncoderInfo(encoder); // Updates encoderAngle
        if (encoderAngle == pastEncoder) {
            endCheck++;
            if (endCheck >= waitThreshold) {
                setMotorVel(motorRight, 0);
                setMotorVel(motorLeft, 0);
                limit = encoderAngle;
                Serial.print(findMax ? "Max limit found: " : "Min limit found: ");
                Serial.println(limit);
                working = false;
            }
        } else {
            endCheck = 0;
            pastEncoder = encoderAngle;
        }
    }
    return limit;
}

float findEncoderLimit(const char* encoder, bool findMax) {
    if (strcmp(encoder, "front") == 0) {
        return findEncoderLimitHelper(angle_front, 1, 2, findMax,encoder);
    } else if (strcmp(encoder, "rear") == 0) {
        return findEncoderLimitHelper(angle_rear, 3, 4, findMax, encoder);
    } else {
        Serial.println("Invalid encoder specified!");
        return -1;
    }
}


void moveToEncoderValue(const char* encoder, float targetValue) {
    float currentEncoderValue = 0;
    const float tolerance = 0.5;   // Allowable error margin
    int MotorLeft = 0, MotorRight = 0;
    const float motorSpeed = 0.05; // Speed of the motors
 
    // Determine motors based on encoder
    if (strcmp(encoder, "front") == 0) {
        MotorRight = 1;
        MotorLeft = 2;
    } else if (strcmp(encoder, "rear") == 0) {
        MotorRight = 3;
        MotorLeft = 4;
    } else {
        Serial.println("Invalid encoder specified.");
        return; // Exit the function if the encoder is invalid
    }

    while (true) {
        GetEncoderInfo(encoder);
        // Retrieve current encoder value
        if (strcmp(encoder, "front") == 0) {
            currentEncoderValue = angle_front; // Assuming these are global variables
        } else if (strcmp(encoder, "rear") == 0) {
            currentEncoderValue = angle_rear;
        }

        float error = targetValue - currentEncoderValue;
        Serial.println("Error: " + String(error));

        // Check if the position is within tolerance
        if (fabs(error) <= tolerance) {
            setMotorVel(MotorRight, 0); // Stop the motors
            setMotorVel(MotorLeft, 0);
            Serial.println("Wheels positioned at target value.");
            break;
        }

        // Adjust motor speeds for turning
      float rightSpeedAdjustment = (error < 0) ? motorSpeed : -motorSpeed;
      float leftSpeedAdjustment = (error < 0) ? -motorSpeed : motorSpeed;
      
      setMotorVel(MotorRight, rightSpeedAdjustment);
      setMotorVel(MotorLeft, leftSpeedAdjustment);


        delay(10); // Small delay for motor adjustment
    }
}









//Calculate feedforward setpoints from input steering angle and radius
void feedforward_update(float v, float gamma){
  gamma = gamma * deg2rad;
  //safetly limit - ensure target angle (gamma) remains within the steering limits
  //gamma = constrain(gamma, -steer_angle_max, steer_angle_max);
  //Serial.print("gamma:");
  //Serial.println(gamma);
 
  //update feedforward angle following saturation limit checks
  angle_ff = gamma/2; //feedforward angle is half the steer angle 
  //Serial.print("angle_ff:");
  //Serial.println(angle_ff);
  
  //safetly limit - ensure target velocity (v) remains within the command vel limits
  //v = constrain(v, -velocity_command_max, velocity_command_max);
  //  Serial.print("v:");
  //  Serial.println(v);
  
  //Condition if angle gamma set to zero, do not perform calculations and set all velocities to input velocity (straight ahead)
  if (gamma == 0){
    M1_vel_ff = v;
    M2_vel_ff = v;
    M3_vel_ff = v;
    M4_vel_ff = v;
    }//end if

   //feedforward calculations can proceed under this condition 
   else {
   //update motor velocities
   float r = L / (2*tan(gamma/2))     ;//calculate robot steer radius
   float r_axel = L / (2*sin(gamma/2)) ;//calculate axel steer radius (r_axel) 
   float r_L = r_axel - 0.5*B ;//calculate left arc radius (r_L) 
   float r_R = r_axel + 0.5*B ;//calculate left arc radius (r_L)
   float omega = v/r; //robot rotation speed around Instantaneous Centre of Rotation (ICR)
   // calculate requred motor velocities
   float vL = omega*r_L; //left wheel velocities
   float vR = omega*r_R; //right wheel velocities
   // Change velocity to factor in [-1:1]
   // vL = (vL * 2) - 1;
   // vR = (vR * 2) - 1;   
   //Serial.print("vL:");
   //Serial.println(vL);
   //Serial.print("vR:");
   //Serial.println(vR);
   M1_vel_ff = vR;
   M2_vel_ff = vL;
   M3_vel_ff = vR;
   M4_vel_ff = vL;
   }//end else for V==0
  }//end feedforward update
 // END of Fucntion

 
////ANGLE ERROR PID CONTROLLER AND SUMMATION OF FEEDFORWARD
void angle_PID(){
    float dt; //change in time
    float e; //velocity error
    float dedt; //derivative of error
    float fb; //feedback term from controller
  
    //Time update - calculate tie since last poll
    float time_current = micros(); //get current time in microseconds
    dt = ((float)(time_current - time_previous_anglePID))/1.0e6; //calculate delta time and convert to seconds
    time_previous_anglePID = time_current;  //update previous time in record to current time

    //FRONT steer angle S01 sensor(motors M01 & M02) - note left motor is M02
    Serial.print("angle:");
    Serial.println(angle_front);
    angle_front = angle_front* deg2rad;
    Serial.println(angle_front);
    e = angle_ff - angle_front;                //calculate error from feed forward angle and feedback angle
    dedt = (e - angle_front_error_previous)/dt;  // calculate error derivative
    angle_front_error_integral += e*dt;          //update integration for time step
    fb = steer_kp*e + steer_kd*dedt + steer_ki*angle_front_error_integral;  //PID Controller
//  Serial.print("fb:");
//  Serial.print(fb);
//  Serial.print("M1_vel_set:");
//  Serial.print(M1_vel_set);
    M2_vel_set = M2_vel_ff - fb; //left motor feedback, slows with positive angle error
    M1_vel_set = M1_vel_ff + fb; //lright motor feedback, speeds with positive angle error

    
    //REAR steer angle S02 sensor(motors M03 & M04) - note left motor is M04, negative angle_ff due to rear facing
    e = -angle_ff - angle_rear;                //calculate error from feed forward angle and feedback angle
    dedt = (e - angle_front_error_previous)/dt;  // calculate error derivative
    angle_front_error_integral += e*dt;          //update integration for time step
    fb = steer_kp*e + steer_kd*dedt + steer_ki*angle_front_error_integral;  //PID Controller 
    M4_vel_set = M4_vel_ff - fb; //left motor feedback, slows with positive angle error
    M3_vel_set = M3_vel_ff + fb; //lright motor feedback, speeds with positive angle error
} //end steer angle PID


// CODE GRAVEYARD / WAITING ROOM -----------------------------------------------------------------------------------------------------------------------------------------------
