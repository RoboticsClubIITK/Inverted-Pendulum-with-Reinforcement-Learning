// Encoder pins
const int motorEncoderPinA = 21;
const int motorEncoderPinB = 20;
const int angleEncoderPinA = 2;
const int angleEncoderPinB = 3;

// Motor control pins
const int motorPWMPin = 6;
const int motorIN1 = 5;
const int motorIN2 = 4;
const int motorEnablePin = 7;

// Encoder positions
volatile long motorPosition = 0;
volatile long angleStepCount = 0;
volatile byte motorLastEncoded = 0;
volatile byte angleLastEncoded = 0;

// Other variables
unsigned long lastTime = 0;
long lastMotorPosition = 0;
long lastAngleStepCount = 0;
float motorSpeed = 0;
float angularSpeed = 0;
bool usePositionControl = true;
long desiredPosition = 0;
const int fixedVoltage = 100;
const int positionTolerance = 100;
int voltageOutput = 0;

// Angle calculation constants
const float encoderStepsPerRevolution = 1440; // Adjusted for 4x PPR (400 * 4)
const float gearRatio = 1.0;

// Variables for smoothing
const int bufferSize = 10;
unsigned long timeBuffer[bufferSize];
long motorPositionBuffer[bufferSize];
long angleStepCountBuffer[bufferSize];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(motorEncoderPinA, INPUT_PULLUP);
  pinMode(motorEncoderPinB, INPUT_PULLUP);
  pinMode(angleEncoderPinA, INPUT_PULLUP);
  pinMode(angleEncoderPinB, INPUT_PULLUP);
  
  pinMode(motorPWMPin, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  pinMode(motorEnablePin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(motorEncoderPinA), readMotorEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorEncoderPinB), readMotorEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(angleEncoderPinA), readAngleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(angleEncoderPinB), readAngleEncoder, CHANGE);

  digitalWrite(motorEnablePin, LOW); // Initially disable the motor
  
  // Initialize buffers
  unsigned long currentTime = micros();
  for (int j = 0; j < bufferSize; j++) {
    timeBuffer[j] = currentTime;
    motorPositionBuffer[j] = motorPosition;
    angleStepCountBuffer[j] = angleStepCount;
  }
  
  Serial.println("Inverted Pendulum System Ready");
}

void loop() {
  // i++;
  unsigned long currentTime = micros();

  // Update buffers
  bufferIndex = (bufferIndex + 1) % bufferSize;
  timeBuffer[bufferIndex] = currentTime;
  motorPositionBuffer[bufferIndex] = motorPosition;
  angleStepCountBuffer[bufferIndex] = angleStepCount;
  
  // Calculate smoothed speeds
  int oldestIndex = (bufferIndex + 1) % bufferSize;
  float deltaTime = (currentTime - timeBuffer[oldestIndex]) / 1000000.0;  // Convert to seconds
  motorSpeed = (motorPosition - motorPositionBuffer[oldestIndex]) / deltaTime;
  angularSpeed = (angleStepCount - angleStepCountBuffer[oldestIndex]) * (360.0 / encoderStepsPerRevolution) / gearRatio / deltaTime;

  // Calculate angle from vertical (0 degrees is vertical, positive is clockwise)
  float angle = calculateAngle(angleStepCount);

  // Handle serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.charAt(0) == 'R') { // Read request
      sendData(motorPosition, angle, motorSpeed, angularSpeed);
    } else {
      parseCommand(input);
    }
  }

  // Motor control
  int output;
  if (usePositionControl) {
    long positionError = desiredPosition - motorPosition;
    if (abs(positionError) <= positionTolerance) {
      output = 0;
    } else if (positionError > 0) {
      output = fixedVoltage;
    } else {
      output = -fixedVoltage;
    }
  } else {
    output = voltageOutput;
    
  }
  if(abs(motorPosition) > 5000){
      if (motorPosition <= 0){
        output = 170;
      }else{
        output = -170;
      }
    }
  // Set motor direction and speed
  setMotor(output);

  // sendData(motorPosition, angle, motorSpeed, angularSpeed);
  delay(1);
}

void readMotorEncoder() {
  byte MSB = digitalRead(motorEncoderPinA);
  byte LSB = digitalRead(motorEncoderPinB);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (motorLastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) motorPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) motorPosition--;

  motorLastEncoded = encoded;
}

void readAngleEncoder() {
  byte MSB = digitalRead(angleEncoderPinA);
  byte LSB = digitalRead(angleEncoderPinB);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (angleLastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) angleStepCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) angleStepCount--;

  angleLastEncoded = encoded;
}

float calculateAngle(long stepCount) {
  float rawAngle = (stepCount / encoderStepsPerRevolution) * 360.0 / gearRatio;
  rawAngle = fmod(rawAngle, 360.0); // Normalize to 0-360 range
  if (rawAngle < 0) rawAngle += 360.0; // Ensure positive angle
  
  // Adjust the angle so that the downward position is 180 degrees
  float adjustedAngle = rawAngle + 180.0;
  if (adjustedAngle >= 360.0) adjustedAngle -= 360.0;
  
  // Convert to -180 to 180 range
  if (adjustedAngle > 180.0) adjustedAngle -= 360.0;
  
  return adjustedAngle;
}

void setMotor(int output) {
  if (output != 0) {
    digitalWrite(motorEnablePin, HIGH); // Enable the motor
    if (output > 0) {
      digitalWrite(motorIN1, HIGH);
      digitalWrite(motorIN2, LOW);
    } else {
      digitalWrite(motorIN1, LOW);
      digitalWrite(motorIN2, HIGH);
    }
    analogWrite(motorPWMPin, abs(output));
  } else {
    digitalWrite(motorEnablePin, LOW); // Disable the motor
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorPWMPin, 0);
  }
}

void sendData(long motorPos, float angle, float motorSpd, float angularSpd) {
  Serial.print(motorPos);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.print(motorSpd);
  Serial.print(",");
  Serial.println(angularSpd);
}

void parseCommand(String command) {
  usePositionControl = command.charAt(0) - '0';
  String valueStr = command.substring(1);
  float value = valueStr.toFloat();
  
  if (usePositionControl) {
    desiredPosition = value;
  } else {
    voltageOutput = map(value, -12, 12, -255, 255);
  }
}