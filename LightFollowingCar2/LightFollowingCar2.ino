class PidController {
  private:
    double targetValue, proportionalGain, integralGain, derivativeGain, max, min, offset, lowPassFilterGain, minAccumError, maxAccumError; //vars set by constructor
    double previousError = 0;
    double previousDerivative = 0;
    double totalError = 0;
    double lastTime = 0;

  public: PidController(double targetValue, double Kp, double Ki, double Kd, double offset, double lowPassFilterGain, double minOutput, double maxOutput, double minTotalError, double maxTotalError) {
    this->targetValue = targetValue;
    this->proportionalGain = Kp;
    this->integralGain = Ki;
    this->derivativeGain = Kd;
    this->offset = offset;
    this->lowPassFilterGain = lowPassFilterGain;
    this->max = maxOutput;
    this->min = minOutput;
    this->minAccumError = minTotalError;
    this->maxAccumError = maxTotalError;
  }

  public: double calculateOutput(double currentInput) {
    //calculate the change in time since function was last called
    //side note: micros() is the most accurate but overflows after 70 minutes. Might be a problem? Can be substitued for millis() if need be
    double deltaTime = micros() - lastTime;
    deltaTime /= 1000000; //convert to seconds
    lastTime = micros();

    //calculate error + proportional
    double error = currentInput - targetValue;
    double proportional = proportionalGain * error;

    //calculate accumulated error, clamp it, then calculate integral. todo: clamp may be unneccesary??
    totalError += error * deltaTime;
    totalError = constrain(totalError, minAccumError, maxAccumError);
    double integral = integralGain * totalError;

    //calculate derivative
    double derivative = (error - previousError) / deltaTime;
    //apply low pass filter
    derivative = (lowPassFilterGain * previousDerivative) + ((1 - lowPassFilterGain) * (derivative));

    //update "previous" vars to prepare for when this function next runs
    previousDerivative = derivative;
    previousError = error;

    //apply derivative gain to get final derivative value
    derivative = derivative * derivativeGain;

    //calculate sum of p+i+d, clamp it, then apply offset, then return value
    double pid = proportional + integral + derivative;
    return offset + constrain(pid, min, max);
  }

  public: void multiplyTotalError(double multi) {
    totalError = totalError * multi;
  }
};

PidController turnPid = PidController(
  0, 1, 0, 0, 0, 0.5, -255, 255, -10000, 10000
);

PidController travelPid = PidController(
  0, 1, 0, 0, 0, 0.5, -255, 255, -10000, 10000
);


int rightMotor1 = 7;
int rightMotor2 = 8;
int leftMotor1 = 12;
int leftMotor2 = 13;

int pwmLeft = 9;
int pwmRight = 10;

void traverse(int backward, int forward, int left, int right) {
  double forwardImbalance = forward - backward;
  double rightImbalance = right - left;

  double baseMotorSpeed = travelPid.calculateOutput(forwardImbalance);
  double motorSpeedBias = turnPid.calculateOutput(rightImbalance);

  Serial.println(String(baseMotorSpeed) + " " + String(motorSpeedBias));

  if (baseMotorSpeed > -100 && baseMotorSpeed < 100) {
    baseMotorSpeed = 0;
    travelPid.multiplyTotalError(0.9);
  }
  else {
    travelPid.multiplyTotalError(0.95);
  }


  if (motorSpeedBias > -40 && motorSpeedBias < 40) {
    motorSpeedBias = 0;
    turnPid.multiplyTotalError(0.9);
  }
  else {
    travelPid.multiplyTotalError(0.95);
  }

  //Serial.println(String(baseMotorSpeed) + " " + String(motorSpeedBias));

  int leftMotorSpeed = baseMotorSpeed + motorSpeedBias;
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  int rightMotorSpeed = baseMotorSpeed - motorSpeedBias;
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  }
  else if (rightMotorSpeed == 0) {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
  else {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }
  
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
  }
  else if (leftMotorSpeed == 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
  }
  else {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
  }

  leftMotorSpeed = abs(leftMotorSpeed);
  rightMotorSpeed = abs(rightMotorSpeed);

  double speedMulti = 1;

  analogWrite(pwmLeft, leftMotorSpeed * speedMulti);
  analogWrite(pwmRight, rightMotorSpeed * speedMulti);
}

//you can't get rid of this. i have no idea why
int leftPin = 4;
int forwardPin = 5;
int rightPin = 3;
int backPin = 2;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  pinMode(leftPin, OUTPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(backPin, OUTPUT);
}

void loop() {

  //int left = analogRead(A0);
  //int forward = analogRead(A1);
  //int right = analogRead(A2);
  //int backward = analogRead(A4);
  //read LDRs
  int left = analogRead(A4);
  int forward = analogRead(A1);
  int right = analogRead(A2);
  int backward = analogRead(A0);

  //Serial.println(String(forward) + " " + String(backward) +  " " +String(right) + " " + String(left));

  traverse(backward, forward, left, right);

  delay(100);
}

