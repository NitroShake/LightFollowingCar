class PidController {
  private:
    double targetValue, proportionalGain, integralGain, derivativeGain, max, min, offset, lowPassFilterGain; //vars set by constructor
    double previousError = 0;
    double previousDerivative = 0;
    double totalError = 0;
    double lastTime = 0;

  public: PidController(double targetValue, double Kp, double Ki, double Kd, double offset, double lowPassFilterGain, double minOutput, double maxOutput, double minIntegral, double maxIntegral) {
    this->targetValue = targetValue;
    this->proportionalGain = Kp;
    this->integralGain = Ki;
    this->derivativeGain = Kd;
    this->offset = offset;
    this->lowPassFilterGain = lowPassFilterGain;
    this->max = maxOutput;
    this->min = minOutput;
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

    //calculate accumulated error, clamp it, then calculate integral
    totalError += error * deltaTime;
    totalError = constrain(totalError, minIntegral, maxIntegral);
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

  //
  public: void lowerIntegral() {
    totalError = totalError * 0.9;
  }
};

PidController turnPid = PidController(
  0, 0.1, 0.1, 0.1, 0, 0.5, -255, 255, -80, 80
);

PidController travelPid = PidController(
  0, 0.1, 0.1, 0.1, 0, 0.5, -255, 255, -80, 80
);


int in1 = 8;
int in2 = 9;
int in3 = 10;
int in4 = 11;

void traverse(int backward, int forward, int left, int right) {
  double forwardImbalance = forward - backward;
  double rightImbalance = right - left;

  double baseMotorSpeed = travelPid.calculateOutput(forwardImbalance);
  double motorSpeedBias = turnPid.calculateOutput(rightImbalance);

  if (baseMotorSpeed > -15 && baseMotorSpeed < 15) {
    baseMotorSpeed = 0;
    travelPid.lowerIntegral();
  }
  if (motorSpeedBias > -15 && motorSpeedBias < 15) {
    motorSpeedBias = 0;
    turnPid.lowerIntegral();
  }

  int leftMotorSpeed = baseMotorSpeed - motorSpeedBias;
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  int rightMotorSpeed = baseMotorSpeed + motorSpeedBias;
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  if (leftMotorSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if (rightMotorSpeed > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  leftMotorSpeed = abs(leftMotorSpeed);
  rightMotorSpeed = abs(rightMotorSpeed);

  analogWrite(PWMLEFTPIN, leftMotorSpeed);
  analogWrite(PWMRIGHTPIN, rightMotorPin);
}

//TODO: you can probably get rid of this
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
  //read LDRs
  int left = analogRead(A0);
  int forward = analogRead(A1);
  int right = analogRead(A2);
  int backward = analogRead(A4);

  Serial.println(String(forward) + " " + String(left) +  " " +String(right) + " " + String(backward));

  traverse(backward, forward, left, right);

  delay(1000);
}

