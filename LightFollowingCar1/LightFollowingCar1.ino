class PidController {
  private:
    double targetValue, proportionalGain, integralGain, derivativeGain, max, min, offset, lowPassFilterGain; //vars set by constructor
    double previousError = 0;
    double previousDerivative = 0;
    double totalError = 0;
    double lastTime = 0;

  public: PidController(double targetValue, double Kp, double Ki, double Kd, double offset, double lowPassFilterGain, double minOutput, double maxOutput) {
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
    //clamp values are calculated outside before constrain (documentation says don't do operations inside constrain)
    //clamp values are arbitrary btw, feel free to change maybe if doesn't work well
    double minClamp = (min / integralGain) / 1.3;
    double maxClamp = (max / integralGain) / 1.3;
    totalError = constrain(totalError, minClamp, maxClamp);
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

  public: void reset() {
    totalError = totalError * 0.9;
    lastTime = micros();
  }
};

PidController turnPid = PidController(
  0, 0.1, 0.1, 0.1, 0, 0.5, -255, 255
);

PidController travelPid = PidController(
  0, 1, 1, 1, 0, 0.5, -255, 255
);


int in1 = 8;
int in2 = 9;
int in3 = 10;
int in4 = 11;

void travel(int backward, int forward) {
  int imbalance = forward - backward;
  
  double motorOutput = travelPid.calculateOutput(imbalance);

  int intMotorOutput = round(motorOutput);

  if (intMotorOutput > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  Serial.println(motorOutput);
  Serial.println(intMotorOutput);
  //analogWrite(TRAVELSLOT, motorOutput);
}

void turn(int left, int right) {
  int imbalance = right - left;
  double motorOutput = turnPid.calculateOutput(imbalance);
  int intMotorOutput = round(motorOutput);
  if (intMotorOutput > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  Serial.println(motorOutput);
  Serial.println(intMotorOutput);
  //analogWrite(TURNSLOT, motorOutput);
}

void stop() {
  //analogWrite(TRAVELSLOT, 0); //0, or whatever neutral is
}

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
  // put your main code here, to run repeatedly:
  // int value = analogRead(A0);
  // Serial.println("1:" + String(value));

  // int value2 = analogRead(A1);
  // Serial.println("2:" + String(value2));

  // int value3 = analogRead(A2);
  // Serial.println("3:" + String(value3));

  // int value4 = analogRead(A4);
  // Serial.println("4:" + String(value4));

  int left = analogRead(A0);
  int forward = analogRead(A1);
  int right = analogRead(A2);
  int backward = analogRead(A4);

  Serial.println(String(forward) + " " + String(left) +  " " +String(right) + " " + String(backward));

  //calculate threshold (mean + num). If any lightsensor is above the threshold, run turn/travel
  int threshold = ((forward + backward + left + right) / 4) + 20;

  if (backward > threshold || forward > threshold) {
    travel(backward, forward);
  }
  else {
    stop();
    travelPid.reset();
  }

  if (left > threshold || right > threshold) {
    turn(left,right);
  }
  else {
    stop();
    turnPid.reset();
  }

  

  delay(1000);
}

