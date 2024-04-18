class PidController {
  private:
    double targetValue, proportionalGain, integralGain, derivativeGain, max, min, offset, lowPassFilterGain; //vars set by constructor
    double previousError = 0;
    double previousDerivative = 0;
    double totalError = 0;
    double lastTime = 0;

  public: PidController(double targetValue, double Kp, double Ki, double Kd, double offset, double lowPassFilterGain, double maxOutput, double minOutput) {
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
    totalError = 0;
    lastTime = micros();
  }
};

PidController travelPid = PidController(
  1,1,1,1,1,1,0,255
);

PidController turnPid = PidController(
  1,1,1,1,1,1,0,255
);

void travel(int backward, int forward) {
  int imbalance = forward - backward;
  double motorOutput = travelPid.calculateOutput(imbalance);
  //analogWrite(TRAVELSLOT, motorOutput);
}

void turn(int left, int right) {
  int imbalance = right - left;
  double motorOutput = turnPid.calculateOutput(imbalance);
  //analogWrite(TURNSLOT, motorOutput);
}

void stop() {
  //analogWrite(TRAVELSLOT, 0); //0, or whatever neutral is
}

void setup() {
  
    
}

void loop() {
  //TODO: READ THE VALUES
  int forward;
  int backward;
  int left;
  int right;

  //calculate threshold. If any lightsensor is above the threshold, run turn/travel
  int threshold = ((forward + backward + left + right) / 4) + 50;
  if (backward > threshold || forward > threshold || left > threshold || right > threshold) {
    turn(left, right);
    travel(backward, forward);
  }
  //if no lightsensors above threshold, stop the vehicle and reset the PIDs
  else {
    stop();
    travelPid.reset();
    turnPid.reset();
  }
  delay(50);
}

