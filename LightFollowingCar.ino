class PidController {
  private:
    double targetValue, proportionalGain, integralGain, derivativeGain, max, min, offset, lowPassFilterGain; //vars set by constructor
    double previousError = 0;
    double previousDerivative = 0;
    double totalError = 0;
    double timeCounter = 0;
    double lastTime = 0;

  public: PID(double targetValue, double Kp, double Ki, double Kd, double offset, double lowPassFilterGain, double maxOutput, double minOutput) {
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
};

void setup() {
  // put your setup code here, to run once:

}

void loop() {

}

