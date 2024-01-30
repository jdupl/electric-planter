/*
// Electric holland self-propelled planter
//
// BTS7960 ctrl Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
// BTS7960 ctrl Pin 8 (GND) to Arduino GND
*/

#define DEBUG_EN 0 // set to 1 or 0
#define TICK_MS 50

const int NUM_READINGS = 10;

// Analog pins
#define THROTTLE_PIN  0

// Digital pins
#define SEL_PIN 7

// Wheel motor
#define W_RPWM  3 // BTS7960 M1 Pin 1 (RPWM)
#define W_LPWM  5 // BTS7960 M1 Pin 2 (LPWM)

// min and max PWM delta whitin a tick
#define MAX_PWM_CHANGE_PER_TICK_ACCEL 128
#define MAX_PWM_CHANGE_PER_TICK_DECCEL 80
#define MAX_PWM 255


#define THROTTLE_MIN_VAL 0 // analog value at idle
#define THROTTLE_MAX_VAL 1023 // analog value at max input


#define FORWARD 1
#define REVERSE -1


int g_lastThrottlePWM = 0;
int g_lastDirectionState = 0;

void printTx(String chars) {
  if (DEBUG_EN == 1) {
    Serial.println(chars);
  }
}

int smoothAcceleration(int targetPWM) {
    int scaledPWM = 0;

    // no change in PWM
    if (g_lastThrottlePWM == targetPWM) {
      return targetPWM;
    }

    int pwmDelta = targetPWM - g_lastThrottlePWM;

    if (pwmDelta > 0 && pwmDelta > MAX_PWM_CHANGE_PER_TICK_ACCEL) {
      // acceleration too fast
      printTx("Slowing down acceleration...");
      scaledPWM = g_lastThrottlePWM + MAX_PWM_CHANGE_PER_TICK_ACCEL;

    } else if (pwmDelta < 0 && abs(pwmDelta) > MAX_PWM_CHANGE_PER_TICK_DECCEL) {
      // deacceleration too fast
      printTx("Slowing down deacceleration...");
      scaledPWM = g_lastThrottlePWM - MAX_PWM_CHANGE_PER_TICK_DECCEL;

    } else {
      // pwm change is legit
      scaledPWM = targetPWM;
    }
    return scaledPWM;
}

int convertAnalogThrottleToPercent(int val) {
  if (val <= THROTTLE_MAX_VAL && val > THROTTLE_MIN_VAL) {
    return map(val, THROTTLE_MIN_VAL, THROTTLE_MAX_VAL, 0, 100);
  }
  return 0;
}

void readThrottleInputs(int &userThrottleAnalogVal, int &userDirectionState) {
  int userReverseDigitalVal = digitalRead(SEL_PIN);
  userThrottleAnalogVal = analogRead(THROTTLE_PIN);

  if (userReverseDigitalVal == LOW) {
    userDirectionState = FORWARD;
  } else if (userReverseDigitalVal == HIGH){
    userDirectionState = REVERSE;
  }

  printTx("throttle input from analog: " + String(userThrottleAnalogVal));
  printTx("direction input from user:" + String(userDirectionState));
}

int convertPercentToPWM(int percent) {
  int pwm = int(percent * MAX_PWM / 100);

  if (pwm > MAX_PWM) {
    printTx("FATAL ERROR: Wheel pwm exceeds max pwm set: " + String(pwm));
    pwm = 0;
  }

  printTx("Wheel pwm: " + String(pwm));

  return pwm;
}

boolean hasDirectionConflict(int userDirectionState) {
  // avoid switching from FORWARD to REVERSE when motors are moving
  return g_lastDirectionState != userDirectionState \
      && g_lastThrottlePWM != 0;
}

void saveCurrentThrottleInfos(int scaledPWM, int protectedDirectionState) {
  g_lastThrottlePWM = scaledPWM;
  g_lastDirectionState = protectedDirectionState;
}

void doThrottleUpdate(int targetPWM, int protectedDirectionState) {
  int scaledPWM = smoothAcceleration(targetPWM);

  // Write the PWM value to the respective pins
  if (protectedDirectionState == FORWARD) {
    analogWrite(W_LPWM, scaledPWM);
    analogWrite(W_RPWM, 0);
  } else if (protectedDirectionState == REVERSE) {
    analogWrite(W_LPWM, 0);
    analogWrite(W_RPWM, scaledPWM);
  }

  saveCurrentThrottleInfos(scaledPWM, protectedDirectionState);
}

int doDirectionConflictHandling(int userDirectionState) {
  if (hasDirectionConflict(userDirectionState)) {
    printTx("WARNING: refusing to change direction state. Motors not halted");
    return g_lastDirectionState;
  } else {
    return userDirectionState;
  }
}

void update() {
  int userThrottleAnalogVal = 0;
  int userDirectionState = 0;

  readThrottleInputs(userThrottleAnalogVal, userDirectionState);

  int throttlePercent = convertAnalogThrottleToPercent(userThrottleAnalogVal);

  doThrottleUpdate(convertPercentToPWM(throttlePercent),
                   doDirectionConflictHandling(userDirectionState));

}

void setup() {
  Serial.begin(9600);

  pinMode(W_RPWM, OUTPUT);
  analogWrite(W_RPWM, 0);

  pinMode(W_LPWM, OUTPUT);
  analogWrite(W_LPWM, 0);


  // pinMode(SEL_PIN, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT);

  g_lastDirectionState = FORWARD;
  g_lastThrottlePWM = 0;
}

void loop() {

  update();

  delay(TICK_MS);
}
