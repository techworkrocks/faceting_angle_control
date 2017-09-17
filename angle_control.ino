/*
 * FACETING HEAD LIMIT ANGLE CONTROL + DEPTH OF CUT INDICATOR
 * 
 * Arduino Uno
 * 20x4 I2C Display
 * A4988 Stepper motor controller
 * 
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 20, 4);

# define PRESELECTED_COUNT 10

# define X_STP 2 // x -axis stepper control
# define X_DIR 3 // X -axis stepper motor direction control
# define EN 4 // stepper motor enable , active low
# define DIR_FWD false // direction flag forward
# define DIR_BW true // direction flag backward
# define STEPPOS_MIN 0 // minimum of possible step positions
# define STEPPOS_MAX 16700 // maximum step position
# define BACKLASH_STEPS 200 // number of steps to go forward and back to eliminate backlash
# define TOUCH_SENSOR 5 // indication if the faceting arm touches the angle limit

# define MOVE_DOWN 7 // Manual angle adjustment
# define MOVE_UP 8 // Manual angle adjustment
# define SELECT_NEXT 9 // traverse slots with pre-selected angles
# define GOTO_SELECTED 10 // move motor to currently selected angle
# define SAVE_CURRENT 11 // save current angle to selected slot
# define CLEAR_CURRENT 12 // clear angle in current slot
# define CALIBRATE 13 // Calibrate position to 90 degrees

# define STATUS_STOPPED 1 // Motor is stopped, listening to analog touch sensor input
# define STATUS_RUNNING 2 // Motor running to new angle position

// angle to step map (non-linear)
# define NUM_CALIB_VALS 20 // size of arrays below
int degrees[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95 };
int steps[] = { 0, 785, 1488, 2314, 3200, 4160, 5082, 6109, 7178, 8148, 9188, 10122, 11101, 12017, 12914, 13795, 14605, 15375, 16145, 16739 };
// keep the last result cached, which boosts display refresh time
int bufLastSteps = -1;
float bufLastDegrees = 0;


int stepPos; // current position of stepper motor
int currentState; // status variable of the mini state machine 
boolean hasBacklash = false; // is set, if movement stops in (degree wise) upward direction
int preselectCursor; // index of currently selected memorized position
int preselectedPos[PRESELECTED_COUNT]; // array holding memorized positions

int targetStepPos; // is set at the beginning of a movement
boolean stepDir; // true...forward
boolean dirLast; // remember last direction to avoid setting direction for every step

// button status helpers
boolean lastState[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };

// only redraw numbers, if necessary
boolean screenDirty = true;

// ring buffer for average calculation of touch sensor
# define RINGBUF_SIZE 20
boolean touchBuf[] = { false, false, false, false, false, false, false, false, false, false,
                       false, false, false, false, false, false, false, false, false, false };
int bufIndex = 0;

void setup () {
  // stepper init
  pinMode (X_DIR, OUTPUT);
  pinMode (X_STP, OUTPUT);
  pinMode (EN, OUTPUT);
  
  // enable stepper motor power only when needed
  setMotorPower(false);
  dirLast = DIR_FWD;
  digitalWrite (X_DIR, dirLast); // initial set for performance optimization

  // lcd init
  lcd.init();
  lcd.backlight();
  prefillDisplay();

  // input init
  pinMode (MOVE_DOWN, INPUT);
  digitalWrite(MOVE_DOWN, HIGH); // pull-up
  pinMode (MOVE_UP, INPUT);
  digitalWrite(MOVE_UP, HIGH); // pull-up
  pinMode (SELECT_NEXT, INPUT);
  digitalWrite(SELECT_NEXT, HIGH); // pull-up
  pinMode (GOTO_SELECTED, INPUT);
  digitalWrite(GOTO_SELECTED, HIGH); // pull-up
  pinMode (SAVE_CURRENT, INPUT);
  digitalWrite(SAVE_CURRENT, HIGH); // pull-up
  pinMode (CLEAR_CURRENT, INPUT);
  digitalWrite(CLEAR_CURRENT, HIGH); // pull-up
  pinMode (CALIBRATE, INPUT);
  digitalWrite(CALIBRATE, HIGH); // pull-up

  // touch sensor init
  pinMode ( TOUCH_SENSOR, INPUT);
  digitalWrite ( TOUCH_SENSOR, INPUT_PULLUP);
  

  // variables
  currentState = STATUS_STOPPED;
  stepPos = 0; // assuming 90 degrees
  preselectCursor = 0; // preselection slot index
  targetStepPos = 0;
  preselectedPos[0] = 0;
  preselectedPos[1] = 16145;
  for(int i=2; i<PRESELECTED_COUNT; i++) // all empty at the beginning
    preselectedPos[i] = -1;

  Serial.begin(9600);

  // SOME TEST ROUTINES (TO BE COMMENTED OUT IN LIVE MODE)
  
//Serial.print("-- mapDegreesToSteps(45); "); Serial.println(mapDegreesToSteps(45));
//Serial.print("-- mapDegreesToSteps(0); "); Serial.println(mapDegreesToSteps(0));
//Serial.print("-- mapDegreesToSteps(90); "); Serial.println(mapDegreesToSteps(90));
//Serial.print("-- mapDegreesToSteps(33.5); "); Serial.println(mapDegreesToSteps(33.5));
//Serial.print("-- mapDegreesToSteps(45.1); "); Serial.println(mapDegreesToSteps(45.1));
//Serial.print("-- mapStepsToDegrees(0); "); Serial.println(mapStepsToDegrees(0));

//Serial.print("-- mapStepsToDegrees(16000); "); Serial.println(mapStepsToDegrees(16000));
//Serial.print("-- mapStepsToDegrees(2000); "); Serial.println(mapStepsToDegrees(2000));
//Serial.print("-- mapStepsToDegrees(2314); "); Serial.println(mapStepsToDegrees(2314));
//Serial.print("-- mapStepsToDegrees(10); "); Serial.println(mapStepsToDegrees(10));

}

/**
 * 
 * Loop consists of handling changes in the input buttons and update the display state afterwards.
 * 
 */
void loop () {
  
  if(currentState == STATUS_RUNNING) { // motor in motion
    if(digitalRead(MOVE_DOWN) == LOW) { // LOW means pressed
      if(stepPos > STEPPOS_MIN) {
        targetStepPos = stepPos - 1;
        hasBacklash = false; 
      }      
    } else if(digitalRead(MOVE_UP) == LOW) {
      if(stepPos < STEPPOS_MAX) {
        targetStepPos = stepPos + 1;
        hasBacklash = true;
      }
    } 

    if(stepPos != targetStepPos) {
      stepDir = stepPos < targetStepPos;
      hasBacklash = stepDir; // if forward movement backlash is introduced
      step (stepDir, X_DIR, X_STP, abs(targetStepPos - stepPos));
    } else {    
      // done, go forth and back to avoid backlash if necessary
      if(hasBacklash) {
        step (true, X_DIR, X_STP, BACKLASH_STEPS);
        step (false, X_DIR, X_STP, BACKLASH_STEPS);
        hasBacklash = false;
      }
      
      //switch status back
      currentState = STATUS_STOPPED;
      screenDirty = true;
      setMotorPower(LOW);
    }
  } else if(currentState == STATUS_STOPPED) {
    if(wasPressed(SELECT_NEXT)) {
      preselectCursor++;
      preselectCursor %= PRESELECTED_COUNT; // after 9 comes index 0
      screenDirty = true;
    }
    if(wasPressed(GOTO_SELECTED)) {
      if(preselectedPos[preselectCursor] != -1) {
        targetStepPos = preselectedPos[preselectCursor];
        setMotorPower(HIGH);
        currentState = STATUS_RUNNING;
        screenDirty = true;
      }
    }
    if(wasPressed(SAVE_CURRENT)) {
      preselectedPos[preselectCursor] = stepPos;
      screenDirty = true;
    }
    if(wasPressed(CLEAR_CURRENT)) {
      preselectedPos[preselectCursor] = -1;
      screenDirty = true;
    }
    if(wasPressed(CALIBRATE)) {
      stepPos = 0; // current motor position to origin (i.e. 90° for head)
      screenDirty = true;
    }
    // switch status to running if manual move starts
    if(wasPressed(MOVE_DOWN) || wasPressed(MOVE_UP)) {
      setMotorPower(HIGH);
      currentState = STATUS_RUNNING;
      screenDirty = true;
    }

    updateDisplay();
  }
  
} // END LOOP

void updateDisplay() {
  if(currentState == STATUS_STOPPED) {
    if(screenDirty) {
      lcd.setCursor(15,0); lcd.print("     ");
      lcd.setCursor(8,0); lcd.print(stepPos);  lcd.print("/"); lcd.print(mapStepsToDegrees(stepPos));  lcd.print(char(223));
      lcd.setCursor(4,1); if(preselectCursor<9) lcd.print("0"); lcd.print(preselectCursor+1);  
  
      if(preselectedPos[preselectCursor] > -1) {
        lcd.setCursor(15,1); lcd.print("     ");
        lcd.setCursor(8,1);
        lcd.print(preselectedPos[preselectCursor]);
        lcd.print("/");
        lcd.print(mapStepsToDegrees(preselectedPos[preselectCursor]));
        lcd.print(char(223));
      } else {
        lcd.setCursor(8,1);
        lcd.print("<empty>     ");
      }
      screenDirty = false;
    }
    // analog read and display
    lcd.setCursor(0,3);
    int touchDegree = getTouchDegree();

    for(int j=0; j<20; j++) {
      lcd.print(touchDegree > j ? char(255) : char(45));
    }
  } else if (currentState == STATUS_RUNNING) {
    lcd.setCursor(8,0); lcd.print("- RUNNING -"); 
  }
}

/**
 * Returns true, if the specified input was pressed since last checking
 */
boolean wasPressed(int input) {
  boolean cur = digitalRead(input);
  boolean last = lastState[input - MOVE_DOWN];
  boolean ret = false;
  
  if(last == HIGH && cur == LOW) {
    ret = true;
    Serial.print("Pressed: ");
    Serial.println(input);
  }

  lastState[input-MOVE_DOWN] = cur;
  return ret;
}

/**
 * Perform a movement with the stepper motor
 * 
 * Parameters: 
 *   dir        direction control
 *   dirPin     corresponding stepper motor DIR pin
 *   stepperPin corresponding stepper motor step pin
 *   steps      number of steps of no return value.
 * 
 */
void step (boolean dir, byte dirPin, byte stepperPin, int steps)
{
  // omit setting the direction, if we already have the right one
  if(dirLast != dir) {
    digitalWrite (dirPin, dir);
    delay (50);
    dirLast = dir;
  }

  // The values d, t, a were obtained by testing with the specific stepper motor used
  // (might be different in other setups)
  int d = 1000; // start delay
  int t = 400; // travel delay
  int a = 10; // acceleration
  
  int c = d; // current delay  
  for (int i = 0; i <steps; i++) {
    digitalWrite (stepperPin, HIGH);
    delayMicroseconds (c);
    digitalWrite (stepperPin, LOW);
    delayMicroseconds (c);

    if(dir == true) 
      stepPos ++;
    else
      stepPos --;

    // accelerate until travel speed is reached
    if(c > t) {
      c = c - a;
    }
  }
}

/**
 * Return how close the head is touching the target angle. Therefore a ring buffer with 20 entries
 * is round-robin wise filled with the binary indication if the head touches the angle limit, or not. 
 * With a spinning lap it will always be the case that the signal will be jumping between touch and no touch.
 * By counting the touch results in the ring buffer the degree how close we are to a constant "touch" is a 
 * value between 0 and 20, where 20 means that the last 20 samples indicated a touch.
 */
int getTouchDegree() {
  // TODO: actually a digital input would do it as well
  //int r = analogRead(A1);
  boolean isTouched = !digitalRead(TOUCH_SENSOR);
  
  //touchBuf[bufIndex] = (r < 820);
  touchBuf[bufIndex] = isTouched;
  bufIndex ++;
  bufIndex %= RINGBUF_SIZE;

  int degree = 0;
  for (int i=0; i<RINGBUF_SIZE; i++) {
    if (touchBuf[i]) degree++;
  }
  return degree;
}

/**
 * Control die ENABLE line for the stepper motor driver
 * If act is HIGH, power is ON
 **/
void setMotorPower(boolean act) {
Serial.print("Set Motor Power to: ");
Serial.println(act);  
  digitalWrite (EN, !act);
  delay(100);
}

void prefillDisplay() {
  lcd.print("   Act: ");
  lcd.setCursor(0,1); lcd.print("Mem-  :");
  lcd.setCursor(0,3); lcd.print("--------------------");
}

/**
 * Use the discrete arrays as calibration values and interpolate between the two
 * closest points.
 */
 /*
int mapDegreesToSteps(int d) {
  // find closest lower index
  int i = findIndexOf(d, degrees);

  int distanceSteps = steps[i+1] - steps[i];
  int distanceDegrees = degrees[i+1] - degrees[i];

  return steps[i] + distanceSteps * (d - degrees[i]) / distanceDegrees;
}
*/

/**
 * Use the discrete arrays as calibration values and interpolate between the two
 * closest points to map the step count to the inclination angle.
 * 
 * Also has a simple cache mechanism in place to remember the last result.
 */
float mapStepsToDegrees(int s) {
  if(bufLastSteps == s)
    return bufLastDegrees;
    
  int i = findIndexOf(s, steps);

  int distanceSteps = steps[i+1] - steps[i];
  int distanceDegrees = degrees[i+1] - degrees[i];

  bufLastSteps = s;
  bufLastDegrees = (float) degrees[i] + (float) distanceDegrees * (float) (s - steps[i]) / distanceSteps;
  return bufLastDegrees;
}

/**
 * Simple recursive binary search to quickly find the right calibration points to interpolate within.
 */
int findIndexOf(int v, int* arr) {
  return findIndexOfRec(v, arr, 0, NUM_CALIB_VALS);
}

int findIndexOfRec(int v, int* arr, int low, int high) { 
//Serial.print("find: v="); Serial.print(v); Serial.print(", low="); Serial.print(low);
//Serial.print(", high="); Serial.println(high);

  int mid = (high+low) / 2;
  
  if(arr[low] <= v) {
    if(arr[low+1] > v) {
//Serial.print("Got it: ");      
//Serial.println(low);
      return low; // found
    } 

    if(arr[mid] <= v) {
      return findIndexOfRec(v, arr, mid, high );
    } else { 
      return findIndexOfRec(v, arr, low, mid );
    } 
  } 

  return -1;
}


