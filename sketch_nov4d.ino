#include "LiquidCrystal_I2C.h"
#include "Wire.h"

#include "pitches.h"

LiquidCrystal_I2C lcd (0x27,  16, 2);

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        2 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

void setupNeopixels() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}


// BEGIN COPIED SECTION
// FROM: https://www.instructables.com/12V-4-Wire-Fan-Controller-With-Arduino-Uno/ | Temp_humid_Fan_add_anim.ino

#define PIN_SENSE 3 //where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno) // modified
unsigned long volatile ts1=0,ts2=0;

const int PIN_PIEZO = 6;

//configure Timer 1 (pins 9,10) to output 25kHz PWM
void setupTimer1(){
    //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;
}

//equivalent of analogWrite on pin 9
void setPWM1A(float f){
    f=f<0?0:f>1?1:f;
    OCR1A = (uint16_t)(320*f);
}

volatile unsigned long lastTach;
volatile unsigned long timeDelta;

const unsigned int PIN_DIAL = A0;

bool tachISRBase() {
    unsigned long m=micros();
    //if ((m - lastTach) < 1) return false;
    timeDelta = m - lastTach;
    lastTach = m;
    return true;
}

const unsigned int N_TIMEDELTAS = 6;

volatile unsigned short timeDeltaWindowFilled;
volatile unsigned long timeDeltaWindow [N_TIMEDELTAS];

void tachISRAverage() {
  if (tachISRBase()) {
    for (unsigned int i = 0; i < (N_TIMEDELTAS - 1); i++) {
      timeDeltaWindow[i] = timeDeltaWindow[i + 1];
    }
    timeDeltaWindow[N_TIMEDELTAS - 1] = timeDelta;
  }
}

const unsigned long RPM_FACTOR = 15000;

//Calculates the RPM based on the timestamps of the last 2 interrupts. Can be called at any time.
unsigned long calcRPM(){
    return (1000 * RPM_FACTOR) / timeDelta;
}

unsigned long calcRPMAverage() {
  unsigned long tdSum = 0;
  for (unsigned int i = 0; i < N_TIMEDELTAS; i++) tdSum += timeDeltaWindow[i];
  return (1000 * RPM_FACTOR) / (tdSum / N_TIMEDELTAS);
}

unsigned long calcRPMAverageN(unsigned int n) {
  unsigned long tdSum = 0;
  for (unsigned int i = (N_TIMEDELTAS - n); i < N_TIMEDELTAS; i++) tdSum += timeDeltaWindow[i];
  return (1000 * RPM_FACTOR) / (tdSum / n);
}

void setupFanCode() { // modified
  pinMode(9,OUTPUT); //1A  enable outputs for Timer 1
  setupTimer1();
  pinMode(PIN_SENSE,INPUT_PULLUP); //set the sense pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE),tachISRAverage,FALLING); //set tachISR to be triggered when the signal on the sense pin goes low
} // modified

// END COPIED SECTION

struct Prog {
  char name [4];
  unsigned int stepCount;
  unsigned int rpmSteps [9];
  unsigned int timeSecSteps [9];
};


unsigned long lastSwitch = 0;
bool fast = false;
bool setFast = false;

unsigned int spinPos = 0;
unsigned long lastSpin = 0;

int currProg = 0;
int lastProg = 0;
unsigned short currStep = 0;
unsigned long stepStartedMillis = 0;

float currPwm = 0;
double currPwmD = 0;

const int N_PROGS = 2;

Prog progs [N_PROGS] = {
  {"TST", 2, {1000, 2000}, {25, 15}},
  {"1K1", 1, {1000}, {60}}
};

unsigned int speedSet = 0;

float clampFloat(float x, float l, float u) {
  if (x < l) return l;
  if (x > u) return u;
  return x;
}

double clampDouble(double x, double l, double u) {
  if (x < l) return l;
  if (x > u) return u;
  return x;
}

unsigned long lastScreen = 0;
unsigned long lastPidUpdate = 0;

bool endMode = false;

bool progJustStarted = false;
bool actuallyDone = false;
unsigned long slowDownStart = 0;

const double PID_KP = 0.000018;
const unsigned int N_RPMSAMPLES_CONTROLLER = 3;
const double CONTROLLER_RECENCYBIAS = 0.25;

struct Melody {
  unsigned int length;
  int notes [16];
  int durations [16];
};

enum MelodyNo {
  MELODY_RDY,
  MELODY_START,
  MELODY_STEP,
  MELODY_SLOW,
  MELODY_END,
};

Melody melodies [] = {
  {
    5,
    {NOTE_D4, NOTE_G4, NOTE_G3, NOTE_G3, NOTE_G3},
    {4, 8, 8, 8, 8}
  }, // ^ MELODY_RDY
  {
    4,
    {NOTE_AS4, NOTE_D4, NOTE_G3, NOTE_D3, NOTE_AS4, NOTE_D4},
    {8, 8, 4, 4, 8, 8}
  }, // ^ MELODY_START
  {
    2,
    {NOTE_AS4, NOTE_AS3},
    {8, 8}
  }, // ^ MELODY_STEP
  {
    4,
    {NOTE_G2, NOTE_G2, NOTE_AS2, NOTE_AS2},
    {8, 8, 8, 8}
  }, // ^ MELODY_SLOW
  {
    8,
    {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4},
    {4, 8, 8, 4, 4, 4, 4, 4}
  }, // ^ MELODY_END
};

const int PIN_BTN = 5;

void playMelody(int currMelody) {
  for (unsigned int i = 0; i < melodies[currMelody].length; i++) {
    int noteDuration = 1000 / melodies[currMelody].durations[i];
    tone(6, melodies[currMelody].notes[i], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(6);
  }
}

bool doneCleared = false;

unsigned long lastSelectScreenUpdate = 0;
unsigned long timeCleared = 0;

bool playedEndMelody = false;

void showSelectScreen() {
  if ((millis() - timeCleared) < 1000) {
    return;
  }
  int dialPos = analogRead(PIN_DIAL) / (1024 / N_PROGS);
  if (dialPos >= N_PROGS) dialPos = N_PROGS - 1;
  int chosenProg = dialPos;
  if ((millis() - lastSelectScreenUpdate) > 500) {
    lcd.clear();
    lcd.setCursor(0, 0);
    if (!doneCleared) {
      lcd.print(progs[lastProg].name);
      lcd.print(" done. Hold");
      lcd.setCursor(0, 1);
      lcd.print("button to clear");
      if (!playedEndMelody) {
        pixels.setBrightness(90);
        pixels.show();
        playMelody(MELODY_END);
        pixels.setBrightness(160);
        pixels.show();
        playedEndMelody = true;
      }
      if (digitalRead(PIN_BTN) == LOW) {
        doneCleared = true;
        timeCleared = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("CLEARED OK");
      }
      return;
    }
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(100, 100, 100));
    }
    pixels.show();
    lcd.print("Program ");
    lcd.print(chosenProg + 1);
    lcd.print("/");
    lcd.print(N_PROGS);
    lcd.print(": ");
    lcd.print(progs[chosenProg].name);
    lcd.setCursor(0, 1);
    int totTime = 0;
    for (int i = 0; i < progs[chosenProg].stepCount; i++) {
      totTime += progs[chosenProg].timeSecSteps[i];
    }
    if (totTime >= 60) {
      if (((totTime / 60) * 60) != totTime) {
        lcd.print("~");
      }
      lcd.print((totTime + 30) / 60);
      lcd.print(" min, ");
    } else {
      lcd.print(totTime);
      lcd.print(" sec, ");
    }
    if (progs[chosenProg].stepCount == 1) {
      lcd.print("1 step");
    } else {
      lcd.print(progs[chosenProg].stepCount);
      lcd.print(" steps");
    }
    lastSelectScreenUpdate = millis();
  }
  Serial.println(digitalRead(PIN_BTN));
  if (digitalRead(PIN_BTN) == LOW) {
    currProg = chosenProg;
    progJustStarted = true;
    currStep = 0;
    actuallyDone = false;
    doneCleared = false;
    playMelody(MELODY_START);
    digitalWrite(7, HIGH);
  }
}

void setup() {
  // put your setup code here, to run once:
  setupFanCode();
  pinMode(PIN_BTN, INPUT_PULLUP);
  lcd.init();
  lcd.backlight();
  //lcd.noBacklight();
  lcd.setCursor(0, 0);
  Serial.begin(9600);
  setupNeopixels();
  pixels.setBrightness(160);
  // prepare prog 0
  // currProg = 0;
  // progJustStarted = true;
  pinMode(7, OUTPUT);
  // digitalWrite(7, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  // start at select screen
  currProg = -1;
  actuallyDone = true;
  doneCleared = true;
  // indicate ready
  lcd.print("System Ready!");
  playMelody(MELODY_RDY);
}

void loop() {
  if (currProg != -1) lastProg = currProg;
  if (currProg == -1) {
    if (!actuallyDone) {
      endMode = true;
      doneCleared = false;
      if (slowDownStart == 0) {
        slowDownStart = millis();
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(255, 0, 0));
        }
        pixels.show();
        playMelody(MELODY_SLOW);
      }
      unsigned int rpm = (unsigned int)calcRPMAverage();
      actuallyDone = (
        ((millis() - slowDownStart) > 5000) && (
          (rpm < 100) || (rpm > 10000) || (currPwmD < 0.30)
        )
      );
      if (actuallyDone) {
        digitalWrite(7, LOW);
        int rpmUse = (int)rpm;
        setPWM1A(0.0f);
        playedEndMelody = false;
        while (!((rpmUse < 200) || (rpmUse > 10000))) {
          digitalWrite(7, HIGH);
          delay(50);
          rpmUse = (unsigned int)calcRPMAverage();
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("!0rpm (OFF)");
          lcd.setCursor(13, 0);
          lcd.print(progs[lastProg].name);
          lcd.setCursor(0, 1);
          lcd.print("=");
          if (rpmUse > 10000) {
            lcd.print("NO TACH");
          } else {
            lcd.print(rpmUse);
            lcd.print("rpm");
          }
          lcd.setCursor(8, 1);
          lcd.print("STOP");
          lcd.setCursor(13, 1);
          lcd.print("END");
          digitalWrite(7, LOW);
          delay(500);
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("!0rpm");
        lcd.setCursor(9, 0);
        lcd.print("3s");
        lcd.setCursor(13, 0);
        lcd.print(progs[lastProg].name);
        lcd.setCursor(0, 1);
        lcd.print("TACH OK");
        lcd.setCursor(8, 1);
        lcd.print("WAIT");
        lcd.setCursor(13, 1);
        lcd.print("END");
        delay(1000);
        lcd.setCursor(9, 0);
        lcd.print("2s");
        delay(1000);
        lcd.setCursor(9, 0);
        lcd.print("1s");
        delay(1000);
      }
      speedSet = 0;
    } else {
      slowDownStart = 0;
      endMode = false;
      setPWM1A(0.0f);
      digitalWrite(7, LOW);
      showSelectScreen();
      if (!doneCleared) {
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        }
        pixels.show();
        delay(100);
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        }
        pixels.show();
        delay(150);
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        }
        pixels.show();
        delay(100);
        for (int i = 0; i < NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        }
        pixels.show();
        delay(800);
      }
      return;
    }
  } else {
    // before we actually update fan:
    if (progJustStarted) {
      currStep = -1;
    }
    unsigned int stepSecsPassed = (int)((millis() - stepStartedMillis) / 1000);
    unsigned int stepSecsRemaining = progs[currProg].timeSecSteps[currStep] - stepSecsPassed;
    if (progJustStarted || (stepSecsRemaining <= 0)) {
      currStep++;
      playMelody(MELODY_STEP);
      progJustStarted = false;
      speedSet = progs[currProg].rpmSteps[currStep];
      stepStartedMillis = millis();
      if (currStep >= progs[currProg].stepCount) {
        currProg = -1;
        return;
      }
    }
  }
  // actually update fan
  if ((millis() - lastPidUpdate) > 40) {
    unsigned int rpmI = (unsigned int)calcRPMAverageN(N_RPMSAMPLES_CONTROLLER);
    unsigned int rpmL = (unsigned int)calcRPM();
    unsigned int rpm = (unsigned int)(
      (((double)rpmL) * CONTROLLER_RECENCYBIAS)
      +
      (((double)rpmI) * (1.0 - CONTROLLER_RECENCYBIAS))
    );
    double err = (double)(((int)speedSet) - ((int)rpm));
    if (err > 103) {
      err = (double)(((int)(speedSet - 100)) - ((int)rpm));
    }
    double usePID_KP = PID_KP;
    if (speedSet < 1350) {
      if (err < -5) {
        usePID_KP = PID_KP * 1.15;
      }
      if (err < -7) {
        usePID_KP = PID_KP * 1.3;
      }
      if (abs(err) < 5) {
        usePID_KP = PID_KP * 0.97;
      }
    } else {
      if (err > -20) {
        usePID_KP = PID_KP * 0.5;
      } else {
        usePID_KP = PID_KP * 0.93;
      }
    }
    double adj;
    if (err > 50.0) {
      if ((currPwmD < 0.3) && (speedSet > 800)) {
        adj = -101.;
      } else {
        adj = clampDouble(usePID_KP * err, -0.09, 0.09);
      }
    } else {
      adj = clampDouble(usePID_KP * err, -0.004, 0.004);
    }
    if (endMode) {
      adj = -0.01;
    }
    /*
    Serial.print(speedSet);
    Serial.print(", act = ");
    Serial.print(rpm);
    Serial.print(" -> err = ");
    Serial.print(err);
    Serial.print(" -> adj = ");
    Serial.println(adj);
    // */
    if (adj < -100.0) {
      currPwmD = 0.3;
      currPwm = ((float)currPwmD);
    } else {
      currPwmD = clampDouble(currPwmD + adj, 0.0, 1.0);
      currPwm = ((float)currPwmD);
    }
    setPWM1A(currPwm);
    lastPidUpdate = millis();
  }
  if ((millis() - lastScreen) > 500) {
    if ((currProg == -1) && endMode) {
      lastScreen = millis();
      unsigned int rpm = (unsigned int)calcRPMAverage();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("!0rpm");
      lcd.setCursor(13, 0);
      lcd.print(progs[lastProg].name);
      lcd.setCursor(0, 1);
      lcd.print("=");
      if (rpm > 10000) {
        lcd.print("NO TACH");
      } else {
        lcd.print(rpm);
        lcd.print("rpm");
      }
      lcd.setCursor(9, 1);
      if (currPwm < 0.01) {
        lcd.print(".");
        lcd.print((int)(currPwm * 1000.0f));
      } else {
        lcd.print(((int)(currPwm * 100.0f)));
      }
      lcd.print("%");
      lcd.setCursor(13, 1);
      lcd.print("END");
    }
    if ((currProg != -1)) {
      lastScreen = millis();
      unsigned int rpm = (unsigned int)calcRPMAverage();
      unsigned int stepSecsPassed = (int)((millis() - stepStartedMillis) / 1000);
      unsigned int stepSecsRemaining = progs[currProg].timeSecSteps[currStep] - stepSecsPassed;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("!");
      lcd.print(speedSet);
      lcd.print("rpm");
      lcd.setCursor(9, 0);
      if (stepSecsRemaining > 60) {
        lcd.print(stepSecsRemaining / 60);
        lcd.print("m");
      } else {
        lcd.print(stepSecsRemaining);
        lcd.print("s");
      }
      lcd.setCursor(13, 0);
      lcd.print(progs[currProg].name);
      lcd.setCursor(0, 1);
      lcd.print("=");
      // 1234567890123456
      // !1000rpm 25s ZnO
      // =1000rpm 95% 1/9
      // Program 2/5:
      // ZnO #1 (ZnO1)
      if (rpm > 10000) {
        lcd.print("NO TACH");
      } else {
        lcd.print(rpm);
        lcd.print("rpm");
      }
      lcd.setCursor(9, 1);
      if (currPwm < 0.01) {
        lcd.print(".");
        lcd.print((int)(currPwm * 1000.0f));
      } else {
        lcd.print(((int)(currPwm * 100.0f)));
      }
      lcd.print("%");
      lcd.setCursor(13, 1);
      lcd.print(currStep + 1);
      lcd.print("/");
      lcd.print(progs[currProg].stepCount);
      //Serial.println(rpm);
    }
    if ((speedSet > 0.01) && ((millis() - lastSpin) >= 60)) {
      spinPos = (spinPos + 1) % NUMPIXELS;
      pixels.setPixelColor((spinPos + 0) % 8, pixels.Color(214 / 2, 2 / 2, 112 / 2));
      pixels.setPixelColor((spinPos + 1) % 8, pixels.Color(155 / 3, 79 / 3, 150 / 3));
      pixels.setPixelColor((spinPos + 2) % 8, pixels.Color(0 / 2, 56 / 2, 168 / 2));
      pixels.setPixelColor((spinPos + 3) % 8, pixels.Color(0 / 4, 56 / 4, 168 / 4));
      pixels.setPixelColor((spinPos + 4) % 8, pixels.Color(0, 0, 0));
      pixels.setPixelColor((spinPos + 5) % 8, pixels.Color(0, 0, 0));
      pixels.setPixelColor((spinPos + 6) % 8, pixels.Color(0, 0, 0));
      pixels.setPixelColor((spinPos + 7) % 8, pixels.Color(214 / 4, 2 / 4, 112 / 4));
      pixels.show();
      lastSpin = millis();
    }
  }
}
