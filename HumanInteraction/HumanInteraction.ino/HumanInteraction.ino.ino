#include <FastLED.h>

/* -------------------------------
 * Defines
 * ------------------------------- */
#define LED_PIN     3 //4 is MOSI pin on Uno; example sketch says 5
#define NUM_LEDS    96

#define centerFlashPin 7
#define sirenPin    6

#define inBit0 8
#define inBit1 9
#define inBit2 10
#define inBit3 11

//to prevent too high  a power draw use 25% (64) brightness
#define BRIGHTNESS  25
#define LED_TYPE    WS2811 //WS2811; NEOPIXEL
#define COLOR_ORDER GRB

/* -------------------------------
 * Structs
 * ------------------------------- */
CRGB leds[NUM_LEDS];

/* -------------------------------
 * Constants
 * ------------------------------- */
// Variables for the navigation lights
int greenIndicators[] = {31, 46};
int greenIndicatorsSize = (sizeof(greenIndicators)/sizeof(int));
int redIndicators[] = {15,95};
int redIndicatorsSize = (sizeof(redIndicators)/sizeof(int));
int whiteIndicators[] = {63,79};
int whiteIndicatorsSize = (sizeof(whiteIndicators)/sizeof(int));

// Variables for the lights on the arms used for simulated rotation light
// Note that this numbering follows a CC numbering starting at the right top most arm.
// Therefore: 1=3, 2=5, 1=3, 4=4, 5=6, 6=3
// 16 LEDs on each arm
int arm1[] = {0  ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 , 10 , 11 , 12 , 13 , 14};
int arm1Size = (sizeof(arm1)/sizeof(int));
int arm2[] = {16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 , 28 , 29 , 30};
int arm2Size = (sizeof(arm2)/sizeof(int));
int arm3[] = {32 , 33 , 34 , 35 , 36 , 37 , 38 , 39 , 40 , 41 , 42 , 43 , 44 , 45};
int arm3Size = (sizeof(arm3)/sizeof(int));
int arm4[] = {48 , 49 , 50 , 51 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62};
int arm4Size = (sizeof(arm4)/sizeof(int));
int arm5[] = {64 , 65 , 66 , 67 , 68 , 69 , 70 , 71 , 72 , 73 , 74 , 75 , 76 , 77 , 78};
int arm5Size = (sizeof(arm5)/sizeof(int));
int arm6[] = {80 , 81 , 82 , 83 , 84 , 85 , 86 , 87 , 88 , 89 , 90 , 91 , 92 , 93 , 94 };
int arm6Size = (sizeof(arm6)/sizeof(int));

// The following variables are for the simulated rotating light
long prevMillisRescueFlash;
long rescueFlashTiming[] = { 50, 75, 50, 325 }; // 1 period contains 2 rapid flashed in 1s, if 2 periods per sec change last value from 825 to 325
int rescueFlashState = 0;

// The following variables are for the simulated rotating light
long prevMillisRescueRotation;
long rescueRotationTiming[] = { 1, 1, 1, 1, 1, 1 };
int rescueRotationTimingMultiplier = 83; //83ms per arm; ~166 = 1 RPM
int rescueRotationState = 0;
int rescueRotationType = 1; // 1-5

// The flight indicators should be white at the back and then green on the right and red on the left both going to the front of the craft.
// Arm color layout
//    R  G
// R        G
//    W  W
// The flashing of strobe lights are for collision avoidance
long prevMillisFlightIndicator;
long flightIndicatorTiming[] = { 50, 75, 50, 75, 50, 1700 }; // in total 2s
int flightIndicatorState = 0;

bool updateLEDs = false;
long prevUpdateLED;
long updateLEDTiming = 2; //number of ms between each update

long curMillis;

byte pinBValue;
bool rescueFlash = false;
bool rescueRotation = false;
bool indicators = false;
bool enableSiren = false;
bool allOFF = false;

bool landingLight = false;
long prevMillisLandingLight;
bool safeLight = false;
long prevMillisSafeLight;
int customBrightnessMax = 40;
int customBrightnessMin = 2;
int customBrightnessCur = customBrightnessMin;
bool customBrightnessDir = true;
int customBrightnessDelayLandingLight = 2;
int customBrightnessDelaySafeLight = 10;
int customBrightnessDelay;
int subAdder;
int subAdderLandingLight = 2;
int subAdderSafeLight = 1;


/* -------------------------------
 * Setup
 * ------------------------------- */
void setup() {
    delay( 2000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    //FastLED.addLeds<LED_TYPE, LED_PIN>(leds, NUM_LEDS);
    //FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );

    setLEDColor(0,NUM_LEDS,CRGB::Black);
    FastLED.show();

    // Setup center flashing light and siren
    pinMode(centerFlashPin, OUTPUT);
    pinMode(sirenPin, OUTPUT);

    // Setup input pins
    pinMode(inBit0, INPUT);
    digitalWrite(inBit0, HIGH);       // turn on pullup resistors
    pinMode(inBit1, INPUT);
    digitalWrite(inBit1, HIGH);       // turn on pullup resistors
    pinMode(inBit2, INPUT);
    digitalWrite(inBit2, HIGH);       // turn on pullup resistors
    pinMode(inBit3, INPUT);
    digitalWrite(inBit3, HIGH);       // turn on pullup resistors


    // Esnure initial timing
    curMillis = millis();
    prevMillisRescueFlash = curMillis - rescueFlashTiming[0];
    prevMillisRescueRotation = curMillis - ( rescueRotationTiming[0] * rescueRotationTimingMultiplier );
    prevMillisFlightIndicator = curMillis - flightIndicatorTiming[0];
    prevMillisLandingLight = curMillis - customBrightnessDelayLandingLight;
    prevMillisSafeLight = curMillis - customBrightnessDelaySafeLight;
    prevUpdateLED = curMillis;
}

/* -------------------------------
 * Main loop
 * ------------------------------- */
void loop() {
  curMillis = millis();
  pinBValue = PINB & B00001111; // Take only the pins of interest
  //pinBValue = 7;

  if(pinBValue == 0) {
    allOFF = true;
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = false;
    safeLight = false;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 1) {
    allOFF = false;
    rescueFlash = true;
    rescueRotation = true;
    indicators = false;
    enableSiren = false;
    safeLight = false;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 2) {
    allOFF = false;
    rescueFlash = false;
    rescueRotation = false;
    indicators = true;
    enableSiren = false;
    safeLight = false;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 3) {
    allOFF = false;
    rescueFlash = true;
    rescueRotation = true;
    indicators = true;
    enableSiren = false;
    safeLight = false;
    landingLight = false;
    
    //updateLEDs = true;
  } else if(pinBValue == 4) {
    allOFF = false;
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = true;
    safeLight = false;
    landingLight = false;

    updateLEDs = true;
  } else if(pinBValue == 5) {
    allOFF = false;
    rescueFlash = true;
    rescueRotation = true;
    indicators = false;
    enableSiren = true;
    safeLight = false;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 6) {
    allOFF = false;
    rescueFlash = true;
    rescueRotation = true;
    indicators = true;
    enableSiren = true;
    safeLight = false;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 7) {
    allOFF = false;
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = false;
    safeLight = true;
    landingLight = false;

    //updateLEDs = true;
  } else if(pinBValue == 8) {
    allOFF = false;
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = false;
    safeLight = false;
    landingLight = true;

    //updateLEDs = true;
  } else if(pinBValue == 9) {
    allOFF = false;
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = true;
    safeLight = false;
    landingLight = true;

    //updateLEDs = true;
  } else {
    setLEDColor(0,NUM_LEDS,CRGB::Black);
    
    rescueFlash = false;
    rescueRotation = false;
    indicators = false;
    enableSiren = false;
    safeLight = false;
    landingLight = false;

    updateLEDs = true;
  }

  /* RESCUE FLASH */
  if(rescueFlash) {
    if ((curMillis - prevMillisRescueFlash) > rescueFlashTiming[rescueFlashState]){
      switch (rescueFlashState) {
        case 0:
          digitalWrite(centerFlashPin, LOW);   // turn the LED on (HIGH is the voltage level)
          rescueFlashState = rescueFlashState + 1;
          break;
        case 1:
          digitalWrite(centerFlashPin, HIGH);   // turn the LED on (HIGH is the voltage level)
          rescueFlashState = rescueFlashState + 1;
          break;
        case 2:
          digitalWrite(centerFlashPin, LOW);   // turn the LED on (HIGH is the voltage level)
          rescueFlashState = rescueFlashState + 1;
          break;
        case 3:
          digitalWrite(centerFlashPin, HIGH);   // turn the LED on (HIGH is the voltage level)
          rescueFlashState = 0;
          break;
        default:
          rescueFlashState = 0;
      }
      prevMillisRescueFlash = curMillis;
    }
  } else {
    digitalWrite(centerFlashPin, LOW);   // turn the LED on (HIGH is the voltage level)
  }
  
  
  /* SIMULATED ROTATION LIGHT */
  if(rescueRotation) {
    if ((curMillis - prevMillisRescueRotation) > rescueRotationTiming[rescueRotationState]*rescueRotationTimingMultiplier){
      if(rescueRotationType == 1) {
        // 1 light rotating
        switch (rescueRotationState) {
          case 0:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 1:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 2:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 3:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 4:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 5:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = 0;
            break;
          default:
            rescueFlashState = 0;
        }
      } else if(rescueRotationType == 2) {
        // 3 lights rotating
        switch (rescueRotationState) {
          case 0:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
    
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 1:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
    
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 2:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
    
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 3:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 4:
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 5:
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = 0;
            break;
          default:
            rescueFlashState = 0;
        }
      } else if(rescueRotationType == 3) {
        // 2 lights i mirror configuration rotating
        switch (rescueRotationState) {
          case 0:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 1:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 2:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 3:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 4:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 5:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = 0;
            break;
          default:
            rescueFlashState = 0;
        }
      } else if(rescueRotationType == 4) {
        // 2 lights moving forwards and backwards on each side
        switch (rescueRotationState) {
          case 0:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 1:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 2:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 3:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 4:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 5:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = 0;
            break;
          default:
            rescueFlashState = 0;
        }
      } else if(rescueRotationType == 5) {
        // 2 lights moving to the front of the drone on each side
        switch (rescueRotationState) {
          case 0:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Blue);
            setLEDColor(arm4, arm4Size, CRGB::Blue);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 1:
            setLEDColor(arm1, arm1Size, CRGB::Black);
            setLEDColor(arm2, arm2Size, CRGB::Blue);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Blue);
            setLEDColor(arm6, arm6Size, CRGB::Black);
            rescueRotationState = rescueRotationState + 1;
            break;
          case 2:
            setLEDColor(arm1, arm1Size, CRGB::Blue);
            setLEDColor(arm2, arm2Size, CRGB::Black);
            setLEDColor(arm3, arm3Size, CRGB::Black);
            setLEDColor(arm4, arm4Size, CRGB::Black);
            setLEDColor(arm5, arm5Size, CRGB::Black);
            setLEDColor(arm6, arm6Size, CRGB::Blue);
            rescueRotationState = 0;
            break;
          default:
            rescueFlashState = 0;
        }
      }
      
      updateLEDs = true;
      prevMillisRescueRotation = curMillis;
    }
  } else {
    
    if( !isLEDColor(arm1, arm1Size, CRGB::Black) || !isLEDColor(arm2, arm2Size, CRGB::Black) || !isLEDColor(arm3, arm3Size, CRGB::Black) || !isLEDColor(arm4, arm4Size, CRGB::Black) || !isLEDColor(arm5, arm5Size, CRGB::Black) || !isLEDColor(arm6, arm6Size, CRGB::Black) ) {
      setLEDColor(arm1, arm1Size, CRGB::Black);
      setLEDColor(arm2, arm2Size, CRGB::Black);
      setLEDColor(arm3, arm3Size, CRGB::Black);
      setLEDColor(arm4, arm4Size, CRGB::Black);
      setLEDColor(arm5, arm5Size, CRGB::Black);
      setLEDColor(arm6, arm6Size, CRGB::Black);
      //updateLEDs = true;
    }
    
  }

  /* FLIGHT INDICATORS */
  if(indicators) {
    if ((curMillis - prevMillisFlightIndicator) > flightIndicatorTiming[flightIndicatorState]){
      switch (flightIndicatorState) {
        case 0:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::Red);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::Green);
          flightIndicatorState = flightIndicatorState + 1;
          break;
        case 1:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::White);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::White);        
          flightIndicatorState = flightIndicatorState + 1;
          break;
        case 2:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::Red);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::Green);        
          flightIndicatorState = flightIndicatorState + 1;
          break;
        case 3:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::White);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::White);        
          flightIndicatorState = flightIndicatorState + 1;
          break;
        case 4:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::Red);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::Green);        
          flightIndicatorState = flightIndicatorState + 1;
          break;
        case 5:
          setLEDColor(redIndicators, redIndicatorsSize, CRGB::White);
          setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::White);        
          flightIndicatorState = 0;
          break;
        default:
          flightIndicatorState = 0;
      }
      setLEDColor(whiteIndicators, whiteIndicatorsSize, CRGB::White);
      
      updateLEDs = true;
      prevMillisFlightIndicator = curMillis;
    }
  } else {
    if( !isLEDColor(redIndicators, redIndicatorsSize, CRGB::Black) || !isLEDColor(greenIndicators, greenIndicatorsSize, CRGB::Black) || !isLEDColor(whiteIndicators, whiteIndicatorsSize, CRGB::Black) ) {
      setLEDColor(redIndicators, redIndicatorsSize, CRGB::Black);
      setLEDColor(greenIndicators, greenIndicatorsSize, CRGB::Black);
      setLEDColor(whiteIndicators, whiteIndicatorsSize, CRGB::Black);
      //updateLEDs = true;
    }
  }

  /* SIREN */
  if(enableSiren) {
    digitalWrite(sirenPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(sirenPin, LOW);   // turn the LED on (HIGH is the voltage level)
  }

  /* SAFE AND LANDING LIGHT */
  if(landingLight || safeLight) {
    if(landingLight) {
      customBrightnessDelay = customBrightnessDelayLandingLight;
      subAdder = subAdderLandingLight;
      for( int i = 0; i < NUM_LEDS; i++) {
        if(i!=47) {
          leds[i] = CRGB::Red;
        }
      }
    } else if(safeLight) {
      customBrightnessDelay = customBrightnessDelaySafeLight;
      subAdder = subAdderSafeLight;
      for( int i = 0; i < NUM_LEDS; i++) {
        if(i!=47) {
          leds[i] = CRGB::Green;
        }
      }
    }
    
    if ((curMillis - prevMillisLandingLight) > customBrightnessDelay){
      if(customBrightnessDir) {
        customBrightnessCur = customBrightnessCur + subAdder;
        if (customBrightnessCur > customBrightnessMax) {
          customBrightnessCur = customBrightnessMax;
          customBrightnessDir = false;
        }
      } else {
        customBrightnessCur = customBrightnessCur - subAdder;
        if (customBrightnessCur < customBrightnessMin) {
          customBrightnessCur = customBrightnessMin;
          customBrightnessDir = true;
        }
      }
      FastLED.setBrightness(  customBrightnessCur );
      
      updateLEDs = true;
      prevMillisLandingLight = curMillis;
    }
  } else {
    FastLED.setBrightness(  BRIGHTNESS );
  }

  if(allOFF == true) {
    setLEDColor(0,NUM_LEDS,CRGB::Black);
    digitalWrite(sirenPin, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(centerFlashPin, LOW);   // turn the LED on (HIGH is the voltage level)
    
    updateLEDs = true;
  }
  
  if( updateLEDs && (curMillis - prevUpdateLED) > updateLEDTiming ) {
    FastLED.show();
    updateLEDs = false;
    prevUpdateLED = curMillis;
  }
}


/* -------------------------------
 * Methods
 * ------------------------------- */
void setLEDColor(int LEDArray[], int arrayLength, CRGB colorLED) {
  for( int i = 0; i < arrayLength; i++) {
    if(LEDArray[i] != 47) {
          leds[LEDArray[i]] = colorLED;
     }
  }
}
void setLEDColor(int startLED, int endLED, CRGB colorLED) {
  if(startLED >= 0 && endLED < NUM_LEDS) {
    for( int i = startLED; i < endLED+1; i++) {
      if(i != 47) {
        leds[i] = colorLED;
      }
    }
  }
}

bool isLEDColor(int LEDArray[], int arrayLength, CRGB colorLED) {
  for( int i = 0; i < arrayLength; i++) {
    if (leds[LEDArray[i]] != colorLED) {
      return false;
    }
  }
  return true;
}

/*
CRGB::Red
CRGB::Gray // 'white' is too bright compared to red and blue
CRGB::Blue
CRGB::Black
CRGB::Red
CRGB::White
CHSV( HUE_PURPLE, 255, 255)
CHSV( HUE_GREEN, 255, 255)
*/

