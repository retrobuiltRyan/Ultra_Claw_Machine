// Ryan Bates || Brian Lee
// TechSpark Claw Machine + Accel Stepper Motors
// Sept 29 2021

/* ============Ryan's Change Log [Add new features]:=================
 * Start button, Coin Insert button; triggers game start and game cycles
 * Gameplay loop, 60 second timers 
 * 4x20 i2C LCD to give user feedback and debug
 * [On going] LCD and LED updates are only once per 1 second
 * [On going] Swapped in sub routines to make code more readable
 * [On going] Addressable LEDs change color based on what is happening (idle, in game-timer, gameover)
 * 
 * [Have Not Added] homing functions for  1) power on and 2) gameOver
 * [Have Not Added] Test mode (full control without timer and LCD read what inputs are pressed.)
 * [On going] Serial Monitor debug (print what is happening all the time) will eventually comment out all of this to save processor resources. 
 * [Have Not Added] Demo Mode (press button (secret location) for a random x,y, position and claw grab. 
 * [Have Not Added] Luck/ Mode to toggle between.
 * [Have Not Added] Enable/ Diable Stepper moters & Claw when idle. ENA pin needs to be wired and pulled LOW to cut motor
   power when not in use (Claw Machine is in idle).ENA+ (+5v) ENA- (to arduino) i think this is right? 
 * [Might Change/ Refine] Servo / Claw behavior

i am ass at programming. Sincerely, Ryan
*/
#include <Servo.h>
#include <Stepper.h>
#include <Adafruit_NeoPixel.h> // for addressable RGB LEDs
#define PIN            16 //pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS      12 // How many NeoPixels are attached to the Arduino?
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 (address can vary) for a 20 chars and 4 line display

Servo claw;
const int clawPin = 5;
const int clawUp = 26;
const int clawDown = 27;
const int clawClose = 28;
const int clawOpen = 29;
const int startButton = 30;
const int coinInsert= 31; 
const int servoRelay = 53;
const int StepperEnable = 52; /// NEW!
const int stepperSpeedDelay = 1;
const int stepperPulse = 1;

int clawStep = 15; // degrees
int clawCurValue = constrain(clawCurValue, 0, 180);
int clawOpenButtonState = 0;
int clawOpenButtonLastState = 0;
int clawCloseButtonState = 0;
int clawCloseButtonLastState = 0;

const int xStepPin = 11; 
const int xDirPin = 10;
const int y1StepPin = 9;
const int y1DirPin = 8;
const int y2StepPin = 7;
const int y2DirPin = 6;

const int zStepPin = 13;  //(claw up/down)
const int zDirPin = 12;   //(claw up/down)

const int xMinLmtSwtch = 15;
const int xMaxLmtSwtch = 14;
const int yMinLmtSwtch = 3;
const int yMaxLmtSwtch = 2;
const int clawLmtSwtch = 4;

const int forwardJoystick = 22;
const int backwardJoystick = 23;
const int leftJoystick = 24;
const int rightJoystick = 25;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
//Stepper Xstepper(stepsPerRevolution, 11,11); // library wants 2 or 4 wires (pins) to send pulses too, but these stepper
//Stepper Y1stepper(stepsPerRevolution, 9,9); //  drivers (TB6600?) only step and direction pin to interface. 
//Stepper Y2stepper(stepsPerRevolution, 7,7); //  What i am saying is, this will compile ok, just pay no mind to it. 
//Stepper Zstepper(stepsPerRevolution, 13,13); // 

//------------------time keeping variables for the gameplay loop------------------------------------

//unsigned long previousMillis = 0;            // timestamp made when something happens; dont think this is used anymore.
const long gameloopTimeLimit = 20;             // Game play Time Limit. Time that clawmachine will respond to user input
long elapsedTime;                              // Elasped game time in seconds (counts up!) 
long countDown;                                // Remaining game time in seconds, (counts down!)

unsigned long seconds;
unsigned long gameStartTime;

// ------------------time keeping variables for updating the LCD and LED strips once every so often------------------
unsigned long previousMillis1 = 0;        // will store last time LED was updated
const long updateInterval = 1000;         // interval at which to update user visual feedback

void setup() {
  pinMode(xStepPin, OUTPUT);
  pinMode(xDirPin, OUTPUT);
  pinMode(y1StepPin, OUTPUT);
  pinMode(y1DirPin, OUTPUT);
  pinMode(y2StepPin, OUTPUT);
  pinMode(y2DirPin, OUTPUT);
  pinMode(zStepPin, OUTPUT);
  pinMode(zDirPin, OUTPUT);
  pinMode(servoRelay, OUTPUT);
  pinMode(StepperEnable, OUTPUT);
    
  pinMode(xMinLmtSwtch, INPUT_PULLUP);
  pinMode(xMaxLmtSwtch, INPUT_PULLUP);
  pinMode(yMinLmtSwtch, INPUT_PULLUP);
  pinMode(yMaxLmtSwtch, INPUT_PULLUP);
  pinMode(clawLmtSwtch, INPUT_PULLUP);

  pinMode(clawUp, INPUT_PULLUP);
  pinMode(clawDown, INPUT_PULLUP);
  pinMode(clawOpen, INPUT_PULLUP);
  pinMode(clawClose, INPUT_PULLUP);
  pinMode(forwardJoystick, INPUT_PULLUP);
  pinMode(backwardJoystick, INPUT_PULLUP);
  pinMode(leftJoystick, INPUT_PULLUP);
  pinMode(rightJoystick, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);


//Xstepper.setSpeed(400);
//Y1stepper.setSpeed(200);
//Y2stepper.setSpeed(200);
//Zstepper.setSpeed(400);
  claw.attach(clawPin); //attaches servo to PWN pin
  pixels.begin();       // This initializes the NeoPixel library.
  
  Serial.begin(9600);   // initialize the serial port:
  
  lcd.init();      //initialize the LCD
  lcd.backlight(); //turn on backlight
  lcd.setCursor(5,0);
  lcd.print("TechSpark's");
  lcd.setCursor(4,1);
  lcd.print("Ultra Claw");
  delay(1000);
  lcd.clear();
}

void loop() {//+++++++++++START OF LOOP+++++++++++++++++++++++++
lcd.setCursor(0,0);
lcd.print("Press Start to play!");
digitalWrite(StepperEnable, HIGH);     // disable motors at the motor driver (EN+)
Serial.println("Press Start to play!");
unsigned long seconds = millis()/1000; // mark the system time as currentmillis
  
  //lcd.setCursor(0,1);   //this is debug, showing the Arduino system time
  //lcd.print("System time: ");lcd.print(seconds); 

  
while (digitalRead(startButton)==LOW){ //enter this loop only when the start button is pressed
      runGameSkill();
  }

 
}  //---------------END OF LOOP---------------------------------



void runGameSkill(){
  lcd.clear();
  digitalWrite(StepperEnable, LOW);     // Enaable motors at the motor driver (EN+)
   digitalWrite(servoRelay, HIGH); // enable power to the servo (claw)
   for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,10,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    lcd.setCursor(0,3); lcd.print("elapsed Time: ");
    delay(5); // Delay for a period of time (in milliseconds).
   }
   
  //===========Time Keeping Functions=====MUCH IMPORTANT, NO TOUCH==================================
  gameStartTime = millis()/1000; //mark the system time as 'now'.
while (millis()/1000 - gameStartTime < gameloopTimeLimit){
  elapsedTime = millis()/1000- gameStartTime; // subtract system clock time from the "now" time stamp
  lcd.print(elapsedTime); // using this variable to light LEDs with Game time
  lcd.print("    ");
  LCD_LED_update(); //updates screen and shows some visual feedback for game timer



     
//==================Enable MOVEMENT======================================================
// Movement must be written as while loops. The controls must have program priority so they under inputs feel responsive.
// In the background of each loop time is being checked to force gameover if stuck in movement loop.

  while ((digitalRead(leftJoystick) == LOW) && (digitalRead(xMinLmtSwtch) == HIGH) ){
    countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // this force breaks the loop if the time expires
    if (countDown== 0)
    {break;}
    X_left();  
      LCD_LED_update();  

    }
    
  while ((digitalRead(rightJoystick) == LOW) && (digitalRead(xMaxLmtSwtch) == HIGH)){
    countDown = gameStartTime+gameloopTimeLimit-millis()/1000;
    if (countDown== 0)
    {break;}
   X_right();
     LCD_LED_update();    
   
   }

  while ((digitalRead(forwardJoystick) == LOW) && (digitalRead(yMaxLmtSwtch) == HIGH)) {
    countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // this force breaks the loop if the time expires
    if (countDown== 0)
    {break;}
  Y_forward(); 
    LCD_LED_update();
  
  }
  
  while ((digitalRead(backwardJoystick) == LOW) && (digitalRead(yMinLmtSwtch) == HIGH)){
    countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // this force breaks the loop if the time expires
    if (countDown== 0)
    {break;}
  Y_backward(); 
    LCD_LED_update();
  }

  
  while ((digitalRead(clawUp) == LOW) && (digitalRead(clawLmtSwtch) == HIGH)){
  countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // this force breaks the loop if the time expires
    if (countDown== 0)
    {break;}
  Zclaw_up(); 
    LCD_LED_update(); 
  }

  while ((digitalRead(clawDown) == LOW)){
   countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // this force breaks the loop if the time expires
    if (countDown== 0)
    {break;} 
  Zclaw_down();
    LCD_LED_update();
  }

 clawOpenServo();
 clawCloseServo();

}
gameOver();
returnHome();

}
void LCD_LED_update()                                       {
  unsigned long currentMillis1 = millis();          // does this go here? maybe higher up in the loop
    if (currentMillis1 - previousMillis1 >= updateInterval) {
    // save the last time you updated the status
    previousMillis1 = currentMillis1;
    // do the thing once per interval
     //-----countdown timer and bar animation
     lcd.setCursor(17,1); 
     countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // count down Timer
     lcd.print(countDown); // show the count down seconds so you know the game will end
     if ((gameStartTime+gameloopTimeLimit-millis()/1000) < 10)
     lcd.print(" "); //fixes the missing leading zero for numbers under 10
     lcd.setCursor(4,1); lcd.print("Time Limit:");
     lcd.setCursor((millis()/1000- gameStartTime)*1,2); lcd.write(0xFF); //LCD status bar for time 
     //-----some LED + visual feedback based on how much time is left
    pixels.setPixelColor(countDown, pixels.Color(10,0,0)); //red color.
    pixels.show();
    }}
//void displayTime(){ //obsolete, rolled into the update LED and LCD routine
// //-----countdown timer and bar animation
//     lcd.setCursor(17,1); 
//     countDown = gameStartTime+gameloopTimeLimit-millis()/1000; // count down Timer
//     lcd.print(countDown); // show the count down seconds so you know the game will end
//     if ((gameStartTime+gameloopTimeLimit-millis()/1000) < 10)
//     lcd.print(" "); //fixes the missing leading zero for numbers under 10
//     lcd.setCursor(4,1); lcd.print("Time Limit:");
//     lcd.setCursor((millis()/1000- gameStartTime)*1,2); lcd.write(0xFF); //LCD status bar for time
//  
//}
void returnHome(){
 // code to home machine to prize shoot (x0,y0,z0) 
 // Close claw | wait a second | Goto Z0
 // Lift claw to Z min switch
  while (digitalRead(xMinLmtSwtch) == HIGH)
  X_left();
  while (digitalRead(yMinLmtSwtch) == HIGH)
  Y_forward();
 // Open claw

 // Turn off LEDs
    lcd.clear(); lcd.setCursor(0,0);
    LEDS_off();
}

void gameOver(){
   lcd.clear(); lcd.setCursor(0,0);
    for(int i=0;i<3;i++){ //Blink LEDs on/off red for GAMEOVER
      colorWipe(pixels.Color(0, 0, 0), 0); // Red
      delay(250); // Delay for a period of time (in milliseconds).
      colorWipe(pixels.Color(10, 0, 0), 0); // Red
      delay(250); // Delay for a period of time (in milliseconds).
    }
   lcd.print("GAME OVER!");  // could animate this to stall for time? 
   for (int i=0; i<= 4; i++){
     lcd.print(" GAME OVER!    ");  // could animate this to stall for time? 
     delay(200);            }
   delay(100);
   lcd.clear(); lcd.setCursor(4,1);
    digitalWrite(servoRelay, LOW); //disable power to the servo (claw)
 }
void LEDS_off(){
    for(int i=0;i<NUMPIXELS;i++){ //turn LEDs RED for GAMEOVER
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Turn off.
    pixels.show(); 
    delay(100); // Delay for a period of time (in milliseconds).
    }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixels.show();
    delay(0);
  }
}

//==================Gantry Movement subroutines=======Please Keep things tidy=======================
/*
Stepper Xstepper(stepsPerRevolution, 11,11); // library wants 2 or 4 wires (pins) to send pulses too, but these stepper
Stepper Y1stepper(stepsPerRevolution, 7,7); //  drivers TB6600? only step and direction pin to interface. 
Stepper Y2stepper(stepsPerRevolution, 9,9); //  What i am saying is this will compile ok, just pay no mind to it. 
Stepper Zstepper(stepsPerRevolution, 13,13); // 

 */
void X_left(){
  digitalWrite(xDirPin, LOW);   //left direction
  digitalWrite(xStepPin, HIGH); 
  delay(stepperPulse);
  digitalWrite(xStepPin, LOW); 
  delay(stepperPulse);        // speed 
  //Serial.println("Left");
}

void X_right(){
  digitalWrite(xDirPin, HIGH);   //right direction
  digitalWrite(xStepPin, HIGH); 
  delay(stepperPulse);
  digitalWrite(xStepPin, LOW); 
  delay(stepperPulse);        // speed 
 // Xstepper.step(25);
  //Serial.println("Right");
}

void Y_forward(){
  digitalWrite(y1DirPin, HIGH);   //CW
  digitalWrite(y2DirPin, LOW);   //CCW
  digitalWrite(y1StepPin, HIGH); 
  digitalWrite(y2StepPin, HIGH);
  delay(stepperPulse);
  digitalWrite(y1StepPin, LOW); 
  digitalWrite(y2StepPin, LOW);
  delay(stepperPulse);
  //Y2stepper.step(1);
  //Serial.println("Forward");
}

void Y_backward(){
  digitalWrite(y1DirPin, LOW);    //CW
  digitalWrite(y2DirPin, HIGH);   //CCW
  digitalWrite(y1StepPin, HIGH); 
  digitalWrite(y2StepPin, HIGH);
  delay(stepperPulse);
  digitalWrite(y1StepPin, LOW); 
  digitalWrite(y2StepPin, LOW);
  delay(stepperPulse);
  //Serial.println("Backward");
  }

  void Zclaw_up(){
  digitalWrite(zDirPin, HIGH);   //CW
  digitalWrite(zStepPin, HIGH);
  delay(stepperPulse); 
  digitalWrite(zStepPin, LOW);
  delay(stepperPulse); 
  //Zstepper.step(25);
  //Serial.println("up");
  }

  void Zclaw_down(){
  digitalWrite(zDirPin, LOW);   //CCW
  digitalWrite(zStepPin, HIGH);
  delay(stepperPulse); 
  digitalWrite(zStepPin, LOW);
  delay(stepperPulse); 
  //Zstepper.step(25);
  //Serial.println("Down");
  }

  void clawOpenServo(){
 // Claw OPEN
  clawOpenButtonState = digitalRead(clawOpen);
  //Serial.println(clawCurValue);
  // compare the buttonState to its previous state
  if (clawOpenButtonState != clawOpenButtonLastState) 
  {
   
    // if the state has changed, increment the counter
    if (digitalRead(clawOpen)) 
    {clawCurValue = claw.read() + clawStep;
      claw.write(clawCurValue);
      //Serial.println(clawCurValue);
    }
    // Delay a little bit to avoid bouncing
    delay(10);
                                          }
  clawOpenButtonLastState = clawOpenButtonState;
  }
  


  void clawCloseServo(){
  clawCloseButtonState = digitalRead(clawClose);
  // compare the buttonState to its previous statex
  if (clawCloseButtonState != clawCloseButtonLastState) 
  {
   
    // if the state has changed, increment the counter
    if (digitalRead(clawClose)) 
    {clawCurValue = claw.read() - clawStep;
      claw.write(clawCurValue);
      Serial.println(clawCurValue);
    }
    // Delay a little bit to avoid bouncing
    delay(10);
  // save the current state as the last state, for next time through the loop
  clawCloseButtonLastState = clawCloseButtonState;
                            } 
    
  }
