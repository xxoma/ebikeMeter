#include <Nokia2.h>
#include <EEPROM.h>
LCD Display = LCD();


int batMonPin = A5;    // input pin for the voltage divider
int batVal = 0;       // variable for the A/D value

int backlightPin = 9;
int keyPin = 8;
float pinVoltage = 0; // variable to hold the calculated voltage
float batteryVoltage = 0;
float totalVoltage = 0;
float avgVoltage = 0;

int analogInPin = A4;  // Analog input pin that the carrier board OUT is connected to
int sensorValue = 0;        // value read from the carrier board
float outputValue = 0;        // output in milliamps
unsigned long msec = 0;
unsigned long prevMsec = 0;
float time = 0.0;
int sample = 0;
float totalCharge = 0.0;
float averageAmps = 0.0;
float ampSeconds = 0.0;
float ampHours = 0.0;
float wattHours = 0.0;
float amps = 0.0;
float wattHkm = 0.0;
boolean ledState = false;
boolean on_off_led = true;
boolean saved = false;
boolean led = false;

//addreses
byte ledAddr = 0;
byte msecAddr = 1;
byte totalChargeAddr = 5;
byte sampleAddr = 10;
byte distanceAddr = 15;
byte totalDistanceAddr = 20;
byte totalVoltageAddr = 25;
int R1 = 10000; // Resistance of R1 in ohms
int R2 = 1000; // Resistance of R2 in ohms

int i = 0;

volatile float period = 0.0;
volatile unsigned long prevMillis = 0;
volatile float speedValue = 0;
volatile float distance = 0;
volatile float totalDistance = 0;
volatile float WHEEL = 2.070;

float ratio = 0;  // Calculated from R1 / R2

void setup() {
  attachInterrupt(0, speed, FALLING);
  Display.Setup();
  
  Display.Clear();
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  
  pinMode(backlightPin, OUTPUT); // backlight pin
  digitalWrite(backlightPin, HIGH);
  pinMode(keyPin, INPUT);
  on_off_led =  EEPROM.read(ledAddr);
  msec = EEPROM_float_read(msecAddr); //read time
  totalCharge = EEPROM_float_read(totalChargeAddr); //read totalCharge
  sample = EEPROM_float_read(sampleAddr); //read sample
  distance = EEPROM_float_read(distanceAddr); //read distance
  totalDistance = EEPROM_float_read(totalDistanceAddr);
  totalVoltage = EEPROM_float_read(totalVoltageAddr);
}

void speed(){
  delay(5);
  
  period = millis() - prevMillis;
  prevMillis = millis();
  if(period > 30){
    speedValue = ((WHEEL/1000)/period)*1000*60*60;
    distance = distance+(WHEEL/1000);
    totalDistance = totalDistance+(WHEEL/1000);
  }
}

void loop() {
  
  
  int sampleBVal = 0;
  int avgBVal = 0;  
  int sampleAmpVal = 0;
  int avgSAV = 0;
 

  if( digitalRead(keyPin) == LOW ){
    delay(10);
      i++;
      led = true;
    }
    
  if( digitalRead(keyPin) == HIGH ){
      led = false;
      if(i>0){
        on_off_led =!on_off_led;
        i=0;
      }
      if (i>10){
        resetMemory();
        Display.Clear();
        Display.setTextColor(1);
        Display.setTextSize(2);
        Display.setCursor(5,0);
        Display.print("RESET...");
        Display.Update();
        delay(1000);
        i = 0;
      }else{
        i=0;
      }
  }
  
  if((on_off_led &&ledState) || led){
    digitalWrite(backlightPin, HIGH);
  }else{
    digitalWrite(backlightPin, LOW);
  }
  
 for (int x = 0; x < 10; x++){ // run through loop 10x
 
  // read the analog in value:
  sensorValue = analogRead(analogInPin) - 512;   
  sampleAmpVal = sampleAmpVal + sensorValue; // add samples together

  batVal = analogRead(batMonPin);    // read the voltage on the divider 
  sampleBVal = sampleBVal + batVal; // add samples together
  
  delay (10); // let ADC settle before next sample

 }

 avgSAV = sampleAmpVal / 10;
 
 outputValue = (avgSAV*4.98)/66;

 
 avgBVal = sampleBVal / 10; //divide by 10 (number of samples) to get a steady reading
 
 pinVoltage = avgBVal * 0.005265;  

 ratio = (float)R1 / (float)R2;
 batteryVoltage = pinVoltage * ratio;    //  Use the ratio calculated for the voltage divider
                                          //  to calculate the battery voltage
                                          
                                          
 amps = (float) outputValue;
 amps = abs(amps);
 float watts = amps * batteryVoltage;
    
 sample = sample + 1;
 prevMsec = millis()-prevMsec; 
 msec = msec + prevMsec;
 
 totalVoltage = totalVoltage + batteryVoltage;
 
 avgVoltage = totalVoltage / sample;
  
 time = (float) msec / 1000.0;
  
 totalCharge = totalCharge + amps;
  
 averageAmps = totalCharge / sample;
  
 ampSeconds = averageAmps*time;

 ampHours = ampSeconds/3600;
  
 wattHours = avgVoltage * ampHours;
 
 wattHkm = wattHours/distance;


 if(millis() - prevMillis > 3000){
    speedValue = 0;
 }
 
 if(millis() - prevMillis > 10000){
    //save data
    if(!saved){
     EEPROM_float_write(msecAddr, (float)msec); //save time
     EEPROM_float_write(totalChargeAddr, (float)totalCharge); //save totalCharge
     EEPROM_float_write(sampleAddr, (float)sample); //save sample
     EEPROM_float_write(distanceAddr, (float)distance); //save distance
     EEPROM_float_write(totalDistanceAddr, totalDistance);
     EEPROM_float_write(totalVoltageAddr, totalVoltage);
     EEPROM.write(ledAddr, on_off_led);
     Display.Clear();
     Display.setTextColor(1);
        Display.setTextSize(2);
        Display.setCursor(5,0);
        Display.print("SAVED...");
        Display.Update();
        delay(1000);
     ledState = false;
    }
     saved = true;
 }else{
    saved = false;
 }
  
 if((millis() - prevMillis < 10000) || amps > 0.5){
      ledState = true;
 }
  
  Display.setTextColor(1);


  Display.Clear();
  
  Display.fillRoundRect(0,23,95,18,0,1);
  Display.setTextSize(2);
  
  Display.setCursor(0,0);
  Display.print(speedValue, 0);
  Display.setTextSize(1);
  Display.setCursor(0,15);
  Display.print("km/h");
  
  Display.setTextSize(2);
  Display.setCursor(45,0);
  Display.print(watts, 0);
  Display.setTextSize(1);
  Display.setCursor(80,15);
  Display.print("W");
  
  Display.setCursor(1,24);
  Display.setTextColor(0);
  Display.print(distance);
  Display.print("km ");
  Display.setCursor(45,24);
  Display.setTextColor(0);
  Display.print(wattHkm, 1);
  Display.print("Wh/k");
  Display.setCursor(1,33);
  Display.print(wattHours, 1);
  Display.print("Wh ");
  Display.setCursor(45,33);
  Display.setTextColor(0);
  Display.print(amps, 1);
  Display.print("A");
  
  Display.setTextColor(1);
  
  Display.setTextSize(2);
  
  Display.setCursor(0,43);
  Display.print(batteryVoltage, 1);
  Display.setTextSize(1);
  //Display.setCursor(0,58);
  Display.print("V");
  
  Display.setTextSize(2);
  Display.setCursor(50,43);
  Display.print(ampHours, 1);
  Display.setTextSize(1);
  //Display.setCursor(45,58);
  Display.print("Ah");
  
  Display.fillRoundRect(0,58,95,10,0,1);
  Display.setTextColor(0);
  Display.setCursor(35,58);
  Display.print(totalDistance, 0);
  Display.print("km");
  Display.Update();
  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);
  prevMsec = millis();  
}

void EEPROM_float_write(int addr, float val) // write to ЕЕПРОМ
{  
  byte *x = (byte *)&val;
  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
}

float EEPROM_float_read(int addr) // read from ЕЕПРОМ
{    
  byte x[4];
  for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
  float *y = (float *)&x;
  return y[0];
}

void resetMemory(){
    EEPROM_float_write(msecAddr, 0);
    EEPROM_float_write(totalChargeAddr, 0);
    EEPROM_float_write(sampleAddr, 0);
    EEPROM_float_write(distanceAddr, 0);
    
    msec = 0;
    totalCharge = 0;
    sample = 0;
    distance = 0;
}
