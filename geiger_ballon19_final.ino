#include <Adafruit_MCP9808.h>
#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h" 
#include <KalmanFilter.h>
#include <SPI.h>
#include <SD.h>
#include "RadiationWatch.h"
#define DHT11_PIN 0      // ADC0
#define DHT11_PINN 2      // ADC2
unsigned char ret = 0;

double radiationB = 0.0;
double radiationB2 = 0.0;

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

/* Instance */
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter

// defining the varible globally cuz we use it in setup and loop
File myFile;

byte read_dht11_dat()
{
    byte i = 0;
    byte result=0;
    for(i=0; i< 8; i++){

        while(!(PINF & _BV(DHT11_PIN)));  // wait for 50us
        delayMicroseconds(30);

        if(PINF & _BV(DHT11_PIN))
        result |=(1<<(7-i));
        while((PINF & _BV(DHT11_PIN)));  // wait '1' finish
    }
    return result;
}

  ///Other Temp&Humi
  byte read_dht11_dat2()
{
    byte i = 0;
    byte result=0;
    for(i=0; i< 8; i++){

        while(!(PINF & _BV(DHT11_PINN)));  // wait for 50us
        delayMicroseconds(30);

        if(PINF & _BV(DHT11_PINN))
        result |=(1<<(7-i));
        while((PINF & _BV(DHT11_PINN)));  // wait '1' finish
    }
    return result;
}
  //////////

/////////////////////////SD CARD ///////////////////////////////////////////////
//File myFile;

// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const int chipSelect = 8;
////////////////////////////////////////////////////////////

/// Geiger
RadiationWatch radiationWatch;
RadiationWatch radiationWatch2(4,5);

void onRadiation()
{
  radiationB = radiationWatch.uSvh();

  for(int jj=0;jj<2;jj++){
      digitalWrite(30, HIGH);
      delay(500);
      digitalWrite(30, LOW);
      delay(500);
  }
  // Serial.println(radiationWatch.uSvhError());
}

void onRadiation2()
{
  radiationB2 = radiationWatch2.uSvh();

  for(int jj=0;jj<4;jj++){
      digitalWrite(30, HIGH);
      delay(250);
      digitalWrite(30, LOW);
      delay(250);
  }
  // Serial.println(radiationWatch.uSvhError());
}

void onNoise()
{
  Serial.println("Argh, noise, please stop moving");
}

// Geiger end

void setup()
{

     // setup blinking lines 
     pinMode(30, OUTPUT); 
     pinMode(31, OUTPUT);      

    ////
    
    Serial.println("****HP20x_dev demo by seeed studio****\n");
    Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
    /* Power up,delay 150ms,until voltage is stable */
    delay(150);
    /* Reset HP20x_dev */
    HP20x.begin();
    delay(100);
  
    /* Determine HP20x_dev is available or not */
    ret = HP20x.isAvailable();
    if(OK_HP20X_DEV == ret)
    {
    Serial.println("HP20x_dev is available.\n");    
    }
    else
    {
    Serial.println("HP20x_dev isn't available.\n");
    }
    
    DDRF |= _BV(DHT11_PIN);
    PORTF |= _BV(DHT11_PIN);

    ///
    DDRF |= _BV(DHT11_PINN);
    PORTF |= _BV(DHT11_PINN);
    ///

    Serial.begin(9600);
    Serial.println("Ready");

    if (!tempsensor.begin()) {
        Serial.println("Couldn't find MCP9808!");
        while (1);
     }

    ///////////////////////////////SD CARD///////////////////////////////////
    // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("baloon.csv", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to baloon.csv...");
    
    myFile.println("Humdity_out(%);Temp_in (C);Temp_out (C) ;Pressure(hPa);Radiation no shield(uSv/h);Radiation with shield(uSv/h);Altitude(m)");
    // close the file:
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening baloon.csv");
  }

  // reading
  if (myFile) {
    Serial.println("baloon.csv:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    // myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening baloon.csv");
  }
  ///////////////////////////////////////////////////////////////////////////
  // Geiger
  Serial.begin(9600);
  radiationWatch.setup();
  // Register the callbacks.
  radiationWatch.registerRadiationCallback(&onRadiation);
  radiationWatch.registerNoiseCallback(&onNoise);
  //

  // Geiger 2
  radiationWatch2.setup();
  // Register the callbacks.
  radiationWatch2.registerRadiationCallback(&onRadiation2);
  radiationWatch2.registerNoiseCallback(&onNoise);
}

void loop()
{

    radiationWatch.loop(); 
    radiationWatch2.loop(); 
        
  ////////////////////////////Humi_out 
    byte dht11_datt[3];
    byte dht11_inn;
    byte ii;
    // start condition
    // 1. pull-down i/o pin from 18ms
    PORTF &= ~_BV(DHT11_PINN);
    delay(18);
    PORTF |= _BV(DHT11_PINN);
    delayMicroseconds(40);

    DDRF &= ~_BV(DHT11_PINN);
    delayMicroseconds(40);

    dht11_inn = PINF & _BV(DHT11_PINN);

    if(dht11_inn){
        Serial.println("dht11 start condition 1 not met");
        return;
    }
    delayMicroseconds(80);

    dht11_inn = PINF & _BV(DHT11_PINN);

    if(!dht11_inn){
        Serial.println("dht11 start condition 2 not met");
        return;
    }
    delayMicroseconds(80);
    // now ready for data reception
    for (ii=0; ii<5; ii++)
    dht11_datt[ii] = read_dht11_dat2();

    DDRF |= _BV(DHT11_PINN);
    PORTF |= _BV(DHT11_PINN);

    byte dht11_check_summ = dht11_datt[0]+dht11_datt[1];
    // check check_sum
    if(dht11_datt[2]!= dht11_check_summ)
    {
        Serial.println("DHT11 checksum error");
    }  
  ////// End Humi_out////////////////////

  //////////TEMP_in //////////////////////////
    byte dht11_dat[5];
    byte dht11_in;
    byte i;
    // start condition
    // 1. pull-down i/o pin from 18ms
    PORTF &= ~_BV(DHT11_PIN);
    delay(18);
    PORTF |= _BV(DHT11_PIN);
    delayMicroseconds(40);

    DDRF &= ~_BV(DHT11_PIN);
    delayMicroseconds(40);

    dht11_in = PINC & _BV(DHT11_PIN);

    if(dht11_in){
        Serial.println("dht11 start condition 1 not met");
        return;
    }
    delayMicroseconds(80);

    dht11_in = PINF & _BV(DHT11_PIN);

    if(!dht11_in){
        Serial.println("dht11 start condition 2 not met");
        return;
    }
    delayMicroseconds(80);
    // now ready for data reception
    for (i=0; i<5; i++)
    dht11_dat[i] = read_dht11_dat();

    DDRF |= _BV(DHT11_PIN);
    PORTF |= _BV(DHT11_PIN);

    byte dht11_check_sum = dht11_dat[0]+dht11_dat[1]+dht11_dat[2]+dht11_dat[3];
    // check check_sum
    if(dht11_dat[4]!= dht11_check_sum)
    {
        Serial.println("DHT11 checksum error");
    }
    ///////////////// end Temp_in ///////////////

    //sd block
    Serial.print("humdity_out = ");
    Serial.print(dht11_datt[0], DEC);
    Serial.print(".");
    Serial.print(dht11_datt[1], DEC);
    Serial.print("%  ");   
    
    myFile.print(dht11_datt[0], DEC);
    myFile.print(".");
    myFile.print(dht11_datt[1], DEC);
    myFile.print(";");
    
    Serial.print("temp_in = ");
    Serial.print(dht11_dat[2], DEC);
    Serial.print(".");
    Serial.print(dht11_dat[3], DEC);
    Serial.println("C  ");

    myFile.print(dht11_dat[2], DEC);
    myFile.print(".");
    myFile.print(dht11_dat[3], DEC);
    myFile.print(";");

    float c = tempsensor.readTempC();
   
    Serial.print("Temp_out: "); Serial.print(c); Serial.print("*C\t"); 
    myFile.print(c);
    myFile.print(";");

    char display[40];
    if(OK_HP20X_DEV == ret)
    { 
    Serial.println("------------------\n");
 
    long Pressure = HP20x.ReadPressure();
    Serial.println("Pressure:");
    float t = Pressure/100.0;
    Serial.print(t);
    Serial.println("hPa.\n");

    myFile.print(t);
    myFile.print(" ");
    myFile.print(";");

  Serial.println("A wild gamma ray appeared");
  Serial.print(radiationB);
  Serial.println(" uSv/h (no shield) ");
    myFile.print(radiationB);
    myFile.print(";");

  Serial.println("A wild gamma ray appeared");
  Serial.print(radiationB2);
  Serial.println(" uSv/h (with shield) ");
    myFile.print(radiationB2);
    myFile.print(";");
    
    long Altitude = HP20x.ReadAltitude();
    Serial.println("Altitude:");
    t = Altitude/100.0;
    Serial.print(t);
    Serial.println("m.\n");

    myFile.print(t);
    myFile.println(" ");
    Serial.println("\n\n------------------\n");

    // blinking green light: means the system is working
    for(int jj=0;jj<2;jj++){
        digitalWrite(31, HIGH);
        delay(500);
        digitalWrite(31, LOW);
        delay(500);
    }
    //////////////////////
    delay(10000);
  }
  //////////////////////////SD CARD////////////////////////////////////////////////
  myFile.close();
  myFile = SD.open("baloon.csv", FILE_WRITE);
  //  Serial.println("done.");   
  /////////////////////////////////////////////////////////////////////////////////
}



