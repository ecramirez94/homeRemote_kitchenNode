#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include "DHT.h"
#include "printf.h"


#define TEMPLED 15
#define HUMIDLED 14
#define LEDPIN 13
#define FANPIN 4
#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// Initalize NRF24
RF24 radio(9,10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

float h = 0.0;
float t = 0.0;
float f = 0.0;

// The difference between the respective values is the hysteresis for that metric.
float onTemp = 24.0;
float offTemp = 21.0;
float onHumidity = 48.0;
float offHumidity = 40.0;

#define IDL 0
#define HUMIDITY 1
#define TEMPERATURE 2
#define REMOTE_ON 3
#define REMOTE_OFF 4
#define REMOTE_DELAY 5

uint8_t reason = 0;

bool serCom = false;
bool fanOn = false;
bool newCommand = false;
bool reportData = false;
bool readFromDHT = true;

uint8_t remoteCommand = 0;  // Start in auto mode
#define FAN_AUTO 0
#define FAN_ON 1
#define FAN_OFF 2
#define FAN_DELAY 3
#define GET_DATA 4
#define STOP_DATA 5

void setup() 
{
  pinMode(FANPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(TEMPLED, OUTPUT);
  pinMode(HUMIDLED, OUTPUT);
  
  digitalWrite(FANPIN, LOW);
  digitalWrite(LEDPIN, LOW);
  digitalWrite(TEMPLED, LOW);
  digitalWrite(HUMIDLED, LOW);

  setupTimer();
  
  dht.begin();
  radio.begin();  
  enableSerial(true); // Set to true to turn on reporting over serial com
  printf_begin();

  radio.setRetries(15, 15);
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();
  radio.printDetails();
}

void loop() 
{
  if (readFromDHT)
  {
    readDHT();

    if(serCom)
      printReport();

    if(reportData)
      sendReport();

    readFromDHT = false;
  }

  if(radio.available())
    receiveData();
  
  chooseState();
}

// ====== Funtions =====
void readDHT(void)
{
  flashLED();
    
  h = dht.readHumidity();
  t = dht.readTemperature();
  f = dht.readTemperature(true);
}

void chooseState(void)
{
  switch(remoteCommand)
  {
    case FAN_AUTO:
      if (h >= onHumidity && reason == IDL)
      {
        reason = HUMIDITY;
        digitalWrite(FANPIN, HIGH);
        digitalWrite(HUMIDLED, HIGH);
      }
      else if (t >= onTemp && reason == IDL)
      {
        reason = TEMPERATURE;
        digitalWrite(FANPIN, HIGH);
        digitalWrite(TEMPLED, HIGH);
      }
    
      switch (reason)
      {
        case HUMIDITY:
          if (h <= offHumidity)
          {
            digitalWrite(FANPIN, LOW);
            digitalWrite(HUMIDLED, LOW);
            reason = IDL;
          }
        break;
        case TEMPERATURE:
          if (t <= offTemp)
          {
            digitalWrite(FANPIN, LOW);
            digitalWrite(TEMPLED, LOW);
            reason = IDL;
          }
        break;
        case IDL:
          digitalWrite(FANPIN, LOW);
          digitalWrite(TEMPLED, LOW);
        break;
      }
    break;
    case FAN_ON:
      if(newCommand)
      {
        digitalWrite(FANPIN, HIGH);
        digitalWrite(HUMIDLED, HIGH);
        digitalWrite(TEMPLED, HIGH);
        reason = REMOTE_ON;
        newCommand = false;
      }
    break;
    case FAN_OFF:
      if(newCommand)
      {
        digitalWrite(FANPIN, LOW);
        digitalWrite(HUMIDLED, LOW);
        digitalWrite(TEMPLED, LOW);
        reason = REMOTE_OFF;
        newCommand = false;
      }
    break;
    default:
      reason = IDL;
      remoteCommand = FAN_AUTO;
      newCommand = false;
    break;
  }
}

void receiveData(void)
{
  // Positions in the payload
  #define MESSAGE_TYPE 0
  #define COMMAND 1

  /*
   *  The received data will arrive in either of the following forms
   *    1. Data
   *    2. Control
   *    3. Status
   *    
   *  What form it is, is determined by the first byte (char) of the payload. 
   *  Then the payload follows.
   *  
   *  For Data:
   *    message[0] = The message type ('d' for data)
   *    message[1] = The data type ('h' for humidity, 't' for temperature in Celcius, 'f' for temperature in Fahrenheit)
   *    message[2-5] = The actual data  
   *    
   *  For Command:
   *    message[0] = The message type ('c' for command) 
   *    message[1] = The actual command
   *    
   *  For Status:
   *    message[0] = The message type ('s' for status)
   *    message[1] = The status type ('m' for fan mode, 's' for current status)
   *    message[2] = The actual mode/current status 
   *                            mode: ('n' for the fan on via remote, 'f' for the fan off via remote, 'd' for the delay mode, 'a' for auto mode (normal)
   *                            current status: ('H' Humidity turned on the fan, 'T' Temperature turned on the fan, 'O' The fan is currently off
   */

  char message[2];
  radio.read(&message, sizeof(message)); 
  
  char messageType = message[MESSAGE_TYPE];
  
  if (messageType == 'c')  // It is an incoming command
  {
    char com = message[COMMAND];  

    if (com == 'a')
    {
      allLightsOff();
      remoteCommand = FAN_AUTO;
      reason = IDL;
    }
    else if (com == 'n')
      remoteCommand = FAN_ON;
    else if (com == 'f')
      remoteCommand = FAN_OFF;
    else if (com == 'd')
      remoteCommand = FAN_AUTO; // *Change when delay feature is implemented*
    else if (com == 'g')
      reportData = true;
    else if (com == 's')
      reportData = false;
        
    newCommand = true;
  }
  
  radio.startListening();
  
//  if(radio.available())
//  {
//    uint8_t v = 0;
//    bool done = false;
//
//    while (!done)
//      done = radio.read(&v, sizeof(uint8_t));
//       
//    remoteCommand = v;
//    newCommand = true;
//    reason = IDL;
//    Serial.print(F("Data: "));
//    Serial.println(remoteCommand);
//  }
}

void flashLED(void)
{
  digitalWrite(TEMPLED, HIGH);
  delay(50);
  digitalWrite(TEMPLED, LOW);
}

void enableSerial(bool en)
{
  if(en)
  {
    Serial.begin(57600);
    serCom = true;
  }
}

bool sendReport(void)
{
  bool ok = false;

  char reportPayload[10] = {'d', chopFloat(h, 3), chopFloat(h, 2), chopFloat(h, 1), chopFloat(h, 0), 
                                chopFloat(t, 3), chopFloat(t, 2), chopFloat(t, 1), chopFloat(t, 0), 
                                                                                   reasonToChar()};
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.stopListening();
  
  ok = radio.write(&reportPayload, sizeof(reportPayload));

  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();
  return ok;
}

char chopFloat(float input, uint8_t pos)
{
  uint32_t in = input;
  
  switch (pos)
  {
    case 3:
      return char(in >> 24);
    break;
    case 2:
      in = in << 8;
      return char(in >> 24);
    break;
    case 1:
      in = in << 16;
      return char(in >> 24);
    break;
    case 0:
      in = in << 24;
      return char(in >> 24);
    break;
    default:
      return 0;
    break;
  }
}

void printReport(void)
{
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.print(F("°F"));
  Serial.print(F(" Reason: "));
  Serial.println(Reason());
}

String Reason(void)
{
  switch (reason)
  {
    case IDL:
      return F("Idle");
    break;
    case HUMIDITY:
      return F("Humidity");
    break;
    case TEMPERATURE:
      return F("Temperature");
    break;
    case REMOTE_ON:
      return F("Remote on");
    break;
    case REMOTE_OFF:
      return F("Remote off");
    break;
    case REMOTE_DELAY:
      return F("Remote Delay");
    break;
    default:
      return F("Idle");
    break;
  }
}

char reasonToChar(void)
{
  return char(reason);
}

void allLightsOff(void)
{
  digitalWrite(FANPIN, LOW);
  digitalWrite(HUMIDLED, LOW);
  digitalWrite(TEMPLED, LOW);
}

void setupTimer(void) // Setup timer to generate 1Hz pulses
{
  cli();  // Disable interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  // set compare match register for 2hz increments
  OCR1A = 31248;  // (16 * 10^6) / (1 * 1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
  sei();  // Enable interrupts
}

ISR(TIMER1_COMPA_vect)  // Timer ISR
{ 
  readFromDHT = true;
}
