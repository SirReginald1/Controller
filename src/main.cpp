#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// A4: SDA
// A5: SCL

//const uint8_t muxSIG = A1;
const uint8_t muxS0 = 2;
const uint8_t muxS1 = 3;
const uint8_t muxS2 = 4;
const uint8_t muxS3 = 5;

/* The number of connected channels to the MUX */
const uint8_t nb_mux_chan = 3;

uint16_t adcVals[nb_mux_chan];

Adafruit_ADS1115 ads;  // Create an ADS1115 object

//void receiveEvent(int value);

void selectChannel(uint8_t channel);

void readMUX();

void selectChannelFast(uint8_t channel);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //Wire.onReceive(receiveEvent);

  // ############ Setup MUX #############
  //pinMode(muxS0, OUTPUT); // Try this DDRD &= B
  //pinMode(muxS1, OUTPUT);
  //pinMode(muxS2, OUTPUT);
  //pinMode(muxS3, OUTPUT);
  DDRD &= B00011110; // & because default state is up???
  //DDRD &= B00000100;
  if (!ads.begin()) {
    Serial.println("ADS1115 not detected! Check wiring.");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range (default)
}

void loop() {
  //Wire.requestFrom(8, 2); // request 2 bytes from peripheral device #8
  //while (Wire.available()) { // peripheral may send less than requested
  //  uint16_t adcChan0 = Wire.read(); // receive a byte as int16
  //  Serial.print(adcChan0);         // print the character
  //}
  //if(Serial.available() > 0){
  //  selectChannel(Serial.parseInt());
  //}
  //Serial.println(analogRead(muxSIG));
  //int16_t rawValue = ads.readADC_SingleEnded(0);  // Read from A0
  //float voltage = rawValue * 0.1875 / 1000;  // Convert to voltage (mV to V)
  //  
  //Serial.print("Raw Value: "); Serial.print(rawValue);
  //Serial.print(" | Voltage: "); Serial.print(voltage, 4);
  //Serial.println("V");
  //
  //  delay(500);
  selectChannel(1);
  readMUX();
  for(uint8_t i=0;i<nb_mux_chan;i++){
    Serial.print("Chan");Serial.print(i);Serial.print(": ");Serial.println(adcVals[i]);
  }
  Serial.println("----");
}

//void receiveEvent(int value) {  
//}
/**
 * Reads the values from the multiplexer
 */
void readMUX(){
  for(uint8_t channel=0; channel<nb_mux_chan; channel++){
    //selectChannel(channel);
    selectChannelFast(channel);
    adcVals[channel] = ads.readADC_SingleEnded(0);
  }
}

/**
 * Selects the multiplexer channel to read from
 */
void selectChannel(uint8_t channel){
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}

void selectChannelFast(uint8_t channel){
  DDRD &= (byte)channel;
}