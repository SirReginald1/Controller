#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ###########################################################
// ################### Declaring constants ###################
// ###########################################################

// ################## MUX constants ##########################
const uint8_t muxS0 = 19;
const uint8_t muxS1 = 18;
const uint8_t muxS2 = 5;
const uint8_t muxS3 = 17;
/* The number of connected channels to the MUX */
//const uint8_t nbChanMux0 = 4;
/* Number of variable resistors */
const uint8_t nbVariableResistors = 2;
/* Totale number of variable resistors */
//const uint8_t nbTotalVariableResistors = nbVariableResistors;
/* Maps the ADC and multiplexer addresses for each variable resistor. 
  Dim0: variable resistor number 
  Dim1: 0 = ADC address, 1 = multiplexer address
  */
const uint8_t muxAdcVariableResistorMap[nbVariableResistors][2] = {
  {0, 0},
  {0, 1}
};
/* Number of joysticks */
const uint8_t nbJoysticks = 1;
/* Maps the ADC and multiplexer addresses for each joystick.
  Dim0: joystick number
  Dim1: 0 = x, 1 = y
  Dim2: 0 = ADC address, 1 = multiplexer address
  */
const uint8_t muxAdcJoystickMap[nbJoysticks][2][2] = {
  {{0, 2}, {0, 3}}
};
// ################# rotary encoder pins #################
/* The shift register load pin for rotary encoders */
const int rotaryEncoderLoadPin = 27;
/* The shift register clock in pin for rotary encoders */
const int rotaryEncoderClockInPin = 26;
/* The pin that reads CLK pin from the rotary encoders */
const int rotaryEncoderClkReadPin = 16;
/* The pin that reads DT pin from the rotary encoders */
const int rotaryEncoderDtReadPin = 4;
/* The number of rotary encoders to be read */
const uint8_t nbRotaryEncoders = 2;
// ############### Buttons shift register vars #######################
/* The pin used to load the data into the shift registers registers. */
const int loadPin = 14; // Latch pin
/* The pin used to read the data. */
const int dataInPin = 12;
/* The clock signal used to shift data out of the register. 
The clockEnablePin must be set to low to enable the shifting of data. */
const int clockInPin = 13;
/* Number of buttons */
const uint8_t nbButtons = 3 + nbRotaryEncoders; // Because other elements can be pressed
/* The number of shift registers mounted in series for button inputs */
const int nbShiftRegistersInSeries = 3;
/* The number of bits to be shifted in for the button shift registers */
const int nbBitsBtnShiftRegisters = 8 * nbShiftRegistersInSeries;
// ################# General constants #################
/* Number of controle element types */
const uint8_t nbControleElementTypes = 4;
// Temp variables
const uint8_t max1 = nbVariableResistors>=(nbJoysticks * 2) ? nbVariableResistors : (nbJoysticks * 2);
const uint8_t max2 = nbRotaryEncoders>=nbButtons ? nbRotaryEncoders : nbButtons;
/* The maximum number of controle element of any type. Only used to define the size of
element ID array.
*/
const uint8_t maxNbControleElementTypes = max1>=max2 ? max1 : max2;
/* The array containing the ID representing each element on the controller.
  Dim0: Element type:
        0 = Variable Resistors
        1 = Joysticks
        2 = Rotary encoders
        3 = Buttons
*/
uint8_t controleElementIds[nbControleElementTypes][maxNbControleElementTypes];
// ##################### Serial constants ########################
/* The maximum message buffer size. (nb bytes per message * max nb messages) */
const uint8_t maxMessageBufferSizeOut = 3 * 10;


// ###########################################################
// ################### Declaring variables ###################
// ###########################################################

// ################# Buttons vars #######################
int16_t buttonValues[nbButtons];
// ################## MUX vars ##########################
/* Array containing the last recorded values for each of the variable resistors of the controller */
int variableResistorValues[nbVariableResistors];
/* Array containing the last recorded values for each of the joysticks of the controller.
  Dim0: joystick number
  Dim1: x=0, y=1
  */
int16_t joystickValues[nbJoysticks][2];
// ################# rotary encoder vars #################
/* The value of the currently polled rotary encoder CLK channel  */
int rotaryEncoderClkCurrent = 1;
/* The counters for each rotary encoder */
int rotaryCounters[nbRotaryEncoders];
/* Array containing the last recorded value for all the CLK pins */
int clkLastState[nbRotaryEncoders];
/* Array containing the current recorded value for all the CLK pins */
int clkCurrentState[nbRotaryEncoders]; //!!!!!!!!!!!!!!!!!!!!!! TEST !!!!!!!!!!!!!!!!!!!!!!!!!!!!
/* Array containing the current recorded value for all the the DT pins */
int dtCurrentState[nbRotaryEncoders]; //!!!!!!!!!!!!!!!!!!!!!! TEST !!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ##################### ADC vars ########################
/* The ADC object. */
Adafruit_ADS1115 ads;  // Create an ADS1115 object
// ##################### General vars ########################
/* The buffer variable used for reading inputs */
int16_t readBuffer = 0;
/* The length of the message to send. If message = 0 no message to send */
uint8_t messageLenOut = 0;
/* The buffer containing the message to be sent */
byte messageBufferOut[maxMessageBufferSizeOut];

// ###################### fuction declarations ######################
void setupRotaryEncoders();
void readVariableResistors();
void selectChannel(uint8_t channel);
void readRotrayEncoders();
void readButtons();
void readJoysticks();
void readRotaryEncoders();
void setupControleElementIds();
void setupButtons();
void setupVariableResistors();
void setupRotaryEncoders();
void setupJoysticks();
// ############################# TEST ########################
void proccessRotary();
// ############################# TEST ########################
// ################ Interrupt function #########################
// ################ Test variables #########################
//int16_t 


// ################ SETUP #########################
void setup() {
  Serial.begin(115200);
  // ############ Setup MUX #############
  pinMode(muxS0, OUTPUT); // Try this DDRD &= B
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);
  // ############ Setup button shift registers #############
  pinMode(loadPin, OUTPUT);
  pinMode(clockInPin, OUTPUT);
  pinMode(dataInPin, INPUT);
  // ############ Setup rotary encoder shift registers #############
  pinMode(rotaryEncoderLoadPin, OUTPUT);
  pinMode(rotaryEncoderClockInPin, OUTPUT);
  pinMode(rotaryEncoderClkReadPin, INPUT);
  pinMode(rotaryEncoderDtReadPin, INPUT);
  // ############# ADC setup #############
  Wire.begin(); // I2C library for ADC
  if (!ads.begin()) {
    Serial.println("ADS1115 not detected! Check wiring.");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range (default)
  setupControleElementIds();
  setupButtons();
  setupVariableResistors();
  setupRotaryEncoders();
  setupJoysticks();
  // ################ Test printouts #################
  //for(int i=0;i<nbRotaryEncoders;i++){
  //  Serial.print("Counter val ");Serial.print(i);Serial.print(": ");Serial.println(rotaryCounters[i]);
  //}
  //delay(3000);
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Send id map to receiver and
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DO THIS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Set both rotary encoder input pins as interrupt pins so that when an pin changes it trigers read rotary encoder read
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DO THIS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ############################ LOOP ###############################
void loop() {
  readRotaryEncoders();
  readVariableResistors();
  readRotaryEncoders();
  readJoysticks();
  readRotaryEncoders();
  readButtons();
  readRotaryEncoders();
  proccessRotary();
  // Check if message needs to be sent
  if(messageLenOut > 0){
    Serial.println("Send:");
    for(uint8_t i=0;i<messageLenOut;i+=3){
      Serial.print("ID: ");Serial.print(messageBufferOut[i]);Serial.print(", message: ");
      Serial.println((((int16_t)messageBufferOut[i + 1]) << 8) | ((int16_t)messageBufferOut[i + 2]));
      //Serial.println((((uint16_t)messageBufferOut[i + 1]) << 8));
      //Serial.println(((uint16_t)messageBufferOut[i + 2]));
    }
  }
  // Reset output message length
  messageLenOut = 0;
}


// ###################################################################################
// ################################# FUNCTION DEFINITIONS ############################
// ###################################################################################

/**
 * Instanciates values for the button variables.
 */
void setupButtons(){
  for(uint8_t i=0;i<nbButtons;i++){
    buttonValues[i] = 0;
  }
}

/**
 * Instanciates values for the rotary encoder variables.
 */
void setupRotaryEncoders(){
  for(uint8_t i=0;i<nbRotaryEncoders;i++){
    rotaryCounters[i] = 0;
    clkLastState[i] = 1;
  }
}

/**
 * Instanciates values for the variable resistor variables.
 */
void setupVariableResistors(){
  for(uint8_t i=0;i<nbVariableResistors;i++){
    variableResistorValues[i] = 0;
  }
}

/**
 * Instanciates values for the joystick variables.
 */
void setupJoysticks(){
  for(uint8_t i=0;i<nbJoysticks;i++){
    selectChannel(muxAdcJoystickMap[i][0][1]);
    joystickValues[i][0] = ads.readADC_SingleEnded(muxAdcJoystickMap[i][0][0]);
    selectChannel(muxAdcJoystickMap[i][1][1]);
    joystickValues[i][1] = ads.readADC_SingleEnded(muxAdcJoystickMap[i][1][0]);
  }
}

/**
 * Instanciates controle element IDs.
 */
void setupControleElementIds(){
  //uint8_t out[nbControleElementTypes][maxNbControleElementTypes];
  uint8_t idCounter = 0;
  // Set variable resistor ids
  for(uint8_t i=0;i<nbVariableResistors;i++){
    controleElementIds[0][i] = idCounter;
    idCounter++;
  }
  // Set joysticks ids
  for(uint8_t i=0;i<(nbJoysticks * 2);i++){
    controleElementIds[1][i] = idCounter;
    idCounter++;
  }
  // Set rotary encoders ids
  for(uint8_t i=0;i<nbRotaryEncoders;i++){
    controleElementIds[2][i] = idCounter;
    idCounter++;
  }
  // Set buttons ids
  for(uint8_t i=0;i<nbButtons;i++){
    controleElementIds[3][i] = idCounter;
    idCounter++;
  }
  //controleElementIds = temp;
  //return out;
}

/**
 * Instanciates serial related variables.
 */
/*
void setupSerial(){
  for(uint8_t i=0;i<maxMessageBufferSizeOut;i++){
    messageBufferOut[i] = 0;
  }
}
*/

/**
 * Adds message to output serial buffer only if there is size left in the buffer.
 * Should only called when a value is changed.
 * @param controleElementId The ID of the element that has had its value updated.
 * @param value The value of the changed element.
 */
void addMessageToBuffer(uint8_t controleElementId, int16_t value){
  if(messageLenOut < maxMessageBufferSizeOut){
    messageBufferOut[messageLenOut] = controleElementId;
    messageBufferOut[messageLenOut + 1] = (uint8_t)(value >> 8);
    messageBufferOut[messageLenOut + 2] = (uint8_t)value;
    messageLenOut += 3;
  }
}



/**
 * Function called to pole rotary encoders and increment counter values.
 */
void readRotaryEncoders(){
  GPIO.out_w1tc = ((uint32_t)1 << rotaryEncoderLoadPin); // Set pin low fast
  GPIO.out_w1ts = ((uint32_t)1 << rotaryEncoderLoadPin); // Set pin high fast
  for (int i = 0; i < nbRotaryEncoders; i++) {
    // Read both values
    //rotaryEncoderClkCurrent = (GPIO.in >> rotaryEncoderClkReadPin) & 0x1;
    clkCurrentState[i] = (GPIO.in >> rotaryEncoderClkReadPin) & 0x1;
    dtCurrentState[i] = (GPIO.in >> rotaryEncoderDtReadPin) & 0x1;
    /*
    // Set counter values
    if (rotaryEncoderClkCurrent != clkLastState[i]) {
      if (/*rotaryEncoderDtCurrent*//*((GPIO.in >> rotaryEncoderDtReadPin) & 0x1) != rotaryEncoderClkCurrent) {
        if(rotaryEncoderClkCurrent == 1){
          rotaryCounters[i]++;  // Clockwise
          addMessageToBuffer(controleElementIds[2][i], rotaryCounters[i]);
        }
      } else {
        if(rotaryEncoderClkCurrent == 0){
          rotaryCounters[i]--;  // Counterclockwise
          addMessageToBuffer(controleElementIds[2][i], rotaryCounters[i]);
        }
      }
      //Serial.print("Count ");Serial.print(i);Serial.print(": ");Serial.println(rotaryCounters[i]); // DEBUG
      clkLastState[i] = rotaryEncoderClkCurrent;
    }
      */
    // Shift out the next bit to QH
    GPIO.out_w1ts = ((uint32_t)1 << rotaryEncoderClockInPin); // Set pin high fast
    GPIO.out_w1tc = ((uint32_t)1 << rotaryEncoderClockInPin); // Set pin low fast
  }
}

void proccessRotary(){
  // ################################## Testing #####################################
  for (int i = 0; i < nbRotaryEncoders; i++) {
    // Set counter values
    if (clkCurrentState[i] != clkLastState[i]) {
      if (dtCurrentState[i] != clkCurrentState[i]) {
        if(clkCurrentState[i] == 1){
          rotaryCounters[i]++;  // Clockwise
          addMessageToBuffer(controleElementIds[2][i], rotaryCounters[i]);
        }
      } else {
        if(clkCurrentState[i] == 0){
          rotaryCounters[i]--;  // Counterclockwise
          addMessageToBuffer(controleElementIds[2][i], rotaryCounters[i]);
        }
      }
      //Serial.print("Count ");Serial.print(i);Serial.print(": ");Serial.println(rotaryCounters[i]); // DEBUG
      clkLastState[i] = clkCurrentState[i];
    }
  }
  // ################################## Testing #####################################
}

/**
 * Reads the data from the button shift registers.
 */
void readButtons(){
  GPIO.out_w1tc = ((uint32_t)1 << loadPin); // Set pin low fast
  GPIO.out_w1ts = ((uint32_t)1 << loadPin); // Set pin high fast
  for (uint8_t i = 0; i < nbButtons; i++) {
    //int bit = (GPIO.in >> dataInPin) & 0x1;//digitalRead(dataInPin);
    // Shift out the next bit to QH
    //out = (((GPIO.in >> dataInPin) & 1) << i) | out;
    readBuffer = (GPIO.in >> dataInPin) & 0x1;
    //Serial.print("Buffer: ");Serial.println(readBuffer);
    if(readBuffer != buttonValues[i]){
      buttonValues[i] = readBuffer;
      addMessageToBuffer(controleElementIds[3][i], buttonValues[i]);
    }
    GPIO.out_w1ts = ((uint32_t)1 << clockInPin); // Set pin high fast
    GPIO.out_w1tc = ((uint32_t)1 << clockInPin); // Set pin low fast
  }
}

/**
 * Polls values from variable resistors and place them in variableResistorValues array.
 */
void readVariableResistors(){
  for(uint8_t i=0;i<nbVariableResistors;i++){
    selectChannel(muxAdcVariableResistorMap[i][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcVariableResistorMap[i][0]);
    if(abs(readBuffer - variableResistorValues[i]) > 5){
      variableResistorValues[i] = readBuffer;
      addMessageToBuffer(controleElementIds[0][i], variableResistorValues[i]);
    }
  }
}

/**
 * Polls values from joysticks and place them in the joystickValues array.
 */
void readJoysticks(){
  for(uint8_t i=0;i<nbJoysticks;i+=2){
    selectChannel(muxAdcJoystickMap[i][0][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][0][0]);
    //Serial.print("Buffer: ");Serial.print(readBuffer);Serial.print(", val: ");Serial.println(joystickValues[i][0]);
    if(abs(readBuffer - joystickValues[i][0]) > 2){
      //Serial.println(readBuffer);
      joystickValues[i][0] = readBuffer;
      addMessageToBuffer(controleElementIds[1][i], joystickValues[i][0]);
    }
    selectChannel(muxAdcJoystickMap[i][1][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][1][0]);
    if(abs(readBuffer - joystickValues[i][1]) > 2){
      joystickValues[i][1] = readBuffer;
      addMessageToBuffer(controleElementIds[1][i+1], joystickValues[i][1]);
    }
  }
}

/**
 * Selects the multiplexer channel to read from
 * @param channel The channel 
 */
void selectChannel(uint8_t channel){
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
  //GPIO.out_w1ts = ((uint32_t)1 << clockInPin); // Set pin high fast
  //GPIO.out_w1tc = ((uint32_t)1 << clockInPin); // Set pin low fast

}