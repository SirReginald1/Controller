#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <Arduino.h>
#include <Wire.h>
#define PCF8574_LOW_LATENCY
#include "PCF8574.h"
#include <Adafruit_ADS1X15.h>


// ------------------------------------------------------------------
// ######################## GPIO DEFINITIONS ########################
// ------------------------------------------------------------------
// ################## MUX constants #######################
#define MUX_S0_PIN 19 
#define MUX_S1_PIN 18
#define MUX_S2_PIN 5
#define MUX_S3_PIN 17
// Default I2C pins
// GPIO 21 (SDA)
// GPIO 22 (SCL)
// To change I2C pins
// Wire.begin(SDA, SCL);
/* ADC address */
#define ADC_I2C_ADDRESS 0x48
// ############## Rotary encoder constants ################
// Addresses go from 0x38 to 0x3F
/* I2C address for rotary encoder 0 to 3 using PCF8574 shifter */
#define ENCODER_I2C_ADDRESS_0 0x20
/* Interrupt pin for encoder CLK PCF8574 shifter 0 */
#define ENC_INT_PIN_0_TO_3 32
/* The id of the PCF chip that handles encoders 0 to 4. Used in task notification value */
#define PCF_ID_ENC_0_TO_3 0
/* I2C address for rotary encoder 4 to 7 using PCF8574 shifter */
#define ENCODER_I2C_ADDRESS_1 0x21
/* Interrupt pin for encoder CLK PCF8574 shifter 1 */
#define ENC_INT_PIN_4_TO_7 26
/* The id of the PCF chip that handles encoders 5 to 8. Used in task notification value */
#define PCF_ID_ENC_4_TO_7 1
/* I2C address for rotary encoder 8 to 9 using PCF8574 shifter */
#define ENCODER_I2C_ADDRESS_2 0x22
/* Interrupt pin for encoder DT PCF8574 shifter 0 */
#define ENC_INT_PIN_8_TO_9 33
/* The id of the PCF chip that handles encoders 5 to 8. Used in task notification value */
#define PCF_ID_ENC_8_TO_9 2

// ################# Buttons constants ###################
/* I2C address for base buttons 0 to 8 PCF8574 shifter */
#define BUTTONS_ADDRESS_0 0x23
/* Interrupt pin for buttons 0 to 8 PCF8574 shifter */
#define BUTTONS_INT_PIN_0 33
/* I2C address for base buttons 9 to 16 PCF8574 shifter */
#define BUTTONS_ADDRESS_1 0x24
/* Interrupt pin for buttons 9 to 16 PCF8574 shifter */
#define BUTTONS_INT_PIN_1 33
/* I2C address for base buttons 17 to 24 PCF8574 shifter */
#define BUTTONS_ADDRESS_2 0x25
/* Interrupt pin for buttons 17 to 24 PCF8574 shifter */
#define BUTTONS_INT_PIN_2 33
// ------------------------------------------------------------------
// ######################## NB CONTROLE ELEMENTS ####################
// ------------------------------------------------------------------
// ######## Number of input elements constants ##########
/* The number of rotary encoders to be read */
#define NB_ROTARY_ENCODERS 10
/* Number of variable resistors */
#define NB_VARIABLE_RESISTORS 2
/* Number of joysticks */
#define NB_JOYSTICKS 1
/* Number of buttons */
#define NB_BASE_BUTTONS 3
/* The total number of pressable elements */
const int NB_TOTAL_BUTTONS = NB_BASE_BUTTONS + NB_ROTARY_ENCODERS + NB_JOYSTICKS;
// ################ General constants ###################
#define NB_CONTROLE_ELEMENT_TYPES 4

// ------------------------------------------------------------------
// ################## variable resistor vars ########################
// ------------------------------------------------------------------
/* The difference between the past recoded value and the presently read value
    before recording a value change. Only applied to variable resistors.
*/
#define VARIABLE_RESISTOR_ALLOWED_VARIANCE 10
/* The ADC object. */
Adafruit_ADS1115 ads;
/* Array containing the last recorded values for each of the variable resistors of the controller */
int variableResistorValues[NB_VARIABLE_RESISTORS] = {0};
/* Maps the ADC and multiplexer addresses for each variable resistor. 
  Dim0: variable resistor number 
  Dim1: 0 = ADC address, 1 = multiplexer address
  */
const uint8_t muxAdcVariableResistorMap[NB_VARIABLE_RESISTORS][2] = {
  {0, 0},
  {0, 1},
  // TODO: Set to proper values after testing
  /*
  {0, 0},
  {0, 1},
  {0, 0},
  {0, 1},
  {0, 0},
  {0, 1},
  {0, 0},
  {0, 1},
  */
};
/* The handle for the task responsible for reading the rotary encoders */
TaskHandle_t readVariableResistorsAndJoysticksTaskHandle = NULL;
// #################### joystick vars ####################
/* Array containing the last recorded values for each of the joysticks of the controller.
  Dim0: joystick number
  Dim1: x=0, y=1
  */
int16_t joystickValues[NB_JOYSTICKS][2];
/* Maps the ADC and multiplexer addresses for each joystick.
  Dim0: joystick number
  Dim1: 0 = x, 1 = y
  Dim2: 0 = ADC address, 1 = multiplexer address
  */
const uint8_t muxAdcJoystickMap[NB_JOYSTICKS][2][2] = {
  {{0, 2}, {0, 3}}
};
// ------------------------------------------------------------------
// ####################### ROTARY ENCODER VARS ######################
// ------------------------------------------------------------------
/* Array containing the PCF8574 pins each of the rotary encoder A pins are connected to. */
const int encoderAPins[NB_ROTARY_ENCODERS] = {
  P0,
  P2,
  P4,
  P6,
  P0,
  P2,
  P4,
  P6,
  P0,
  P2,
};


/* Array containing the PCF8574 pins each of the rotary encoder B pins are connected to. */
const int encoderBPins[NB_ROTARY_ENCODERS] = {
  P1,
  P3,
  P5,
  P7,
  P1,
  P3,
  P5,
  P7,
  P1,
  P3,
};


void ICACHE_RAM_ATTR updateEncoder0To3();
void ICACHE_RAM_ATTR updateEncoder4To7();
void ICACHE_RAM_ATTR updateEncoder8To9();


// Initialize the PCF8574 library with the I2C addresses, interrupt pins, and ISRs
PCF8574 pcfEncoder0(ENCODER_I2C_ADDRESS_0, ENC_INT_PIN_0_TO_3, updateEncoder0To3);
PCF8574 pcfEncoder1(ENCODER_I2C_ADDRESS_0, ENC_INT_PIN_0_TO_3, updateEncoder0To3);
PCF8574 pcfEncoder2(ENCODER_I2C_ADDRESS_0, ENC_INT_PIN_0_TO_3, updateEncoder0To3);
PCF8574 pcfEncoder3(ENCODER_I2C_ADDRESS_0, ENC_INT_PIN_0_TO_3, updateEncoder0To3);
PCF8574 pcfEncoder4(ENCODER_I2C_ADDRESS_1, ENC_INT_PIN_4_TO_7, updateEncoder4To7);
PCF8574 pcfEncoder5(ENCODER_I2C_ADDRESS_1, ENC_INT_PIN_4_TO_7, updateEncoder4To7);
PCF8574 pcfEncoder6(ENCODER_I2C_ADDRESS_1, ENC_INT_PIN_4_TO_7, updateEncoder4To7);
PCF8574 pcfEncoder7(ENCODER_I2C_ADDRESS_1, ENC_INT_PIN_4_TO_7, updateEncoder4To7);
PCF8574 pcfEncoder8(ENCODER_I2C_ADDRESS_2, ENC_INT_PIN_8_TO_9, updateEncoder8To9);
PCF8574 pcfEncoder9(ENCODER_I2C_ADDRESS_2, ENC_INT_PIN_8_TO_9, updateEncoder8To9);
/* Reference array for PCF encoder objects. */
PCF8574 pcfEncoders[NB_ROTARY_ENCODERS] = {
  pcfEncoder0,
  pcfEncoder1,
  pcfEncoder2,
  pcfEncoder3,
  pcfEncoder4,
  pcfEncoder5,
  pcfEncoder6,
  pcfEncoder7,
  pcfEncoder8,
  pcfEncoder9,
};
/* The value of the currently polled rotary encoder DT channel  */
int rotaryEncoderDtCurrent = 0;
/* The counters for each rotary encoder */
volatile long encoderCounters[NB_ROTARY_ENCODERS] = {0};
/* Array containing the last recorded value for all the CLK pins */
int dtLastState[NB_ROTARY_ENCODERS] = {0};
/* The handle for the task responsible for reading the rotary encoders */
TaskHandle_t readEncodersTaskHandle = NULL;
// #################### general vars #####################
// Temp variables
const uint8_t max1 = NB_VARIABLE_RESISTORS>=(NB_JOYSTICKS * 2) ? NB_VARIABLE_RESISTORS : (NB_JOYSTICKS * 2);
const uint8_t max2 = NB_ROTARY_ENCODERS>=NB_TOTAL_BUTTONS ? NB_ROTARY_ENCODERS : NB_TOTAL_BUTTONS;
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
uint8_t controleElementIds[NB_CONTROLE_ELEMENT_TYPES][maxNbControleElementTypes];
// ##################### Message queue variables ########################
/* The length of the message to send. If message = 0 no message to send */
uint8_t messageLenOut = 0;
/* Struct representing the a message in the outputted message queue */
typedef struct {
    uint8_t elementId;
    long message;
} ElementMsg;
/* The queue containing the message to be sent */
QueueHandle_t messageOutQueue;
/* Flag indicating if a task that needs to be run */
BaseType_t xHigherPriorityTaskWoken;
/* The message buffer used to read values from the message queue.
   Only one buffer should be needed as there is only one piece of code
   that read the mesage queue.
*/
ElementMsg messageReadBuffer;
/* The handle for the task responsible for reading the buttons */
TaskHandle_t readButtonsTaskHandle = NULL;

// Forward declaration of the interrupt service routine
void addToMessageQueue(uint8_t elementId, int message);
void readRotaryEncodersTask(void *param);
void readVariableResistorAndJoystickTask(void *param);
void setupEncoders();
void selectChannel(uint8_t channel);
void setupVariableResistors();
void setupJoysticks();
void setupControleElementIds();
void readVariableResistorsAndJoysticks();
void listAvailableI2CAddresses();

void setup(){
  Serial.begin (115200);
  // ############### Setup outputted message queue ################
  messageOutQueue = xQueueCreate(100, sizeof(ElementMsg));
  // ############# ADC setup #############
  Wire.begin(); // I2C library for ADC
  if (!ads.begin()) {
    Serial.println("ADS1115 not detected! Check wiring.");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range (default)
  // ############ Setup controle element values ###################
  setupVariableResistors();
  setupJoysticks();
  setupControleElementIds();
  setupEncoders();
  //listAvailableI2CAddresses();
  //pcfEncoder8.encoder(encoder0PinA, encoder0PinB);
  //pcfEncoder9.encoder(encoder0PinA, encoder0PinB);
  // Set the encoder button pin as an input
  // pcfEncoder0.pinMode(P2, INPUT);

  
  // Create the high priority read rotary encoder task
  xTaskCreatePinnedToCore(
      readRotaryEncodersTask,
      "ReadRotaryEncodersTask",
      4096,
      NULL,
      1,
      //configMAX_PRIORITIES - 2,  // Very high priority
      &readEncodersTaskHandle,
      0
  );
  // Create the reading variable resistor and joystick task
  /*
  xTaskCreatePinnedToCore(
      readVariableResistorAndJoystickTask,
      "ReadVariableResistorsAndJoysticksTask",
      4096,
      NULL,
      1,  // Normal priority
      &readVariableResistorsAndJoysticksTaskHandle,
      0
  );
  */
  // Create the reading buttons task
  //xTaskCreatePinnedToCore(
  //    readAllButtonsTask,
  //    "ReadButtonsTask",
  //    4096,
  //    NULL,
  //    1,  // Normal priority
  //    &readButtonsTaskHandle,
  //    0
  //);
}

int debugPrintFlag = -1;

void loop(){
  // If there is a message in the queue print until empty
  if(uxQueueMessagesWaiting(messageOutQueue) > 0){
    while (uxQueueMessagesWaiting(messageOutQueue) > 0){
      if (xQueueReceive(messageOutQueue, &messageReadBuffer, 0) == pdTRUE) {
            Serial.printf("Element changed: ID: %d message: %d\n",
                          messageReadBuffer.elementId,
                          messageReadBuffer.message);
      }
      else{
        Serial.println("Failed to read message from message out queue!");
      }
    }
  }
}

// ---------------------------------------------------------------------
// ########################## SETUP FUNCTIONS ##########################
// ---------------------------------------------------------------------

/**
 * Instantiates values for the variable resistor variables.
 */
void setupVariableResistors(){
  for(uint8_t i=0;i<NB_VARIABLE_RESISTORS;i++){
    selectChannel(muxAdcVariableResistorMap[i][1]);
    variableResistorValues[i] = ads.readADC_SingleEnded(muxAdcVariableResistorMap[i][0]);
  }
}

/**
 * Instantiates values for the joystick variables.
 */
void setupJoysticks(){
  for(uint8_t i=0;i<NB_JOYSTICKS;i++){
    selectChannel(muxAdcJoystickMap[i][0][1]);
    joystickValues[i][0] = ads.readADC_SingleEnded(muxAdcJoystickMap[i][0][0]);
    selectChannel(muxAdcJoystickMap[i][1][1]);
    joystickValues[i][1] = ads.readADC_SingleEnded(muxAdcJoystickMap[i][1][0]);
  }
}

/**
 * Instantiates controle element IDs.
 */
void setupControleElementIds(){
  //uint8_t out[nbControleElementTypes][maxNbControleElementTypes];
  uint8_t idCounter = 0;
  // Set variable resistor ids
  for(uint8_t i=0;i<NB_VARIABLE_RESISTORS;i++){
    controleElementIds[0][i] = idCounter;
    idCounter++;
  }
  // Set joysticks ids
  for(uint8_t i=0;i<(NB_JOYSTICKS * 2);i++){
    controleElementIds[1][i] = idCounter;
    idCounter++;
  }
  // Set rotary encoders ids
  for(uint8_t i=0;i<NB_ROTARY_ENCODERS;i++){
    controleElementIds[2][i] = idCounter;
    idCounter++;
  }
  // Set buttons ids
  for(uint8_t i=0;i<NB_TOTAL_BUTTONS;i++){
    controleElementIds[3][i] = idCounter;
    idCounter++;
  }
  //controleElementIds = temp;
  //return out;
}

/**
 * Function called to setup all rotary encoders.
 */
void setupEncoders(){
  Serial.println("Initializing rotary encoders...");
  for(int i=0;i<NB_ROTARY_ENCODERS;i++){
    if(!pcfEncoders[i].begin()){
      Serial.print("Error when connecting to rotary encoder: ");
      Serial.println(i);
    }
  }
  delay(500);
  for(int i=0;i<NB_ROTARY_ENCODERS;i++){
    pcfEncoders[i].encoder(encoderAPins[i], encoderBPins[i]);
  }
}


// Interrupt Service Routine for rotary encoders 0 to 3 (ISR)
void updateEncoder0To3(){
   //ets_printf("Triggered interrupt!");
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      readEncodersTaskHandle,
      PCF_ID_ENC_0_TO_3,
      eSetBits, // Indicates what to do with the notification value.
      &xHigherPriorityTaskWoken
    );
    if (xHigherPriorityTaskWoken) {
      //ets_printf("Switching context!\n");
      portYIELD_FROM_ISR();
    }
}

// Interrupt Service Routine for rotary encoders 4 to 7 (ISR)
void updateEncoder4To7(){
   //ets_printf("Triggered interrupt!");
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      readEncodersTaskHandle,
      PCF_ID_ENC_4_TO_7,
      eSetBits,
      &xHigherPriorityTaskWoken
    );
    if (xHigherPriorityTaskWoken) {
      //ets_printf("Switching context!\n");
      portYIELD_FROM_ISR();
    }
}

// Interrupt Service Routine for rotary encoders 8 to 9 (ISR)
void updateEncoder8To9(){
   //ets_printf("Triggered interrupt 8 to 9!\n");
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      readEncodersTaskHandle,
      PCF_ID_ENC_8_TO_9,
      eSetBits,
      &xHigherPriorityTaskWoken
    );
    if (xHigherPriorityTaskWoken) {
      //ets_printf("Switching context!\n");
      portYIELD_FROM_ISR();
    }
}


// ---------------------------------------------------------------------
// ######################### GENERAL FUNCTIONS #########################
// ---------------------------------------------------------------------
/**
 * Function in charge of adding messages to the message out queue.
 * @param elementId The controle element id of the controle element sending a message.
 * @param message The message to be added to the queue.
 */
void addToMessageQueue(uint8_t elementId, long message){
    ElementMsg msgOut;
    msgOut.elementId = elementId;
    msgOut.message = message;
    if(xQueueGenericSend(messageOutQueue, &msgOut, (TickType_t) 10, queueSEND_TO_BACK) != pdPASS){
      ets_printf("Failed to add message to message out queue! Element id: %d, message: %d\n",
                 elementId,
                 message);
    }
}

/**
 * Scans all possible I2C addresses and prints all addresses with a device.
 */
void listAvailableI2CAddresses(){
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
}

/**
 * The function responsable for dealing with reading the rotary encoders.
 * @param param ?????
 */
void readRotaryEncodersTask(void *param){
    while(true){
      uint32_t pcfId;
      // Wait for signal from ISR to unblock task
      //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      xTaskNotifyWait(0, 0, &pcfId, portMAX_DELAY);
      // TODO: Add specific flag for each task or fix notification value signaling
      //Serial.print("Task notified: ");Serial.println(pcfId);
      switch (pcfId){
        case PCF_ID_ENC_0_TO_3:
          for(int i=0;i<4;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue(controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        case PCF_ID_ENC_4_TO_7:
          for(int i=4;i<8;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue(controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        case PCF_ID_ENC_8_TO_9:
          for(int i=8;i<10;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue(controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        default:
          break;
      }
    }
}

// ---------------------------------------------------------------------
// ############# VARIABLE RESISTOR AND JOYSTICK FUNCTIONS ##############
// ---------------------------------------------------------------------

/**
 * Selects the multiplexer channel to read from
 * @param channel The channel to set both multiplexers to.
 */
void selectChannel(uint8_t channel){
  digitalWrite(MUX_S0_PIN, channel & 0x1);
  digitalWrite(MUX_S1_PIN, (channel >> 1) & 0x1);
  digitalWrite(MUX_S2_PIN, (channel >> 2) & 0x1);
  digitalWrite(MUX_S3_PIN, (channel >> 3) & 0x1);
  //GPIO.out_w1ts = ((uint32_t)1 << clockInPin); // Set pin high fast
  //GPIO.out_w1tc = ((uint32_t)1 << clockInPin); // Set pin low fast
}

/**
 * The function in charge of switching between the multiplexer and ADC addresses
 * as well as checking if significant change has happen between curent and past readings.
 * If readings are significantly different it sets the new value for the controle
 * element and places the appropriate message to the message output queue.
 *//*
void readVariableResistorsAndJoysticks(){
  int16_t readBuffer = 0;
  for(uint8_t i=0;i<NB_VARIABLE_RESISTORS;i++){
    selectChannel(muxAdcVariableResistorMap[i][1]);
    //ets_printf("Chanel select: %d, ADC chanel: %d\n", muxAdcVariableResistorMap[i][1], muxAdcVariableResistorMap[i][0]);
    readBuffer = ads.readADC_SingleEnded(muxAdcVariableResistorMap[i][0]);
    if(abs(readBuffer - variableResistorValues[i]) > VARIABLE_RESISTOR_ALLOWED_VARIANCE){
      variableResistorValues[i] = readBuffer;
      addToMessageQueue((uint8_t)controleElementIds[0][i], variableResistorValues[i]);
    }
  }
  for(uint8_t i=0;i<NB_JOYSTICKS;i+=2){
    selectChannel(muxAdcJoystickMap[i][0][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][0][0]);
    //Serial.print("Buffer: ");Serial.print(readBuffer);Serial.print(", val: ");Serial.println(joystickValues[i][0]);
    if(abs(readBuffer - joystickValues[i][0]) > 2){
      //Serial.println(readBuffer);
      joystickValues[i][0] = readBuffer;
      addToMessageQueue((uint8_t)controleElementIds[1][i], joystickValues[i][0]);
    }
    selectChannel(muxAdcJoystickMap[i][1][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][1][0]);
    if(abs(readBuffer - joystickValues[i][1]) > 2){
      joystickValues[i][1] = readBuffer;
      addToMessageQueue((uint8_t)controleElementIds[1][i + 1], joystickValues[i][1]);
    }
  }
}
*/

/**
 * The function responsable for dealing with reading the variable resistors
 * and joysticks.
 * @param param ?????
 */
void readVariableResistorAndJoystickTask(void *param){
    while (true) {
        readVariableResistorsAndJoysticks();
        vTaskDelay(pdMS_TO_TICKS(10));  // Optional delay to yield CPU
    }
}

/**
 * The function responsable for dealing with reading the all buttons.
 * @param param ?????
 */
void readButtonsTask(void *param){
    while(true){
      static uint32_t pcfId = 10;
      // Wait for signal from ISR to unblock task
      //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      xTaskNotifyWait(0, 0xFFFFFFFF, &pcfId, portMAX_DELAY);
      switch (pcfId){
        case PCF_ID_ENC_0_TO_3:
          for(int i=0;i<4;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue((uint8_t)controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        case PCF_ID_ENC_4_TO_7:
          for(int i=4;i<8;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue((uint8_t)controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        case PCF_ID_ENC_8_TO_9:
          for(int i=8;i<10;i++){
            if(pcfEncoders[i].readEncoderValue(encoderAPins[i], encoderBPins[i], &encoderCounters[i])){
              addToMessageQueue((uint8_t)controleElementIds[2][i], encoderCounters[i]);
            }
          }
          break;
        default:
          break;
      }
    }
}

