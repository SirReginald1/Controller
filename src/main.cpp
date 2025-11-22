#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <Wire.h>
#include <Arduino.h>
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
// ############## Rotary encoder constants ################
/* The shift register latch pin for rotary encoders */
#define ROTARY_ENCODER_LOAD_PIN 27
/* The shift register clock in pin for rotary encoders */
#define ROTARY_ENCODER_SHIFT_PIN 26
/* The pin that reads DT pin from the rotary encoders */
#define ROTARY_ENCODER_CLK_READ_PIN 4
/* The pin that reads CLK pin from the rotary encoders */
#define ROTARY_ENCODER_DT_READ_PIN 16
/* The interrupt pin used to trigger rotary encoder reading */
#define ENC_DT_INT_PIN 33
// ################# Buttons constants ###################
/* The shift register latch pin for buttons */
#define BUTTON_LOAD_PIN 23
/* The clock signal used to shift data out of the register. 
The clockEnablePin must be set to low to enable the shifting of data. */
#define BUTTON_SHIFT_PIN 13
/* The pin used to read base buttons data */
#define BUTTON_BASE_READ_PIN 25
/* The pin used to read base buttons data */
#define BUTTON_OTHER_READ_PIN 34
/* The interrupt pin used to trigger button value reading */
#define BUTTON_INT_PIN 32
// ------------------------------------------------------------------
// ######################## NB CONTROLE ELEMENTS ####################
// ------------------------------------------------------------------
// ######## Number of input elements constants ##########
/* The number of rotary encoders to be read */
#define NB_ROTARY_ENCODERS 2
/* Number of variable resistors */
#define NB_VARIABLE_RESISTORS 2
/* Number of joysticks */
#define NB_JOYSTICKS 1
/* Number of buttons */
#define NB_BASE_BUTTONS 3
/* The total number of shifts to be executed when reading buttons.
   Depends on the max number of shift registers in series for all clickables.
*/
#define NB_BUTTON_SHIFTS 3 // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/* The total number of pressable elements */
const int NB_TOTAL_BUTTONS = NB_BASE_BUTTONS + NB_ROTARY_ENCODERS + NB_JOYSTICKS;
// ################ General constants ###################
#define NB_CONTROLE_ELEMENT_TYPES 4
// ------------------------------------------------------------------
// ######################## MODEL VARIABLES #########################
// ------------------------------------------------------------------
// ################# Buttons vars #######################
/* Array containing the current value for each button */
bool buttonValues[NB_BASE_BUTTONS] = {0};
/* The handle for the task responsible for reading the buttons */
TaskHandle_t readButtonsTaskHandle = NULL;
// ################## variable resistor vars ########################
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
// ################# rotary encoder vars #################
/* The value of the currently polled rotary encoder DT channel  */
int rotaryEncoderDtCurrent = 0;
/* The counters for each rotary encoder */
int encoderCounters[NB_ROTARY_ENCODERS] = {0};
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
    int message;
} EncoderMsg;
/* The queue containing the message to be sent */
QueueHandle_t messageOutQueue;
/* Flag indicating if a task that needs to be run */
BaseType_t xHigherPriorityTaskWoken;
/* The message buffer used to read values from the message queue.
   Only one buffer should be needed as there is only one piece of code
   that read the mesage queue.
*/
EncoderMsg messageReadBuffer;

// Right turn: 00 -> 10 -> | 11 --> 01 -> 00
// Left turn:  00 -> | 01 -> 11 --> 10 -> 00
// Quadrature lookup table for state transitions
/*
static const int8_t rotaryEncoderTransitionTable[16] = {
    0, -1, 1,  0,
   1,  0,  0, 1,
   -1,  0,  0, 1,
    0, 1, -1,  0
};
*/


// ---------------------------------------------------------------------
// ######################## DECLARING FUNCTIONS ########################
// ---------------------------------------------------------------------
void selectChannel(uint8_t channel);
void setupVariableResistors();
void setupJoysticks();
void setupControleElementIds();
void addToMessageQueue(uint8_t elementId, int message);
void readVariableResistorsAndJoysticks();
void readVariableResistorAndJoystickTask(void *param);
void readRotaryEncoderShiftRegisters();
void readRotaryEncodersTask(void *param);
void IRAM_ATTR readRotaryEncoderISR();
void readAllButtonsTask(void *param);
void readButtonShiftRegisters();
void IRAM_ATTR readAllButtonsISR();


// ---------------------------------------------------------------------
// ############################## SETUP ###############################
// ---------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  // ############### Setup outputted message queue ################
  messageOutQueue = xQueueCreate(100, sizeof(EncoderMsg));
  // ####################### Setup Buttons ########################
  pinMode(BUTTON_LOAD_PIN, OUTPUT);
  pinMode(BUTTON_SHIFT_PIN, OUTPUT);
  pinMode(BUTTON_BASE_READ_PIN, INPUT);
  pinMode(BUTTON_OTHER_READ_PIN, INPUT);
  pinMode(BUTTON_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), readAllButtonsISR, CHANGE);
  // ######################### Setup MUX ##########################
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S3_PIN, OUTPUT);
  // ############ Setup rotary encoder shift registers #############
  pinMode(ROTARY_ENCODER_LOAD_PIN, OUTPUT);
  pinMode(ROTARY_ENCODER_SHIFT_PIN, OUTPUT);
  pinMode(ROTARY_ENCODER_CLK_READ_PIN, INPUT);
  pinMode(ROTARY_ENCODER_DT_READ_PIN, INPUT);
  // ############ Make sure that pins start correct ################ 
  digitalWrite(ROTARY_ENCODER_LOAD_PIN, HIGH);
  digitalWrite(ROTARY_ENCODER_SHIFT_PIN, LOW);
  // ############ Setup rotary encoder interrupt ###################
  pinMode(ENC_DT_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_INT_PIN), readRotaryEncoderISR, CHANGE);
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
  // ######################## Start tasks #########################
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
  xTaskCreatePinnedToCore(
      readVariableResistorAndJoystickTask,
      "ReadVariableResistorsAndJoysticksTask",
      4096,
      NULL,
      1,  // Normal priority
      &readVariableResistorsAndJoysticksTaskHandle,
      0
  );
  // Create the reading buttons task
  xTaskCreatePinnedToCore(
      readAllButtonsTask,
      "ReadButtonsTask",
      4096,
      NULL,
      1,  // Normal priority
      &readButtonsTaskHandle,
      0
  );
}

// ---------------------------------------------------------------------
// ############################### LOOP ################################
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// ######################### GENERAL FUNCTIONS #########################
// ---------------------------------------------------------------------
/**
 * Function in charge of adding messages to the message out queue.
 * @param elementId The controle element id of the controle element sending a message.
 * @param message The message to be added to the queue.
 */
void addToMessageQueue(uint8_t elementId, int message){
    EncoderMsg msgOut;
    msgOut.elementId = elementId;
    msgOut.message = message;
    if(xQueueGenericSend(messageOutQueue, &msgOut, (TickType_t) 10, queueSEND_TO_BACK) != pdPASS){
      ets_printf("Failed to add message to message out queue! Element id: %d, message: %d\n",
                 elementId,
                 message);
    }
}

// ---------------------------------------------------------------------
// ########################## BUTTON FUNCTIONS #########################
// ---------------------------------------------------------------------

/**
 * The function in charge of shifting in the button values from the
 * shift registers, recording values and adding to the message queue
 * when appropriate.
 */
void readButtonShiftRegisters(){
  static bool buttonBaseReadBuffer = 0;
  static bool buttonOtherReadBuffer = 0;
  //ets_printf("Reading shift registers.\n");
  GPIO.out_w1tc = ((uint32_t)1 << BUTTON_LOAD_PIN); // Set pin low fast
  delayMicroseconds(1); // This delay seems to improve reading.
  GPIO.out_w1ts = ((uint32_t)1 << BUTTON_LOAD_PIN); // Set pin high fast
  for (int i = 0; i < NB_BUTTON_SHIFTS; i++) {
    buttonBaseReadBuffer = (GPIO.in >> BUTTON_BASE_READ_PIN) & 0x1;
    buttonOtherReadBuffer = (GPIO.in1.val >> (BUTTON_OTHER_READ_PIN - 32)) & 0x1;
    // Only if different than old value
    if(buttonValues[i] != buttonBaseReadBuffer){
      buttonValues[i] = buttonBaseReadBuffer;
      addToMessageQueue(controleElementIds[3][i], (int)buttonBaseReadBuffer);
    }
    // Only if within range and different than previous
    if((i < (NB_ROTARY_ENCODERS + NB_JOYSTICKS)) & (buttonValues[i + NB_BASE_BUTTONS] != buttonOtherReadBuffer)){
      buttonValues[i + NB_BASE_BUTTONS] = buttonOtherReadBuffer;
      addToMessageQueue(controleElementIds[3][i + NB_BASE_BUTTONS], (int)buttonOtherReadBuffer);
    }
    // Shift out the next bit to QH
    GPIO.out_w1ts = ((uint32_t)1 << BUTTON_SHIFT_PIN); // Set pin high fast
    delayMicroseconds(1);
    GPIO.out_w1tc = ((uint32_t)1 << BUTTON_SHIFT_PIN); // Set pin low fast
  }
}

/**
 * The function responsable for dealing with reading the rotary encoders.
 * @param param ?????
 */
void readAllButtonsTask(void *param){
    while (true) {
      // Wait for signal from ISR to unblock task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readButtonShiftRegisters();
    }
}

/**
 * The ISR triggered by the changing of a button value
 * of any clickable controle element. Note that if 2 buttons
 * are pressed and one of them is released it will not be
 * detected due to the the shared line still being heigh.
 */
void IRAM_ATTR readAllButtonsISR(){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      readButtonsTaskHandle,
      0,
      eNoAction,
      &xHigherPriorityTaskWoken
    );
    if (xHigherPriorityTaskWoken) {
      //ets_printf("Switching context!\n");
      portYIELD_FROM_ISR();
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
 */
void readVariableResistorsAndJoysticks(){
  int16_t readBuffer = 0;
  for(uint8_t i=0;i<NB_VARIABLE_RESISTORS;i++){
    selectChannel(muxAdcVariableResistorMap[i][1]);
    //ets_printf("Chanel select: %d, ADC chanel: %d\n", muxAdcVariableResistorMap[i][1], muxAdcVariableResistorMap[i][0]);
    readBuffer = ads.readADC_SingleEnded(muxAdcVariableResistorMap[i][0]);
    if(abs(readBuffer - variableResistorValues[i]) > VARIABLE_RESISTOR_ALLOWED_VARIANCE){
      variableResistorValues[i] = readBuffer;
      addToMessageQueue(controleElementIds[0][i], variableResistorValues[i]);
    }
  }
  for(uint8_t i=0;i<NB_JOYSTICKS;i+=2){
    selectChannel(muxAdcJoystickMap[i][0][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][0][0]);
    //Serial.print("Buffer: ");Serial.print(readBuffer);Serial.print(", val: ");Serial.println(joystickValues[i][0]);
    if(abs(readBuffer - joystickValues[i][0]) > 2){
      //Serial.println(readBuffer);
      joystickValues[i][0] = readBuffer;
      addToMessageQueue(controleElementIds[1][i], joystickValues[i][0]);
    }
    selectChannel(muxAdcJoystickMap[i][1][1]);
    readBuffer = ads.readADC_SingleEnded(muxAdcJoystickMap[i][1][0]);
    if(abs(readBuffer - joystickValues[i][1]) > 2){
      joystickValues[i][1] = readBuffer;
      addToMessageQueue(controleElementIds[1][i + 1], joystickValues[i][1]);
    }
  }
}


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

// ---------------------------------------------------------------------
// ###################### ROTARY ENCODER FUNCTIONS #####################
// ---------------------------------------------------------------------

/**
 * The function in charge of shifting in the rotary encoder values from the
 * shift registers, incrementing or decrementing the counter values as needed
 * and adding message to message queue.
 */
void readRotaryEncoderShiftRegisters() {
  //ets_printf("Reading shift registers.\n");
  GPIO.out_w1tc = ((uint32_t)1 << ROTARY_ENCODER_LOAD_PIN); // Set pin low fast
  delayMicroseconds(1); // This delay seems to improve reading.
  GPIO.out_w1ts = ((uint32_t)1 << ROTARY_ENCODER_LOAD_PIN); // Set pin high fast
  for (int i = 0; i < NB_ROTARY_ENCODERS; i++) {
    rotaryEncoderDtCurrent = (GPIO.in >> ROTARY_ENCODER_DT_READ_PIN) & 0x1;
    // If there is a minimal movement of 1 step
    if ((dtLastState[i] == LOW) && (rotaryEncoderDtCurrent == HIGH)) {
        if ((GPIO.in >> ROTARY_ENCODER_CLK_READ_PIN) & 0x1 == HIGH) {      // If Pin B is HIGH
          encoderCounters[i] ++;
          //ets_printf("i: %d, Counter: %d\n", i, encoderCounters[i]);        
        } else {
          encoderCounters[i] --;
          //ets_printf("i: %d, Counter: %d\n", i, encoderCounters[i]);
        }
        // TODO: Set element id to reference table value.
        addToMessageQueue(controleElementIds[2][i], encoderCounters[i]);
      }
      dtLastState[i] = rotaryEncoderDtCurrent;
      // Shift out the next bit to QH
      GPIO.out_w1ts = ((uint32_t)1 << ROTARY_ENCODER_SHIFT_PIN); // Set pin high fast
      delayMicroseconds(1);
      GPIO.out_w1tc = ((uint32_t)1 << ROTARY_ENCODER_SHIFT_PIN); // Set pin low fast
    }
}

/**
 * The function responsable for dealing with reading the rotary encoders.
 * @param param ?????
 */
void readRotaryEncodersTask(void *param){
    while (true) {
      // Wait for signal from ISR to unblock task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readRotaryEncoderShiftRegisters();
    }
}

/**
 * The ISR triggered by the changing in the value of the DT value
 * of any of the rotary encoders.
 */
void IRAM_ATTR readRotaryEncoderISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      readEncodersTaskHandle,
      0,
      eNoAction,
      &xHigherPriorityTaskWoken
    );
    if (xHigherPriorityTaskWoken) {
      //ets_printf("Switching context!\n");
      portYIELD_FROM_ISR();
    }
}