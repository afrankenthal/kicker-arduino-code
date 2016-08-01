/*
  kickerControlGalileoV2.ino - Andre Frankenthal (last updated: 7/26/16)
  
  This program reads the file containing control data produced from the webserver
  (controlData.json) and transmits it to the Arduino Due, using the SPI protocol.
  It also receives the monitoring data from the Due, which it saves in the SD card
  for readout by the webserver (monitorData.json). Since SPI is full-duplex, both
  control and monitoring data are exchanged at the same time. Note that the Galileo
  acts as SPI master, since that's the only possible mode. Therefore the clock is set
  by the Galileo, and currently the limit is apparently 4 MHz (I have been trying to
  increase it without success). Right now the SPI transfer happens every 1 second, but
  this is only for testing purposes. It should be increased to the maximum rate that
  is still compatible with reliant and stable operation.
  
*/

// Includes

#include <ArduinoJson.h>
#include <stdio.h>
#include <SPI.h>


// Defines

#define VCC 3.253

// set pin 10 as the slave select for the Due:
#define slaveSelectPin 10

// Analog input pins -- these must correspond to the definitions in the Due code!!
#define HV1_MONITOR_PIN        0 // AD7 corresponds to Analog 0
#define HV2_MONITOR_PIN        2 // AD5 corresponds to Analog 2
#define HV3_MONITOR_PIN        4 // AD3 corresponds to Analog 4
#define TRANSFORMER1_VOLT_PIN  1 // AD6 corresponds to Analog 1
#define TRANSFORMER2_VOLT_PIN  3 // AD4 corresponds to Analog 3
#define TRANSFORMER3_VOLT_PIN  5 // AD2 corresponds to Analog 5

// ADC- and SPI-related defines
#define ADC_LEN 1024          // The size of one full sampling of the transformer voltages
#define SPI_BUFF_SIZE 4092    // The size of the SPI buffer used to communicate with the Due
                              // (4092 bits per single transfer is the maximum I was able to achieve,
                              // empirically)

// Inline helper functions

inline static uint8_t convertAnalogChannelToADC(uint8_t analog) {
  return (analog <= 7) ? 7 - analog : analog + 2;
}

inline static uint8_t convertADCChannelToAnalog(uint8_t adc) {
  return (adc <= 7) ? 7 - adc : adc - 2;
}

// This is the bit mask to select only the 4 bits out of 16 that
// are used to identify which channel an ADC reading came from
uint16_t channelMask = 0xf000;

// The in and out buffers, for SPI transfer (note the in is much larger,
// as it contains the monitoring data sent from the Due; the out buffer
// only contains control information and thus is much shorter)
uint8_t recData[SPI_BUFF_SIZE + 8];
uint8_t outBuffer[30];

uint16_t channelDataCounter[12];


// Define the structs that contain monitoring and control data

struct sensorData {
  uint16_t highVoltage1;
  uint16_t highVoltage2;
  uint16_t highVoltage3;
  uint16_t transformerVoltage1[ADC_LEN];
  uint16_t transformerVoltage2[ADC_LEN];
  uint16_t transformerVoltage3[ADC_LEN];
};

struct sensorData currentData;

struct controlParams {
  uint8_t controlFlags;
  uint8_t VoltageOut1;
  uint8_t VoltageOut2;
  uint8_t VoltageOut3;
  
  // These params require more than 8 bits
  uint16_t CAP1_Time;
  uint16_t CAP2_Time;
  uint16_t CAP3_Time;
  uint16_t CAP1_SCR1_Delay;
  uint16_t CAP2_SCR2_Delay;
  uint16_t CAP3_SCR3_Delay;
  uint16_t SCR1_THYR1_Delay;
  uint16_t SCR2_THYR2_Delay;
  uint16_t SCR3_THYR3_Delay;
  uint16_t Fill_Spacing_Time;
  uint16_t Bunch_Spacing_Time;
  uint16_t Cycle_Spacing_Time;
  
  // This vector is used to be able to iterate over the 16-bit params
  uint16_t all16bitParams[12];
};

struct controlParams kickerParams;


/**
  This function fills the monitoring data struct according to
  the channel number encoded by the ADC. The 16-bit piece of data
  has a 4-bit channel identifier plus 12-bit actual data. The ADC
  channel id is converted into the more conventional analog channel id 
  and then assigned to the appropriate variable in the struct. For
  transformer voltages, which are sequences of 1024 data points, a counter
  is kept to make sure we don't go over the assigned ADC_LEN limit.
*/
void inline channelToStruct(uint8_t channelNumber, uint16_t datum) {
  
  uint8_t analogChannelNumber = convertADCChannelToAnalog(channelNumber);
  
  switch (analogChannelNumber) {
    case HV1_MONITOR_PIN:
      currentData.highVoltage1 = datum;
      break;
    case HV2_MONITOR_PIN:
      currentData.highVoltage2 = datum;
      break;
    case HV3_MONITOR_PIN:
      currentData.highVoltage3 = datum;
      break;
    case TRANSFORMER1_VOLT_PIN:
      if (channelDataCounter[channelNumber] < ADC_LEN - 1)
        currentData.transformerVoltage1[channelDataCounter[channelNumber]++] = datum;
      break;
    case TRANSFORMER2_VOLT_PIN:
      if (channelDataCounter[channelNumber] < ADC_LEN - 1)
        currentData.transformerVoltage2[channelDataCounter[channelNumber]++] = datum;
      break;
    case TRANSFORMER3_VOLT_PIN:
      if (channelDataCounter[channelNumber] < ADC_LEN - 1)
        currentData.transformerVoltage3[channelDataCounter[channelNumber]++] = datum;
      break;
  }
}


/**
  This function writes the monitoring data the Galileo received
  from the Due into a file for readout by the webserver. The monitoring
  data is encoded in the currentData struct, and we use an json parser
  library (ArduinoJson) to serialize this data into JSON format and write
  it to file. 
*/
void sendSerializedMonitorData(const struct sensorData & data) {
  
  // Creates the buffer for holding all the variables
  const int JSON_BUFFER_SIZE = JSON_OBJECT_SIZE(6) + 3 * JSON_ARRAY_SIZE(ADC_LEN);
  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  
  // Creates the actual JSON object
  JsonObject& jsonOut = jsonBuffer.createObject();
  
  // Adds single-data elements to the JSON object
  jsonOut["highVoltage1"] = currentData.highVoltage1;
  jsonOut["highVoltage2"] = currentData.highVoltage2;
  jsonOut["highVoltage3"] = currentData.highVoltage3;
  
  // Adds the transformer voltages and other 1024-long data
  JsonArray& transformerVoltage1 = jsonOut.createNestedArray("transformerVoltage1");
  JsonArray& transformerVoltage2 = jsonOut.createNestedArray("transformerVoltage2");
  JsonArray& transformerVoltage3 = jsonOut.createNestedArray("transformerVoltage3");
  
  for (int i = 0; i < ADC_LEN; i++) {
    transformerVoltage1.add(currentData.transformerVoltage1[i]);
    transformerVoltage2.add(currentData.transformerVoltage2[i]);
    transformerVoltage3.add(currentData.transformerVoltage3[i]);
  }
  
  // Prints the JSON object to a char array
  char jsonStrOut[20000];
  jsonOut.printTo(jsonStrOut, 20000);
  
  // Writes the char array to file
  __STDIO_FILE_STRUCT * file;
  file = fopen("/media/card/json/test.json","w");
  if (file) {
    fputs(jsonStrOut, file);
    fclose(file);
  }
  else {
    Serial.println("[sendSerializedMonitorData] Failed to open monitorData/test file!");
  }
  
}


/**
  This function receives control data from the webserver
  via a JSON file (controlData.json) and assigns it to 
  the control struct kickerParams, making it ready to be
  sent via SPI to the Due.
*/
bool receiveSerializedControlData() {
  
  char jsonInStr[1000];
  
  // Reads control data file
  __STDIO_FILE_STRUCT * file;
  file = fopen("/media/card/json/controlData.json", "r");
  if (file) {
    if (fgets(jsonInStr, 1000, file) == NULL) {
      Serial.println("[receiveSerializedControlData] Failed to read controlData file!");
      fclose(file);
      return false;
    }
    else
      fclose(file);
  }
  else {
    Serial.println("[receiveSerializedControlData] Failed to open controlData file!");
    return false;
  }
  
  // Creates the json buffer and the parsed object
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& jsonIn = jsonBuffer.parseObject(jsonInStr);
  
  if (!jsonIn.success()) {
    Serial.println("[receiveSerializedControlData] JSON: parseObject() failed!");
    return false;
  }
  
  // Assigns all on/off (or internal/external) switches to a bit-mask variable,
  // called controlFlags. Each bit represents a different boolean parameter
  bool tempControl;
  tempControl = (!strcmp(jsonIn["TriggerMode"], "Internal")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 0);
  tempControl = (!strcmp(jsonIn["RackPowerSupplyStatus"], "ON")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 1);
  tempControl = (!strcmp(jsonIn["HVPowerSupplyStatus"], "ON")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 2);
//  tempControl = (!strcmp(jsonIn["OilPumpStatus"], "ON")) ? 1 : 0;
//  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 3);
//  tempControl = (!strcmp(jsonIn["FlourinetPumpStatus"], "ON")) ? 1 : 0;
//  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 4);
  tempControl = (!strcmp(jsonIn["GeneralTriggerStatus"], "ON")) ? 1 : 0; 
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 3);
  tempControl = (!strcmp(jsonIn["Kicker_Status_1"], "ACTIVE")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 4);
  tempControl = (!strcmp(jsonIn["Kicker_Status_2"], "ACTIVE")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 5);
  tempControl = (!strcmp(jsonIn["Kicker_Status_3"], "ACTIVE")) ? 1 : 0;
  kickerParams.controlFlags ^= (-tempControl ^ kickerParams.controlFlags) & (1 << 6);
  
  // Adds to struct the time/delay params
  kickerParams.CAP1_Time = jsonIn["CAP1_Time"];
  kickerParams.CAP2_Time = jsonIn["CAP2_Time"];
  kickerParams.CAP3_Time = jsonIn["CAP3_Time"];
  kickerParams.CAP1_SCR1_Delay = jsonIn["CAP1-SCR1_Delay"];
  kickerParams.CAP2_SCR2_Delay = jsonIn["CAP2-SCR2_Delay"];
  kickerParams.CAP3_SCR3_Delay = jsonIn["CAP3-SCR3_Delay"];
  kickerParams.SCR1_THYR1_Delay = jsonIn["SCR1-THYR1_Delay"];
  kickerParams.SCR2_THYR2_Delay = jsonIn["SCR2-THYR2_Delay"];
  kickerParams.SCR3_THYR3_Delay = jsonIn["SCR3-THYR3_Delay"];
  kickerParams.Fill_Spacing_Time = jsonIn["Fill_Spacing_Time"];
  kickerParams.Bunch_Spacing_Time = jsonIn["Bunch_Spacing_Time"];
  kickerParams.Cycle_Spacing_Time = jsonIn["Cycle_Spacing_Time"];
  
  // Copies the params that were just added to an indexed vector,
  // to make it easier to split the 16-bit params into two 8-bit vectors
  kickerParams.all16bitParams[0] = kickerParams.CAP1_Time;
  kickerParams.all16bitParams[1] = kickerParams.CAP2_Time;
  kickerParams.all16bitParams[2] = kickerParams.CAP3_Time;
  kickerParams.all16bitParams[3] = kickerParams.CAP1_SCR1_Delay;
  kickerParams.all16bitParams[4] = kickerParams.CAP2_SCR2_Delay;
  kickerParams.all16bitParams[5] = kickerParams.CAP3_SCR3_Delay;
  kickerParams.all16bitParams[6] = kickerParams.SCR1_THYR1_Delay;
  kickerParams.all16bitParams[7] = kickerParams.SCR2_THYR2_Delay;
  kickerParams.all16bitParams[8] = kickerParams.SCR3_THYR3_Delay;
  kickerParams.all16bitParams[9] = kickerParams.Fill_Spacing_Time;
  kickerParams.all16bitParams[10] = kickerParams.Bunch_Spacing_Time;
  kickerParams.all16bitParams[11] = kickerParams.Cycle_Spacing_Time;
  
  kickerParams.VoltageOut1 = (uint8_t)(255 * atof(jsonIn["VoltageOut1"])/VCC);
  kickerParams.VoltageOut2 = (uint8_t)(255 * atof(jsonIn["VoltageOut2"])/VCC);
  kickerParams.VoltageOut3 = (uint8_t)(255 * atof(jsonIn["VoltageOut3"])/VCC);
  
  return true;
}


/**
  This function prepares the out buffer that will be sent to the 
  Due over SPI. The goal here is to minimize the amount of overhea that
  needs to be sent over, because we need to maximize the amount of actual
  data and the transmission (even though SPI is the fastest) takes time.
  So there is a rigid structure about the order in which the information is sent
  (in a sense a protocol) which needs to be reversed on the other end exactly as
  built here. The order is as follows: first the controlFlag bits are sent, then
  the HV voltages, and then the 16-bit param vectors that were constructed in the
  receiveSerializedControlData function (note that these are split in two to send
  over 8-bit SPI). The two hex values are a primitive form of error-identification:
  if the values are different on the receiving end, the data was corrupted somehow
  and is thrown away.
*/
void prepareArduinoBuffer() {
  
  outBuffer[0] = 0x46;
  outBuffer[1] = kickerParams.controlFlags;
  outBuffer[2] = kickerParams.VoltageOut1;
  outBuffer[3] = kickerParams.VoltageOut2;
  outBuffer[4] = kickerParams.VoltageOut3;
  
  // Does a bit of bitmask operations to split the data into two 8-bit parts
  for (int i = 0; i < 2 * 12; i++) {
      outBuffer[i+4] = (uint8_t)((kickerParams.all16bitParams[i/2] & 0xff00) >> 8);
      i++;
      outBuffer[i+4] = (uint8_t)(kickerParams.all16bitParams[i/2] & 0xff);
  }
  
  outBuffer[29] = 0x55;
  
//  String test;
//  for (int i = 0; i < 28; i++) {
//    test += String(outBuffer[i], HEX) + ",";
//  }
//  Serial.println(test);
}

void setup() {
  
  // Initializes SPI with fastest clock possible
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE1);
      
  // Initializes serial port for debug
  Serial.begin(115200);
  
  // Initializes SPI slave select and sets it to low.
  // This basically means we are always choosing the Due
  // to be the slave for the SPI, which is okay since we
  // only have one slave anyway
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin,LOW);
  
}

void loop() {
  
  // Reads control data from webserver and stores into
  // struct to send over SPI
  bool serializedSuccess = receiveSerializedControlData();
  
  if (serializedSuccess) {
    
    // Prepares out buffer to send to Due via SPI
    prepareArduinoBuffer();  
    
    // Times how long SPI transfer takes
    long int startt = millis();
    
    // 4092 bits per single transfer is the maximum
    // I was able to achieve, empirically
    SPI.transferBuffer(outBuffer, recData, SPI_BUFF_SIZE);
    
    long int stopt = millis();
   
    Serial.println("SPI transfer duration:");
    Serial.println(stopt - startt);
    
    // Check if received data from Due has errors
    bool errorFlag = false;

    for (int i = 0; i < SPI_BUFF_SIZE; i++) {
      if (i == 0)
        if (recData[i] != 0x4C) // First byte is supposed to be this
          errorFlag = true;
      else if (i == 1)
        if (recData[i] != 0x47) // Second byte is supposed to be this
          errorFlag = true;
      else {
        uint16_t datum = ((uint16_t)recData[i] << 8) | recData[++i];
        uint8_t channelNumber = (datum & channelMask) >> 12;
        channelToStruct(channelNumber, (datum & ~(channelMask)));
      }
    }
    
    if (errorFlag) {
      Serial.println("ERROR in SPI transmission! Trying to reset SPI...");
      SPI.begin(); 
      SPI.setClockDivider(SPI_CLOCK_DIV2);
      SPI.setDataMode(SPI_MODE1);
    } 
    else // If no error, write monitor data to file to be displayed
      sendSerializedMonitorData(currentData);
  
    // Reset counters of data arrays
    channelDataCounter[convertAnalogChannelToADC(TRANSFORMER1_VOLT_PIN)] = 0;
    channelDataCounter[convertAnalogChannelToADC(TRANSFORMER2_VOLT_PIN)] = 0;
    channelDataCounter[convertAnalogChannelToADC(TRANSFORMER3_VOLT_PIN)] = 0;
    
    
    //    Test: print out data received from Due
    //
    //    uint16_t realData[2046];
    //    realData[0] = 0;
    //    
    //    for (int i = 0; i < SPI_BUFF_SIZE; i++) {
    //      if (i != 0 && i != 1) {
    //        uint16_t datum = ((uint16_t)recData[i] << 8) | recData[++i];
    //        realData[i/2] = datum & ~(channelMask);
    //      }
    //    }
    //    
    //    String strin;
    //    for (int i = 0; i < 2046; i++) {
    //      strin += String(realData[i], DEC) + ",";
    //    }
    //    Serial.println(strin);
 
 
  } // end if serializedSuccess
 
  delay(1000);
    
} // end loop
