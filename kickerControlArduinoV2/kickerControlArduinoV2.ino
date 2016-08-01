/**
 * kickerControlArduinoV2.ino - Andre Frankenthal (last updated: 7/27/16)
 * 
 * This program controls the kicker equipment and monitor feedback information provided
 * by the apparatus. It sends triggers and on/off commands to the kicker and also voltage
 * settings, and reads information back with an ADC. The general operation of the code and
 * the system are described in the M&C User Guide document, included in the git package under
 * the folder docs. Basically we use a task scheduler library to make sure that critical time-
 * sensitive functions are performed as scheduled, and to logically divide up the tasks. There
 * are tasks to issue triggers, to read ADC data, and to update the control params from the Intel
 * Galileo using the SPI protocol. The Due acts as a slave, and this requires a bit of configuration
 * as the default behavior is to be master. The tasks can't take too long to execute or else they
 * risk overshadowing another timed task, so we have to be smart about splitting up the program
 * into small sized tasks.
 * 
 */

#undef HID_ENABLED

// Task scheduler defines
// (needs to go before include)
#define _TASK_MICRO_RES
#define _TASK_PRIORITY
#define _TASK_TIMECRITICAL
#define _TASK_STATUS_REQUEST


// Includes
#include <TaskScheduler.h>
#include <SPI.h>


// Basic defines
#define MS 1000
#define TIMEOUT 3000
#define VCC 3.253 // Value calibrated from measuring 3.3 V pin on Arduino


// -----------------------------------------------------------------------------------------------------
// Pin defines

// Communication pins
#define TX_PIN 14
#define RX_PIN 15

// Digital out pins
#define BOS_TRIGGER_PIN 40
#define HV1_INHIBIT_PIN 38
#define SCR1_CHARGE_PIN 44
#define THYR1_TRIGGER_PIN 48

// PWM out pins
#define HV1_CONTROL_PIN 5
#define HV2_CONTROL_PIN 7
#define HV3_CONTROL_PIN 9

// Relay pins
#define RACK1_PS_PIN 22
#define RACK2_PS_PIN 24
#define RACK3_PS_PIN 26
#define HV3_PS_PIN 32
#define HV2_PS_PIN 30
#define HV1_PS_PIN 28
//#define OIL_PUMP_PIN 28
//#define FLOURINET_PUMP_PIN 30
//#define THYR_DRIV_PIN 32

// Analog input pins
#define HV1_MONITOR_PIN        0 // AD7 corresponds to Analog 0
#define HV2_MONITOR_PIN        2 // AD5 corresponds to Analog 2
#define HV3_MONITOR_PIN        4 // AD3 corresponds to Analog 4
#define TRANSFORMER1_VOLT_PIN  1 // AD6 corresponds to Analog 1
#define TRANSFORMER2_VOLT_PIN  3 // AD4 corresponds to Analog 3
#define TRANSFORMER3_VOLT_PIN  5 // AD2 corresponds to Analog 5

/** Fast digital write (manipulates ports directly) */
inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

/** Fast digital read (manipulates ports directly) */
inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}


// -----------------------------------------------------------------------------------------------------
// DMA defines

// DMAC receive channel
#define SPI_DMAC_RX_CH  1
// DMAC transmit channel
#define SPI_DMAC_TX_CH  0
// DMAC Channel HW Interface Number for SPI TX
#define SPI_TX_IDX  1
// DMAC Channel HW Interface Number for SPI RX
#define SPI_RX_IDX  2

/** Disable DMA Controller. */
static void dmac_disable() {
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
/** Enable DMA Controller. */
static void dmac_enable() {
  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
/** Disable DMA Channel. */
static void dmac_channel_disable(uint32_t ul_num) {
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}
/** Enable DMA Channel. */
static void dmac_channel_enable(uint32_t ul_num) {
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}
/** Poll for transfer complete. */
static bool dmac_channel_transfer_done(uint32_t ul_num) {
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
}

/** start RX DMA */
void spiDmaRX(uint8_t* dst, uint16_t count) {
  dmac_channel_disable(SPI_DMAC_RX_CH);
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_SADDR = (uint32_t)&SPI0->SPI_RDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DADDR = (uint32_t)dst;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DSCR =  0;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC |
    DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CFG = DMAC_CFG_SRC_PER(SPI_RX_IDX) |
    DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;
  dmac_channel_enable(SPI_DMAC_RX_CH);
}

/** start TX DMA */
void spiDmaTX(const uint8_t* src, uint16_t count) {
  static uint8_t ff = 0Xff;
  uint32_t src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;
  if (!src) {
    src = &ff;
    src_incr = DMAC_CTRLB_SRC_INCR_FIXED;
  }
  dmac_channel_disable(SPI_DMAC_TX_CH);
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_SADDR = (uint32_t)src;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DSCR =  0;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLB =  DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |
    src_incr | DMAC_CTRLB_DST_INCR_FIXED;

  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CFG = DMAC_CFG_DST_PER(SPI_TX_IDX) |
      DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ALAP_CFG;

  dmac_channel_enable(SPI_DMAC_TX_CH);
}


// -----------------------------------------------------------------------------------------------------
// SPI defines

#define slaveSelectPin 10
#define SPI_BUFF_SIZE 4092
uint8_t out_buffer[SPI_BUFF_SIZE + 8];
uint8_t in_buffer[SPI_BUFF_SIZE + 8];
int spiBufferCounter;
uint8_t spiCommFlag = 0;
bool spiDoneFlag = false;

void SPIInitialization(uint8_t _pin) {
  
  SPI.begin(_pin);
  REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
  REG_SPI0_CR = SPI_CR_SPIEN;     // enable SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;   // slave and no modefault
  
  REG_SPI0_CSR = 0x00;    // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer
  // REG_SPI0_CSR = 0x64640000;

  // Configure DMAC
  pmc_enable_periph_clk(ID_DMAC);
  dmac_disable();
  DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
  dmac_enable();

  spiBufferCounter = 0;
}

static bool spiSendReceiveNonBlock(uint8_t * outBuf, uint8_t * inBuf, size_t len) {
  Spi * pSpi = SPI0;
  bool doneFlag = false;

  if (spiCommFlag == 0) {
    Serial.println("Beginning DMA");
    uint32_t s = pSpi->SPI_SR;
    spiDmaRX(inBuf, len);
    spiDmaTX(outBuf, len);
    spiCommFlag = 1;
  }
  else if (spiCommFlag == 1) {
    if (dmac_channel_transfer_done(SPI_DMAC_RX_CH) && dmac_channel_transfer_done(SPI_DMAC_TX_CH)) {
      spiCommFlag = 2;
    }
  }
  else if (spiCommFlag == 2) {
    Serial.println("Waiting for transmission to finish");
    if (pSpi->SPI_SR & SPI_SR_TXEMPTY) {
      uint8_t b = pSpi->SPI_RDR;
      spiCommFlag = 0;
      doneFlag = true;
    }
  }

  return doneFlag;
}


// -----------------------------------------------------------------------------------------------------
// ADC defines

#define ADC_LEN 1024
#define BUFFER_LEN 256

volatile int bufn,obufn;
uint16_t buf[4][BUFFER_LEN];   // 4 buffers of 256 readings

unsigned int channelMask = 0xf000;

bool discardFirstData = 1;

inline static uint8_t convertAnalogChannelToADC(uint8_t analog) {
  return (analog <= 7) ? 7 - analog : analog + 2;
}

inline void enableAnalogChannel(uint8_t pinAnalog) {
  uint8_t pinADC = convertAnalogChannelToADC(pinAnalog);
  ADC->ADC_CHER |= (1 << pinADC);
  ADC->ADC_CHDR &= ~(1 << pinADC);
}

inline void disableAnalogChannel(uint8_t pinAnalog) {
  uint8_t pinADC = convertAnalogChannelToADC(pinAnalog);
  ADC->ADC_CHER &= ~(1 << pinADC);
  ADC->ADC_CHDR |= (1 << pinADC);
}

inline void disableAllChannels() {
  ADC->ADC_CHER = 0x00;
  ADC->ADC_CHDR = ~(0x00);
}

void ADC_Handler() {     // move DMA pointers to next buffer
  
  int f = ADC->ADC_ISR;
  
  if (f&(1<<27)) {
    bufn=(bufn+1)&3;
    ADC->ADC_RNPR=(uint32_t)buf[bufn];
    ADC->ADC_RNCR=256;
  }
  
}

void ADCInitialization() {

  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, 21000000UL, ADC_STARTUP_FAST);
  ADC->ADC_MR |= 0x80; // free running
  
  disableAllChannels();
  enableAnalogChannel(TRANSFORMER1_VOLT_PIN);
  enableAnalogChannel(TRANSFORMER2_VOLT_PIN);
  enableAnalogChannel(TRANSFORMER3_VOLT_PIN);
  enableAnalogChannel(HV1_MONITOR_PIN);
  enableAnalogChannel(HV2_MONITOR_PIN);
  enableAnalogChannel(HV3_MONITOR_PIN);
  
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_IDR = ~(1<<27);
  ADC->ADC_IER = 1<<27;
  
  ADC->ADC_RPR = (uint32_t)buf[0];   // DMA buffer
  ADC->ADC_RCR = 256;
  ADC->ADC_RNPR = (uint32_t)buf[1]; // next DMA buffer
  ADC->ADC_RNCR = 256;
  
  ADC->ADC_EMR |= (1<<24);
  
  bufn=obufn=1;
  ADC->ADC_PTCR=1;
  ADC->ADC_CR=2;
  
}


// -----------------------------------------------------------------------------------------------------
// Control and monitoring data structs

// Defines data input struct
// (contains all monitoring things)
typedef struct sensorData {
  uint16_t highVoltage1;
  uint16_t highVoltage2;
  uint16_t highVoltage3;
  uint16_t transformerVoltage1[ADC_LEN];
  uint16_t transformerVoltage2[ADC_LEN];
  uint16_t transformerVoltage3[ADC_LEN];
} sensorData;

sensorData currentData;

// Defines data output struct
// (contains all control things)
struct controlParams {
  uint8_t controlFlags;
  uint8_t VoltageOut1;
  uint8_t VoltageOut2;
  uint8_t VoltageOut3;
  
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
  
  uint16_t all16bitParams[12];
};

struct controlParams kickerParams;


// -----------------------------------------------------------------------------------------------------
// TaskScheduler definitions

// Callback methods prototypes
void reandAndSendInputsCallback();
void paramsCallbackUpdate();
void HVCallbackON();
void HVCallbackOFF();
void CSCallback();
void BSCallback();
void FSCallback();

Scheduler runner, hpr;

//Tasks
Task readAndSendInputs(0, TASK_FOREVER, &readAndSendInputsCallback);
Task HV(7*MS, 2, &HVCallbackON);
Task CS((997+2*10*(8-1)+197)*MS, TASK_FOREVER, &CSCallback);
Task BS((197+10*(8-1))*MS, 2, &BSCallback);
Task FS(10*MS, 8, &FSCallback);


// -----------------------------------------------------------------------------------------------------
// Function implementations

void checkIfADCBufferFull() {
  if (obufn != bufn && spiBufferCounter < 8) {
      for (unsigned int i = 0; i < 2*BUFFER_LEN; i++) {
  //      if (spiBufferCounter == 0 && (i == 0 || i == 1)) {
  //        out_buffer[0] = 0x4C;
  //        out_buffer[1] = 0x47;
  //      }
  //      else {
          out_buffer[spiBufferCounter * 2 * BUFFER_LEN + i] = (uint8_t)((buf[obufn][i/2] & 0xff00) >> 8);
          i++;
          out_buffer[spiBufferCounter * 2 * BUFFER_LEN + i] = (uint8_t)(buf[obufn][i/2] & 0xff);
  //      }
      }
    spiBufferCounter++;
    obufn = (obufn+1) & 3;
  }
  else {
    if (obufn != bufn)
      obufn = (obufn+1) & 3;
  }
}

bool checkIfOutBufferFull() {
  if (spiBufferCounter == 8) {
    out_buffer[0] = 0x4C;
    out_buffer[1] = 0x47;
    spiDoneFlag = spiSendReceiveNonBlock(out_buffer, in_buffer, 4092);
  }
  if (spiDoneFlag) {

    if (in_buffer[0] != 0x46 || in_buffer[29] != 0x55) {
      Serial.println("[checkIfOutBufferFull] Error! SPI communication didn't pass integrity test! Dropping data...");
      spiBufferCounter = 0;
      spiDoneFlag = false;
      return false;
    }

    //    Serial.println("Read in_buffer:");
    //    String test;
    //    for (int i = 0; i < 28; i++) {
    //      test += String(in_buffer[i], HEX) + ",";
    //    }
    //    Serial.println(test);
    
    kickerParams.controlFlags = in_buffer[1];
    kickerParams.VoltageOut1 = in_buffer[2];
    kickerParams.VoltageOut2 = in_buffer[3];
    kickerParams.VoltageOut3 = in_buffer[4];

    for (int i = 0; i < 2 * 12; i++) {
      kickerParams.all16bitParams[i/2] = ((uint16_t)in_buffer[i+1 + 5] << 8) | in_buffer[i + 5];
      i++;
    }

    kickerParams.CAP1_Time = kickerParams.all16bitParams[0];
    kickerParams.CAP2_Time = kickerParams.all16bitParams[1];
    kickerParams.CAP3_Time = kickerParams.all16bitParams[2];
    kickerParams.CAP1_SCR1_Delay = kickerParams.all16bitParams[3];
    kickerParams.CAP2_SCR2_Delay = kickerParams.all16bitParams[4];
    kickerParams.CAP3_SCR3_Delay = kickerParams.all16bitParams[5];
    kickerParams.SCR1_THYR1_Delay = kickerParams.all16bitParams[6];
    kickerParams.SCR2_THYR2_Delay = kickerParams.all16bitParams[7];
    kickerParams.SCR3_THYR3_Delay = kickerParams.all16bitParams[8];
    kickerParams.Fill_Spacing_Time = kickerParams.all16bitParams[9];
    kickerParams.Bunch_Spacing_Time = kickerParams.all16bitParams[10];
    kickerParams.Cycle_Spacing_Time = kickerParams.all16bitParams[11];
    
    spiBufferCounter = 0;
    spiDoneFlag = false;

    return true;
  }
  return false;
}

void readAndSendInputsCallback() {
  checkIfADCBufferFull();
  bool spiDone = checkIfOutBufferFull();
  if (spiDone) {
    readAndSendInputs.setCallback(&paramsCallbackUpdate);
    readAndSendInputs.forceNextIteration();
  }
}

void paramsCallbackUpdate() {
  
  uint16_t fst = kickerParams.Fill_Spacing_Time;
  uint16_t bst = kickerParams.Bunch_Spacing_Time;
  uint16_t cst = kickerParams.Cycle_Spacing_Time;
  bool TriggerMode = kickerParams.controlFlags & (1 << 0);
  bool RackPowerSupplyStatus = kickerParams.controlFlags & (1 << 1);
  bool HVPowerSupplyStatus = kickerParams.controlFlags & (1 << 2);
//  bool OilPumpStatus = kickerParams.controlFlags & (1 << 3);
//  bool FlourinetPumpStatus = kickerParams.controlFlags & (1 << 4);
  bool GeneralTriggerStatus = kickerParams.controlFlags & (1 << 3);
  bool Kicker_Status_1 = kickerParams.controlFlags & (1 << 4);
  bool Kicker_Status_2 = kickerParams.controlFlags & (1 << 5);
  bool Kicker_Status_3 = kickerParams.controlFlags & (1 << 6);

  if (!GeneralTriggerStatus) {
    if (CS.isEnabled())
      CS.disable();
  }
  else {
    if (HV.getInterval() != kickerParams.CAP1_Time * MS)
      HV.setInterval(kickerParams.CAP1_Time * MS);
    if (FS.getInterval() != fst * MS)
      FS.setInterval(fst * MS);
    if (BS.getInterval() != (bst + fst*(8-1))*MS)
      BS.setInterval((bst + fst*(8-1))*MS);
    if (CS.getInterval() != (cst + 2*fst*(8-1) + bst)*MS)
      CS.setInterval((cst + 2*fst*(8-1) + bst)*MS);

    CS.enableIfNot();
  }

  // Relay pins (need to be inverted for some reason... TODO: figure out why)
  digitalWriteDirect(RACK1_PS_PIN, !RackPowerSupplyStatus);
  digitalWriteDirect(RACK2_PS_PIN, !RackPowerSupplyStatus);
  digitalWriteDirect(RACK3_PS_PIN, !RackPowerSupplyStatus);
  //  digitalWriteDirect(OIL_PUMP_PIN, !OilPumpStatus);
  //  digitalWriteDirect(FLOURINET_PUMP_PIN, !FlourinetPumpStatus);
  digitalWriteDirect(HV1_PS_PIN, !(HVPowerSupplyStatus & Kicker_Status_1));
  digitalWriteDirect(HV2_PS_PIN, !(HVPowerSupplyStatus & Kicker_Status_2));
  digitalWriteDirect(HV3_PS_PIN, !(HVPowerSupplyStatus & Kicker_Status_3));
  
  // Other digital out pins (DON'T need to be inverted)

  // PWM pins
  analogWrite(HV1_CONTROL_PIN, (HVPowerSupplyStatus & Kicker_Status_1) ? kickerParams.VoltageOut1 : 248);
  analogWrite(HV2_CONTROL_PIN, (HVPowerSupplyStatus & Kicker_Status_2) ? kickerParams.VoltageOut2 : 248);
  analogWrite(HV3_CONTROL_PIN, (HVPowerSupplyStatus & Kicker_Status_3) ? kickerParams.VoltageOut3 : 248);

  readAndSendInputs.setCallback(&readAndSendInputsCallback);
}

void HVCallbackOFF() {
  
  digitalWriteDirect(HV1_INHIBIT_PIN, LOW);
  
  delayMicroseconds(kickerParams.CAP1_SCR1_Delay);
  
  digitalWriteDirect(SCR1_CHARGE_PIN, HIGH);
  digitalWriteDirect(SCR1_CHARGE_PIN, LOW);

  //enableAnalogChannel(TRANSFORMER1_VOLT_PIN);
  
  delayMicroseconds(kickerParams.SCR1_THYR1_Delay);
  
  digitalWriteDirect(THYR1_TRIGGER_PIN, HIGH);
  digitalWriteDirect(THYR1_TRIGGER_PIN, LOW);

  //disableAnalogChannel(TRANSFORMER1_VOLT_PIN);
  
  HV.setCallback(&HVCallbackON);
}

void HVCallbackON() {
  digitalWriteDirect(HV1_INHIBIT_PIN, HIGH);
  HV.setCallback(&HVCallbackOFF);
}

void CSCallback() {
    BS.restart();
}

void BSCallback() {
    FS.restart(); 
}

void FSCallback() {
    if (HV.isEnabled()) {
        Serial.println("Skipping a beat...");
        return;
    }
    Scheduler &s = Scheduler::currentScheduler();
    Task &t = s.currentTask();
    if (t.getOverrun() < 0) {
      Serial.print("Overrun: ");
      Serial.println(t.getOverrun());
      Serial.print("startDelay: ");
      Serial.println(t.getStartDelay());
      Serial.print("count number of hv: ");
      Serial.println(HV.getRunCounter());
    }
    //Serial.println("BOS_TRIGGER");
    digitalWriteDirect(BOS_TRIGGER_PIN, HIGH);
    digitalWriteDirect(BOS_TRIGGER_PIN, LOW);

    HV.restart();
    //HV.forceNextIteration();
}


void setup () {
  
  Serial.begin(115200);
  Serial.println("Kicker Control And Monitoring - Arduino Due Sketch");

  analogReadResolution(12);

  ADCInitialization();
  
  SPIInitialization(slaveSelectPin);

  pinMode(BOS_TRIGGER_PIN, OUTPUT);
  pinMode(HV1_INHIBIT_PIN, OUTPUT);
  pinMode(SCR1_CHARGE_PIN, OUTPUT);
  pinMode(THYR1_TRIGGER_PIN, OUTPUT);
  pinMode(RACK1_PS_PIN, OUTPUT);
  pinMode(RACK2_PS_PIN, OUTPUT);
  pinMode(RACK3_PS_PIN, OUTPUT);
//  pinMode(OIL_PUMP_PIN, OUTPUT);
//  pinMode(FLOURINET_PUMP_PIN, OUTPUT);
  pinMode(HV1_PS_PIN, OUTPUT);
  pinMode(HV2_PS_PIN, OUTPUT);
  pinMode(HV3_PS_PIN, OUTPUT);
  pinMode(HV1_CONTROL_PIN, OUTPUT);
  pinMode(HV2_CONTROL_PIN, OUTPUT);
  pinMode(HV3_CONTROL_PIN, OUTPUT);

//  pinMode(HV1_MONITOR_PIN, INPUT_PULLUP);
//  pinMode(TRANSFORMER1_VOLT_PIN, INPUT_PULLUP);
//  pinMode(TRANSFORMER2_VOLT_PIN, INPUT_PULLUP);
//  pinMode(TRANSFORMER3_VOLT_PIN, INPUT_PULLUP);

  
  
  runner.init();
  hpr.init();
  Serial.println("[Main] Initialized scheduler");

  runner.setHighPriorityScheduler(&hpr);
  
  hpr.addTask(CS);
  Serial.println("[Main] Added CS");
  
  hpr.addTask(BS);
  Serial.println("[Main] Added BS");

  hpr.addTask(FS);
  Serial.println("[Main] Added FS");

  hpr.addTask(HV);
  Serial.println("[Main] Added HV");

  runner.addTask(readAndSendInputs);
  Serial.println("[Main] Added readAndSendInputs");

  Serial.println("[Main] Waiting 5 seconds before starting...");
  delay(5000);
  
  CS.enable();
  Serial.println("[Main] Enabled CS");
  readAndSendInputs.enable();
  Serial.println("[Main] Enabled readAndSendInputs");
  
}


void loop () {
  runner.execute();
}
