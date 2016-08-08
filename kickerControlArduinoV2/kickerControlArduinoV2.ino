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

/**
 * Start RX DMA - This function configures the RX channel of the DMA,
 * with many obscure registers configuration. Refer to the beginning comments for the
 * reference to this code.
 */
void spiDmaRX(uint8_t* dst, uint16_t count) {
  dmac_channel_disable(SPI_DMAC_RX_CH);
  
  // Configures the source address register to be the SPI Read register
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_SADDR = (uint32_t)&SPI0->SPI_RDR;
  // Configures the destination address register to be the dst pointer in memory
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DADDR = (uint32_t)dst;
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_DSCR =  0;
  // Configures a bunch of SPI parameters, such as 8 bits per packet,
  // both in the destination and the source
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
  // Configures another bunch of SPI parameters, such as saying that
  // this (RX) is a peripheral to memory (PER2MEM) transaction and that
  // the memory addresses to store the data are incrementing by one, so
  // we only need to provide the first pointer (dst) and the DMA automatically
  // manages the rest
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC |
    DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING;
  // Configures yet some other DMA stuff (not sure what these ones do, but
  // you can find them in the Atmel documentation)
  DMAC->DMAC_CH_NUM[SPI_DMAC_RX_CH].DMAC_CFG = DMAC_CFG_SRC_PER(SPI_RX_IDX) |
    DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;
    
  dmac_channel_enable(SPI_DMAC_RX_CH);
}

/** Start TX DMA - This function configures the TX channel of the DMA,
 *  also with many obscure registers configuration. Refer to the beginning
 *  comments for the reference to this code.  
*/
void spiDmaTX(const uint8_t* src, uint16_t count) {

  // This block of code enforces that the TX channel in DMA
  // always sends some stuff, even if a source pointer is 
  // not provided. In that case, the DMA just keeps sending
  // the word ff, instead of incrementing the src pointer
  static uint8_t ff = 0Xff;
  uint32_t src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;
  if (!src) {
    src = &ff;
    src_incr = DMAC_CTRLB_SRC_INCR_FIXED;
  }
  
  dmac_channel_disable(SPI_DMAC_TX_CH);
  
  // Configures the source address for TX to be the src pointer
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_SADDR = (uint32_t)src;
  // Configures the destination address for TX to be the TDR register
  // of the SPI, which sends the data
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DSCR =  0;
  // Configures the size of the packet to 8 bits
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLA = count |
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;
  // Configure some DMA stuff, such as saying this is a memory to peripheral
  // transaction (MEM2PER)
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLB =  DMAC_CTRLB_SRC_DSCR |
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |
    src_incr | DMAC_CTRLB_DST_INCR_FIXED;
  // Configures yet more DMA stuff, not sure what this does but you can find
  // out in the Atmel documentation
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

/**
 * This function initializes the SPI, and configures
 * the DMA controller (DMAC).
 */
void SPIInitialization(uint8_t _pin) {
  
  SPI.begin(_pin);
  REG_SPI0_CR = SPI_CR_SWRST;     // reset SPI
  REG_SPI0_CR = SPI_CR_SPIEN;     // enable SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;   // slave and no modefault
  
  REG_SPI0_CSR = 0x00;    // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer
  // REG_SPI0_CSR = 0x64640000; // Test with longer delays between transfers

  // Configure DMAC
  pmc_enable_periph_clk(ID_DMAC);
  dmac_disable();
  DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
  dmac_enable();

  spiBufferCounter = 0;
}

/** 
 *  This function performs the SPI transfer between the Due and the Galileo.
 *  This is a non-blocking function (i.e. doesn't wait for the SPI to conclude)
 *  before finishing and that's why we use the spiCommFlag to keep track of where
 *  in the SPI process we currently are. if spiCommFlag is 0, then we are beginning
 *  a SPI transfer, so we first set up the DMA buffers, which takes a little bit of time,
 *  more than we can afford to wait. So at the next run of this function, we check if the
 *  DMA setup is finish, by checking inside the spiCommFlag = 1 if clause if the
 *  the dmac_channel_transfer_done is true for both RX and TX channels. If so then we proceed to
 *  wait until the actual transfer is done, which is indicated by the SPI_SR_TXEMPTY register,
 *  which tells us the tx buffer is now empty.
 */
static bool spiSendReceiveNonBlock(uint8_t * outBuf, uint8_t * inBuf, size_t len) {

  // Create an SPI instance
  Spi * pSpi = SPI0;
  bool doneFlag = false;

  // Check if we are at the beginning of an SPI/DMA cycle
  if (spiCommFlag == 0) {
    Serial.println("Beginning DMA");

    // Need to read status register just in case
    uint32_t s = pSpi->SPI_SR;

    // Tells DMA which in/out buffers to use
    spiDmaRX(inBuf, len);
    spiDmaTX(outBuf, len);

    // Moves workflow to next stage
    spiCommFlag = 1;
  }
  // Check if we are waiting for DMA to finish
  else if (spiCommFlag == 1) {
    if (dmac_channel_transfer_done(SPI_DMAC_RX_CH) && dmac_channel_transfer_done(SPI_DMAC_TX_CH)) {
      // Moves workflow to next stage
      spiCommFlag = 2;
    }
  }
  // Check if we are waiting for SPI to finish
  else if (spiCommFlag == 2) {
    Serial.println("Waiting for transmission to finish");

    // If TXEMPTY is true then we are done
    if (pSpi->SPI_SR & SPI_SR_TXEMPTY) {
      // Read the RDR just to make sure it's empty
      uint8_t b = pSpi->SPI_RDR;
      spiCommFlag = 0;
      doneFlag = true;
    }
  }

  return doneFlag;
}


// -----------------------------------------------------------------------------------------------------
// ADC defines

// Defines the size of the transformer data in number of samples
#define ADC_LEN 1024
// Defines the size of the ADC buffer length
#define BUFFER_LEN 1024
// Maximum ADC buffers in one SPI packet
int maxAdcBuffers = (SPI_BUFF_SIZE + 4)/(2 * BUFFER_LEN);

// Creates 4 buffers that are rotated consecutively to collect data
volatile int bufn,obufn;
uint16_t buf[4][BUFFER_LEN];   // 4 buffers of 256 readings

// This is the channel mask that selects the first 4 bits out of the 16
// for channel id - the other 12 are ADC data
unsigned int channelMask = 0xf000;

/** Converts between ADC channels and Arduino channels */
inline static uint8_t convertAnalogChannelToADC(uint8_t analog) {
  return (analog <= 7) ? 7 - analog : analog + 2;
}

/** Manipulates the ADC channel enable/disable registers
 *  to enable or disable channels for the ADC to read
 */
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

inline void controlAnalogChannel(uint8_t pinAnalog, bool state) {
  if (state) {
    enableAnalogChannel(pinAnalog);
  }
  else {
    disableAnalogChannel(pinAnalog);
  }
}

/** This function is called by the ADC when a buffer is full 
 *  and an action is required. In this case we simply tell it 
 *  to switch to the next buffer. 
 */
void ADC_Handler() {     // move DMA pointers to next buffer

  // Obtain the Interrupt Status Register
  int f = ADC->ADC_ISR;

  if (f&(1<<27)) { // if bit 27 of ISR is on

    // Bitwise operation to increment the buffer index bufn
    // and reset to 0 if it reaches 3
    bufn=(bufn+1)&3;
    // Asigns the buffer address to be the new buf[bufn]
    ADC->ADC_RNPR=(uint32_t)buf[bufn];
    // Size of the buffer
    ADC->ADC_RNCR=BUFFER_LEN;
  }
  
}

/** 
 *  This functions initializes the ADC. We basically use the highest
 *  possible sampling rate and place the ADC in free-running mode, so
 *  that it is always continously acquiring data. The things we control
 *  are which channels are enabled in the ADC at any given time.
 */
void ADCInitialization() {

  // Enables power to the ADC
  pmc_enable_periph_clk(ID_ADC);
  // Initialized ADC with maximum frequency and sampling rate
  //adc_init(ADC, SystemCoreClock, 21000000UL, ADC_STARTUP_FAST);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  // Puts ADC in free-running mode
  ADC->ADC_MR |= 0x80; // free running

  // Enable only the channels we want at first
  // (this will be changed according to kicker params 
  // sent by the webserver)
  disableAllChannels();
  enableAnalogChannel(TRANSFORMER1_VOLT_PIN);
  //  enableAnalogChannel(TRANSFORMER2_VOLT_PIN);
  //  enableAnalogChannel(TRANSFORMER3_VOLT_PIN);
  //  enableAnalogChannel(HV1_MONITOR_PIN);
  //  enableAnalogChannel(HV2_MONITOR_PIN);
  //  enableAnalogChannel(HV3_MONITOR_PIN);

  // Enable interrupts for the ADC (to call the function
  // ADC_Handler() when it saturates the buffer)
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_IDR = ~(1<<27);
  ADC->ADC_IER = 1<<27;

  // Assigns first buffer and the next buffer
  ADC->ADC_RPR = (uint32_t)buf[0];   // DMA buffer
  ADC->ADC_RCR = BUFFER_LEN;
  ADC->ADC_RNPR = (uint32_t)buf[1]; // next DMA buffer
  ADC->ADC_RNCR = BUFFER_LEN;

  // Hmm can't remember what this one does
  ADC->ADC_EMR |= (1<<24);

  // Start the ADC
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

// Tasks
// This defines the tasks and their properties, such as 
// how often and how many times the task should execute,
// and what is the callback function
Task readAndSendInputs(0, TASK_FOREVER, &readAndSendInputsCallback);
Task HV(7*MS, 2, &HVCallbackON);
Task CS((997+2*10*(8-1)+197)*MS, TASK_FOREVER, &CSCallback);
Task BS((197+10*(8-1))*MS, 2, &BSCallback);
Task FS(10*MS, 8, &FSCallback);


// -----------------------------------------------------------------------------------------------------
// Function implementations

/** 
 *  This function is called periodically to check if the ADC buffer
 *  is full, that is, if obufn is different from bufn, which means
 *  that the ADC_Handler() was invoked and updated the buffer. If
 *  there is enough space in the out_buffer, the latest ADC buffer is
 *  written to it, and if not we need to wait for the SPI transmission
 *  to clear the out_buffer and continue writing to it.
 */
void checkIfADCBufferFull() {
  if (obufn != bufn && spiBufferCounter < maxAdcBuffers) { // If ADC is at next buffer and we have space
    for (unsigned int i = 0; i < 2*BUFFER_LEN; i++) {
      // Fills the out_buffer with the latest ADC buffer, splitting 16-bit into two 8-bits
      out_buffer[spiBufferCounter * 2 * BUFFER_LEN + i] = (uint8_t)((buf[obufn][i/2] & 0xff00) >> 8);
      i++;
      out_buffer[spiBufferCounter * 2 * BUFFER_LEN + i] = (uint8_t)(buf[obufn][i/2] & 0xff);
    }
    spiBufferCounter++;
    // Updates obufn to match bufn
    obufn = (obufn+1) & 3;
  }
  else { // ADC updated buffer but we are out of space, just update the buffer index
    if (obufn != bufn)
      obufn = (obufn+1) & 3;
  }
}

/** 
 *  This function also runs periodically in the task scheduler loop
 *  to see if the out_buffer is full, and if it is, proceeds to call
 *  spiSendReceiveNonBlock to send it via SPI.
 */
bool checkIfOutBufferFull() {
  
  if (spiBufferCounter == maxAdcBuffers) {
    // Changes first two bytes to error-correction code
    out_buffer[0] = 0x4C;
    out_buffer[1] = 0x47;
    spiDoneFlag = spiSendReceiveNonBlock(out_buffer, in_buffer, SPI_BUFF_SIZE);
  }

  // SPI takes a while to complete, check if it's done in 
  // the current call to this function
  if (spiDoneFlag) {

    // Check for errors in the transmission
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
    

    // Updates control struct with newly received info
    
    kickerParams.controlFlags = in_buffer[1];
    kickerParams.VoltageOut1 = in_buffer[2];
    kickerParams.VoltageOut2 = in_buffer[3];
    kickerParams.VoltageOut3 = in_buffer[4];

    // Joins the 16-bit data received from the Galileo
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

    // Resets the SPI out_buffer count
    spiBufferCounter = 0;
    spiDoneFlag = false;

    return true;
  }
  return false;
}

/** 
 *  Callback function for readAndSendInputs task - 
 *  Check ADC and SPI buffers, and if complete, calls
 *  the paramsCallbackUpdate to update the control params
 */
void readAndSendInputsCallback() {
  checkIfADCBufferFull();
  bool spiDone = checkIfOutBufferFull();
  if (spiDone) {
    readAndSendInputs.setCallback(&paramsCallbackUpdate);
    readAndSendInputs.forceNextIteration();
  }
}

/**
 * Second callback function for reandAndSendInputs - 
 * acts on the control updates sent by the Galileo,
 * such as trigger delays and timing, and kicker voltages
 */
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

  // Disables the task CS (Cycle spacing) if we don't want triggers
  if (!GeneralTriggerStatus) {
    if (CS.isEnabled())
      CS.disable();
  }
  // Else, adjusts the frequency of the triggers according to the
  // user preferences (sent by Galileo)
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

  // Monitoring analog input pins
  // (Only enables a monitoring channel if 
  // that kicker has been enabled by the user)

  controlAnalogChannel(HV1_MONITOR_PIN, (HVPowerSupplyStatus & Kicker_Status_1));
  controlAnalogChannel(HV2_MONITOR_PIN, (HVPowerSupplyStatus & Kicker_Status_2));
  controlAnalogChannel(HV3_MONITOR_PIN, (HVPowerSupplyStatus & Kicker_Status_3));

  controlAnalogChannel(TRANSFORMER1_VOLT_PIN, (HVPowerSupplyStatus & Kicker_Status_1));
  controlAnalogChannel(TRANSFORMER2_VOLT_PIN, (HVPowerSupplyStatus & Kicker_Status_2));
  controlAnalogChannel(TRANSFORMER3_VOLT_PIN, (HVPowerSupplyStatus & Kicker_Status_3));

  // Wait 10 microseconds to disable the slow monitors
  // (We only need one sample per second of those, would
  // be a waste to keep reading them just to discard all the data,
  // especially with our limitation of just one ADC for all channels)
  
  delayMicroseconds(10);

  controlAnalogChannel(HV1_MONITOR_PIN, false);
  controlAnalogChannel(HV2_MONITOR_PIN, false);
  controlAnalogChannel(HV3_MONITOR_PIN, false);

  // Resets the callback to the original one
  readAndSendInputs.setCallback(&readAndSendInputsCallback);
}

/**
 * Callback function to HV task - one of two
 * Turns off the capacitor charge trigger, waits for
 * specified delay, then toggles the SCR trigger, waits
 * another dealy, then toggles the thyratron trigger
 */
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

/**
 * Other callback function to HV task
 * This just turns on the capacitor charge trigger,
 * and then changes the callback to OFF, which will wait
 * the CAPX_Time seconds to turn it off again
 */
void HVCallbackON() {
  digitalWriteDirect(HV1_INHIBIT_PIN, HIGH);
  HV.setCallback(&HVCallbackOFF);
}

/**
 * Trigger control callback functions
 * Order: CS -> BS -> FS
 * (Cycle -> Bunch -> Fill)
 */
void CSCallback() {
  BS.restart();
}

void BSCallback() {
  FS.restart(); 
}

void FSCallback() {

  // This check is important to ensure that we are 
  // not "skipping a beat", that is, if for some reason
  // the microcontroller wasn't able to complete a full
  // trigger cycle, we don't want to start another one
  // and overlap with the current, because this would
  // destroy the important real-time character of the
  // trigger. Instead we would rather skip a trigger fill
  // and wait for the current one to end so we can keep
  // the timing right. Note that this should rarely if ever
  // happen. To check we ask if HV is still enabled 
  // (it shouldn't if it's over) and if so we just return
  if (HV.isEnabled()) {
      Serial.println("Skipping a beat...");
      return;
  }

  // Another test to see if we got overrun, that is,
  // if this function is executing at a timer later than
  // what it was scheduled to execute. If it is then it's
  // a problem, because we are no longer running in real-time
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
  
  // Begin a trigger cycle with a BOS signal
  digitalWriteDirect(BOS_TRIGGER_PIN, HIGH);
  digitalWriteDirect(BOS_TRIGGER_PIN, LOW);
  
  HV.restart();
  //HV.forceNextIteration();
}


// -----------------------------------------------------------------------------------------------------
// "Main"

void setup () {

  // Initialize Serial, ADC, SPI
  
  Serial.begin(115200);
  Serial.println("Kicker Control And Monitoring - Arduino Due Sketch");

  analogReadResolution(12);

  ADCInitialization();
  
  SPIInitialization(slaveSelectPin);

  // Set up all pins

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


  // Sets up all tasks and task schedulers
  // Runner is the regular scheduler, and hpr
  // is the high-priority scheduler, that only 
  // contains the trigger tasks (these need to be
  // real-time so require high priority over other tasks)
  
  runner.init();
  hpr.init();
  Serial.println("[Main] Initialized schedulers");

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
