//#######################################################################################################
//#################################### Plugin 053: Plantower PTQS1005( porting from PMSx003 )#####################
//#######################################################################################################
//  Build: 20180529
// http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms5003-manual_v2-3.pdf?sfvrsn=2
//
// The PTQS1005 are sensors of air quality, including particles, TVOC, HCHO, CO2, temperature and humidity.
// Particles are measured by blowing air though the enclosue and, togther with a laser, count the amount of particles.
// These sensors have an integrated microcontroller
// that counts particles and transmits measurement data over the serial connection.
/**
   Supported command:
   PTQSCMD,On/Normal    set device to nomal mode. if the workInterval set in the device webpage greater than zero, means Intermittent Mode.
   PTQSCMD,Off/Standby  set device to standby mode. The device will be at standby mode until a Normal/Wakeup command received.
   PTQSCMD,Wakeup       wakeup the device from Standby or sleeping mode, and keep working for 180 seconds.

   Webform Parameters:
   GPIO ← TX:         Should connected to the PIN 5 (TX) of the device
   GPIO → RX:         Should connected to the PIN 4 (RX) of the device
   GPIO → Reset:      Should connected to the PIN 6 (Reset) of the device, Optional
   Please note that the device needs 5V vcc, and 3.3v digital I/O.

   Work Interval:       Zero interval set the device works continuously. To extend sensor's lifetime, suggested interval is greater than 85 seconds,
                        set device works less than 1/4 of the power on time.
                        To get steady data the sensor will work for 35 seconds and sleep for the internval seconds.
   Warm-up time:        When standby to normal mode, sensor will send zero value of tvoc & hcho. about 10 senconds warm-up time fixes it.
    Value formula: None. The values returned by this plugin have been adjusted based on the PTQS1005 datasheet. That is :
     tvoc = %value%/100;     hcho = %value%/100;     temperature = %value%/10;     humidity = %value%/10
*/

/*  Sensor cmd REFERENCE
    CMD：    42 4d e4 00 00 01 73  待机控制-待机模式 standby mode
    CMD：    42 4d e4 00 01 01 74  待机控制-正常模式 normal mode 
    CMD：    42 4d ab 00 00 01 3a  读取颗粒物一般数据 read particle general data
    CMD：    42 4d ad 00 00 01 3c  读取颗粒物tsi模式数据 read particle tsi data
    CMD：    42 4d ac 00 00 01 3b (读取颗粒物完全数据) read particle complete data
           0     2     4     6     8     10     12     14     16    18    20    22    24    26    28    30    32    34    36    38    40 41
    RSPS：42 4D |00 26|00 32|00 52|00 66|00 25 |00 3A |00 50|1C F2 |08 8B|01 D1|00 45|00 18|00 02|01 E2|F8|00 05|6D|03 21|00 EC|01 37|08 B5|
          Header| LEN |pm1.0|pm2.5|pm10 |pm1.0a| p2.5a|p10a |0.3cnt|0.5c |1.0c |2.5c | 5.0c|10c  |tvoc |  |hcho |  |co2  |temp |humi |chksum
     general data idx |0    | 1   |  2  |   3  |   4  |  5  |   6  |   7 |  8  |  9  | 10  |  11 |  12 |13| 14  |15| 16  |  17 | 18  |
  old algorithm d.idx |0    | 1   |  2  |  3   |  4   |  5  |   6  |   7 |  8  |  9  | 10  |  11 |  12 |X | 13  |X | 14  |  15 | 16  |
 *  */


// #ifdef PLUGIN_BUILD_TESTING
#ifdef PLUGIN_BUILD_PLANTOWER

// #define PTQS1005_DEBUG

#include <ESPeasySoftwareSerial.h>

#define USERVAR_MODE

#define PLUGIN_053
#define PLUGIN_ID_053 53
#define PLUGIN_NAME_053 "Dust - PTQS1005"

#ifdef USERVAR_MODE // if we need a device user var which indicates current working mode.
#define PLUGIN_VALUE_COUNT 9 // by Stoney  20180529, mode var added
#else
#define PLUGIN_VALUE_COUNT 8 // by Stoney 20180504
#endif

#define PLUGIN_VALUENAME1_053 "pm1.0"
#define PLUGIN_VALUENAME2_053 "pm2.5"
#define PLUGIN_VALUENAME3_053 "pm10"
#define PLUGIN_VALUENAME4_053 "tvoc"
#define PLUGIN_VALUENAME5_053 "hcho"
#define PLUGIN_VALUENAME6_053 "co2"
#define PLUGIN_VALUENAME7_053 "temperature"
#define PLUGIN_VALUENAME8_053 "humidity"
#define PLUGIN_VALUENAME9_053 "mode"
// PTQS sensor mode
#define MODE_STANDBY  0
#define MODE_SLEEP    1
#define MODE_WARMUP   2
#define MODE_WORKING  3
#define MODE_WORKING_CONTINUOUS 4

// #define DataReady     ( values_received || ( isModeChanged && UserVar[event->BaseVarIndex + 3] >0) )
#define DataReady     ( values_received || isModeChanged )

#define PTQS1005_SIG1 0X42
#define PTQS1005_SIG2 0X4d
// 颗粒物一般数据长度
#define PTQS1005_SIZE 42
// 数据中除去特征字，长度、校验和的 uint16 个数
#define PTQS1005_WORD_COUNT ((PTQS1005_SIZE - 6)/2)
// 3 bytes data index
#define TVOC_DATA_IDX 12
#define HCHO_DATA_IDX 13

// PTQS1005 should work at least 35 seconds to get steady data.
#define STEADY_PERIOD 35
// PTQS1005 will work 180 seconds after wakeup command..
#define WAKEUP_PERIOD 180

ESPeasySoftwareSerial *swSerial = NULL;
boolean Plugin_053_init = false;
boolean values_received = false;

//cmd to read particle full data
uint8_t cmd_read_particle_data[] = {0x42, 0x4d, 0xac, 00, 00, 01, 0x3b};
uint8_t cmd_standby[] = {0x42, 0x4d, 0xe4, 00, 00, 01, 0x73};
uint8_t cmd_normal[] = {0x42, 0x4d, 0xe4, 00, 01, 01, 0x74};

// to control work mode
unsigned long  workInterval = 0;
int warmupSeconds = 10; // PTQS1005 warmup time after standby/sleep mode
unsigned long  workingTimer = 0; // Intermittent mode workingTimer.
unsigned long  sleepTimer = 0; // Intermittent mode sleep Timer.
unsigned long  warmupTimer = 0; // warm-up Timer. delay after sleep or standby, to avoid zero value of tvoc,hcho,etc
boolean isSleeping = false;
boolean isStandby = false;
boolean isWarmingup = false;
boolean isModeChanged = false;
uint8_t currentMode = -1;
// --------------------------------- functions for data reading -----------------------
// Read 2 bytes from serial and make an uint16 of it. Additionally calculate
// checksum for PTQS1005. Assumption is that there is data available, otherwise
// this function is blocking.
void SerialRead16(uint16_t* value, uint16_t* checksum)
{
  uint8_t data_high, data_low;

  // If swSerial is initialized, we are using soft serial
  if (swSerial != NULL)
  {
    data_high = swSerial->read();
    data_low = swSerial->read();
  }
  else
  {
    data_high = Serial.read();
    data_low = Serial.read();
  }

  *value = data_low;
  *value |= (data_high << 8);

  if (checksum != NULL)
  {
    *checksum += data_high;
    *checksum += data_low;
  }

#if 0
  // Low-level logging to see data from sensor
  String log = F("PTQS1005 : byte high=0x");
  log += String(data_high, HEX);
  log += F(" byte low=0x");
  log += String(data_low, HEX);
  log += F(" result=0x");
  log += String(*value, HEX);
  addLog(LOG_LEVEL_INFO, log);
#endif
}
void SerialRead8(uint8_t* value, uint16_t* checksum)
{
  // If swSerial is initialized, we are using soft serial
  if (swSerial != NULL)
  {
    *value = swSerial->read();
  }
  else
  {
    *value = Serial.read();
  }

  if (checksum != NULL)
  {
    *checksum += *value;
  }

#if 0
  // Low-level logging to see data from sensor
  String log = F("PTQS1005 : byte =0x");
  log += String(*value, HEX);
  addLog(LOG_LEVEL_INFO, log);
#endif
}
void SerialFlush() {
  if (swSerial != NULL) {
    swSerial->flush();
  } else {
    Serial.flush();
  }
}
void SerialWrite( uint8_t data[], int len)
{
  for (int i = 0; i < len; i++) {
    if (swSerial != NULL) {
      swSerial->write(data[i]);
    } else {
      Serial.write(data[i]);
    }
  }
  Serial.flush();

}
boolean PacketAvailable(void)
{
  if (swSerial != NULL) // Software serial
  {
    addLog(LOG_LEVEL_DEBUG_DEV, F("PTQS1005 : Using SOFT serial"));
    // When there is enough data in the buffer, search through the buffer to
    // find header (buffer may be out of sync)
    if (!swSerial->available()) {
      addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : No data! check wire OR send data request cmd first!"));
      return false;
    }
    while ((swSerial->peek() != PTQS1005_SIG1) && swSerial->available()) {
      swSerial->read(); // Read until the buffer starts with the first byte of a message, or buffer empty.
    }
    if (swSerial->available() < PTQS1005_SIZE) {
      String log = F("PTQS1005 : Not enough data!" );
      log += swSerial->available();
      addLog(LOG_LEVEL_ERROR, log);
      return false; // Not enough yet for a complete packet
    }
  }
  else // Hardware serial
  {
    // When there is enough data in the buffer, search through the buffer to
    // find header (buffer may be out of sync)
    if (!Serial.available()) return false;
    while ((Serial.peek() != PTQS1005_SIG1) && Serial.available()) {
      Serial.read(); // Read until the buffer starts with the first byte of a message, or buffer empty.
    }
    if (Serial.available() < PTQS1005_SIZE) return false; // Not enough yet for a complete packet
  }
  return true;
}

/**
   a general version for other type of plantower sensors.
   Supposed the data format is known, modification of the data descriptor will make this function work.
*/
#define DeviceDataCount  19
struct SensorDataDescriptor {
  char *dataName;
  uint8_t offset;
  uint8_t byteCount;
} DeviceDataInfo[] = {{"pm1.0", 0, 2}, {"pm2.5", 2, 2},  {"pm10", 4, 2}, {"pm1.0a", 6, 2}, {"pm2.5a", 8, 2},  {"pm10a", 10, 2},
  {"pc0.3", 12, 2}, {"pc0.5", 14, 2},  {"pc1.0", 16, 2}, {"pc2.5", 18, 2}, {"pc5.0", 20, 2}, {"pc10", 22, 2},
  {"tvoc", 24, 2}, {"tvocequ", 26, 1}, {"hcho", 27, 2}, {"hchoequ", 29, 1}, {"co2", 30, 2}, {"temperature", 32, 2}, {"humidity", 34, 2}
};

boolean Plugin_053_process_data_general(struct EventStruct *event) {
  String log;
  uint16_t checksum = 0, checksum_read = 0;
  uint16_t framelength = 0;
  uint16 packet_header = 0;
  SerialRead16(&packet_header, &checksum); // read PTQS1005_SIG1 + PTQS1005_SIG2
  if (packet_header != ((PTQS1005_SIG1 << 8) | PTQS1005_SIG2)) {
    // Not the start of the packet, stop reading.
    return false;
  }

  SerialRead16(&framelength, &checksum);
  if (framelength != (PTQS1005_SIZE - 4))
  {
    log = F("PTQS1005 : invalid framelength - ");
    log += framelength;
    addLog(LOG_LEVEL_ERROR, log);
    return false;
  }

  uint8_t databuf[ PTQS1005_SIZE - 6 ] ; // data buffer, excluding header and checksum
  for (int i = 0; i < PTQS1005_SIZE - 6 ; i++)  // read all of the packet
  {
    SerialRead8(&databuf[i], &checksum);
  }

  uint16_t data16;
  log = F("PTQS1005 : ");
  for (int i = 0; i < DeviceDataCount ; i++) {
    String nstr(DeviceDataInfo[i].dataName);
    log += nstr;
    log += F("=");
    switch (DeviceDataInfo[i].byteCount) {
      case 1:
        {
          log += databuf[DeviceDataInfo[i].offset] ;
          break;
        }
      case 2:
        {
          getData16(&data16, &databuf[DeviceDataInfo[i].offset]);
          log += data16;
          break;
        }
      default:
        {
          log += F("Unsupported byte count of data:");
          log += DeviceDataInfo[i].byteCount;
        }
    }
    log += F(", ");
    if (0 == ( i + 1 ) % 6)
      log += F("\n\t");
  }
  addLog(LOG_LEVEL_DEBUG, log);

  // Compare checksums
  SerialRead16(&checksum_read, NULL);
  SerialFlush(); // Make sure no data is lost due to full buffer.
  if (checksum == checksum_read)
  {
    // Data is checked and good, fill in output
    getData16(&data16, &databuf[DeviceDataInfo[3].offset]);   UserVar[event->BaseVarIndex] = data16;     // pm1.0a
    getData16(&data16, &databuf[DeviceDataInfo[4].offset]);   UserVar[event->BaseVarIndex + 1] = data16; // pm2.5a
    getData16(&data16, &databuf[DeviceDataInfo[5].offset]);   UserVar[event->BaseVarIndex + 2] = data16; // pm10a
    getData16(&data16, &databuf[DeviceDataInfo[12].offset]);  UserVar[event->BaseVarIndex + 3] = data16/100.0 ; // TVOC = value / 100
    getData16(&data16, &databuf[DeviceDataInfo[14].offset]);  UserVar[event->BaseVarIndex + 4] = data16/100.0 ; // HCHO = value / 100
    getData16(&data16, &databuf[DeviceDataInfo[16].offset]);  UserVar[event->BaseVarIndex + 5] = data16; // CO2
    getData16(&data16, &databuf[DeviceDataInfo[17].offset]);  UserVar[event->BaseVarIndex + 6] = data16 /10.0; // temperature = value / 10
    getData16(&data16, &databuf[DeviceDataInfo[18].offset]);  UserVar[event->BaseVarIndex + 7] = data16 /10.0; // humidity = value / 10

    values_received = true;
    return true;
  } else {
    addLog(LOG_LEVEL_ERROR, F("PTQS1005 : Checksum NOT macth! data ignored!"));
  }
  return false;
}
void getData16( uint16_t *value, uint8_t *offset )
{
  uint8_t data_high = *offset, data_low = *(offset + 1);

  *value = data_low;
  *value |= (data_high << 8);
}

// --------------------------------- functions for sensor mode control & device mode switching -------------------

/**
    send work mode control cmd to device.
*/
boolean PtqsSetWorkMode(boolean isNormalMode)
{
  if (isNormalMode) {
    SerialWrite(cmd_normal, 7);
  } else {
    SerialWrite(cmd_standby, 7);
  }
  return true; //should read response and decide if success or not
}
void PtqsCmd_Standby() //set sensor to standby mode
{
  if (PtqsSetWorkMode(false) ) {
    isStandby = true;
  }
  addLog(LOG_LEVEL_INFO, F("PTQS1005: Set to Standby mode."));
}
void PtqsCmd_Normal()//set sensor to Normal mode
{
  if (PtqsSetWorkMode(true)) {
    isStandby = false;
    workingTimer = 0; // trigger a reset of workingTimer
  }
  addLog(LOG_LEVEL_INFO, F("PTQS1005: Set to Normal mode."));
}

void PtqsCmd_Wakeup() // wakeup the device from sleeping/standby, and keep working for WAKEUP_PERIOD seconds.
{
  String msg =  F("PTQS1005: Device activated from ");
  if (isStandby) {
    msg +=  F("STANDBY mode.");
  } else if (isSleeping) {
    msg +=  F("SLEEPING mode.");
  }
  // workingTimer = millis() + WAKEUP_PERIOD * 1000; //reset workingTimer to WAKEUP_PERIOD
  workingTimer = 0; // trigger a reset of workingTimer
  isSleeping = false;
  isStandby = false;
  PtqsSetWorkMode(true); //make sure device at Normal mode

  addLog(LOG_LEVEL_INFO, msg);
}
boolean updateWorkMode(uint8_t themode, struct EventStruct *event )
{
  boolean success = false;
  if (currentMode != themode) {
    currentMode = themode;
#ifdef USERVAR_MODE
    UserVar[event->BaseVarIndex + 8] = themode;
#endif
    success = true;
    isModeChanged = true;
    
#ifdef PTQS1005_DEBUG
    String log = F("PTQS1005: Work Mode Changed, tvoc value:");
    log +=  UserVar[event->BaseVarIndex + 3];
    addLog(LOG_LEVEL_DEBUG, log);
#endif
  }
  return success;
}

// --------------------------------- Plugin function  --------------------------

boolean Plugin_053(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_053;
        Device[deviceCount].Type = DEVICE_TYPE_TRIPLE;
        Device[deviceCount].VType = SENSOR_TYPE_TRIPLE;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = PLUGIN_VALUE_COUNT;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        //Settings.TaskDevicePluginConfig[event->TaskIndex][1] = 5;
        success = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_053);
        success = true;
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[4], PSTR(PLUGIN_VALUENAME5_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[5], PSTR(PLUGIN_VALUENAME6_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[6], PSTR(PLUGIN_VALUENAME7_053));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[7], PSTR(PLUGIN_VALUENAME8_053));
#ifdef USERVAR_MODE
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[8], PSTR(PLUGIN_VALUENAME9_053));
#endif
        success = true;
        break;
      }

    case PLUGIN_GET_DEVICEGPIONAMES:
      {
        event->String1 = F("GPIO &larr; TX");
        event->String2 = F("GPIO &rarr; RX");
        event->String3 = F("GPIO &rarr; Reset");
        break;
      }

    case PLUGIN_INIT:
      {
        int rxPin = Settings.TaskDevicePin1[event->TaskIndex];
        int txPin = Settings.TaskDevicePin2[event->TaskIndex];
        int resetPin = Settings.TaskDevicePin3[event->TaskIndex];

        workInterval = Settings.TaskDevicePluginConfig[event->TaskIndex][0];
        warmupSeconds = Settings.TaskDevicePluginConfig[event->TaskIndex][1];

        String log = F("PTQS1005 : config ");
        log += rxPin;
        log += txPin;
        log += resetPin;
        addLog(LOG_LEVEL_INFO, log);

        if (swSerial != NULL) {
          // Regardless the set pins, the software serial must be deleted.
          delete swSerial;
          swSerial = NULL;
        }

        // Hardware serial is RX on 3 and TX on 1
        if (rxPin == 3 && txPin == 1)
        {
          log = F("PTQS1005 : using hardware serial");
          addLog(LOG_LEVEL_INFO, log);
          Serial.begin(9600);
          Serial.flush();
        }
        else
        {
          log = F("PTQS1005: using software serial");
          addLog(LOG_LEVEL_INFO, log);
          swSerial = new ESPeasySoftwareSerial(rxPin, txPin, false, PTQS1005_SIZE * 3); // PTQS1005_SIZE * 3 Bytes buffer, enough for up to 3 packets.
          swSerial->begin(9600);
          swSerial->flush();
        }

        if (resetPin >= 0) // Reset if pin is configured
        {
          // Toggle 'reset' to assure we start reading header
          log = F("PTQS1005: resetting module");
          addLog(LOG_LEVEL_INFO, log);
          pinMode(resetPin, OUTPUT);
          digitalWrite(resetPin, LOW);
          delay(250);
          digitalWrite(resetPin, HIGH);
          pinMode(resetPin, INPUT_PULLUP);
        }

        PtqsSetWorkMode(true); //Ensure  device at normal mode at power on.

        Plugin_053_init = true;
        success = true;
        break;
      }

    case PLUGIN_EXIT:
      {
        if (swSerial)
        {
          delete swSerial;
          swSerial = NULL;
        }
        break;
      }

    // The update rate from the module is 200ms .. multiple seconds. Practise
    // shows that we need to read the buffer many times per seconds to stay in
    // sync.
    //case PLUGIN_TEN_PER_SECOND:
    case PLUGIN_ONCE_A_SECOND:
      {
        if (!Plugin_053_init) break;
        if ( isStandby) {
          success = updateWorkMode(MODE_STANDBY,event);
          break;
        }

        if (workInterval > 0) // Intermittent mode
        {
          if (workingTimer == 0 || (isSleeping && timeOutReached(sleepTimer) )) { //When startup or sleeping enough
            PtqsSetWorkMode(true);
            isSleeping = false;
            workingTimer = millis() + STEADY_PERIOD * 1000;
            isWarmingup = true;
            warmupTimer =  millis() + warmupSeconds * 1000;
            success = updateWorkMode(MODE_WARMUP,event);
            addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Intermittent mode, Start warming-up ..."));
          }
          if ( !isSleeping ) {
            if ( timeOutReached(workingTimer) )
            {
              PtqsSetWorkMode(false);
              isSleeping = true;
              sleepTimer =  millis() + workInterval * 1000;
              addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Intermittent mode, Enter Sleeping ..."));
            } else if ( isWarmingup ) {
              if ( timeOutReached( warmupTimer ) ) {
                isWarmingup = false;
              } else {
                success = updateWorkMode(MODE_WARMUP,event);
                addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Warming up ..."));
              }
            } else {
              success = updateWorkMode(MODE_WORKING,event);
              addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Intermittent mode, Working ..."));
            }
          } else {
            success = updateWorkMode(MODE_SLEEP,event);
            addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Intermittent mode, Sleeping ..."));
          }
        } else {
          isSleeping = false;
          success = updateWorkMode(MODE_WORKING_CONTINUOUS,event);
          addLog(LOG_LEVEL_DEBUG_MORE, F("PTQS1005 : Continuous mode."));
        }

        if ( !isSleeping ) // read data even at warming-up mode.
        {
          // addLog(LOG_LEVEL_DEBUG, F("PTQS1005 : Inited"));
          // Check if a complete packet is available in the UART FIFO.
          if (PacketAvailable())
          {
            addLog(LOG_LEVEL_DEBUG_MORE, F("PTQS1005 : Packet available"));
            
            float tvoc = UserVar[event->BaseVarIndex + 3];
            float hcho = UserVar[event->BaseVarIndex + 4];
            success = Plugin_053_process_data_general(event);
            if (values_received && currentMode == MODE_WARMUP && (UserVar[event->BaseVarIndex + 3]<=0 || UserVar[event->BaseVarIndex + 4] <=0 )  ){
              // when a zero value received in MODE_WARMUP, restore tvoc and hcho to previous values. and invalidate values_received.
              // to avoid zero tvoc, hcho when  isModeChanged trigger a device var update 
              UserVar[event->BaseVarIndex + 3] = tvoc;
              UserVar[event->BaseVarIndex + 4] = hcho;
              String log =  F("PTQS1005 : Restore tvoc to none-zero:");
              log += tvoc;
              addLog(LOG_LEVEL_DEBUG,log);
              values_received = false;
            }
            //values_received = values_received && ( currentMode > MODE_WARMUP  || (currentMode == MODE_WARMUP && UserVar[event->BaseVarIndex + 3] > 0) );
          } else {
            addLog(LOG_LEVEL_DEBUG_MORE, F("PTQS1005 : Sending data request CMD"));
            SerialWrite(cmd_read_particle_data, 7);
          }
        }
        break;
      }
    case PLUGIN_READ:
      {
#ifdef PTQS1005_DEBUG
        String log = F("PTQS1005 : PLUGIN_READ: Sensor data ");
        // When new data is available, return true
        if (DataReady) {
          log += F("Ready, value_recived:");
          log += values_received;
          log += F("Current tvoc:") ;
          log += UserVar[event->BaseVarIndex + 3];
        } else {
          log += F("NOT Ready, value_recived:");
          log += values_received;
          log += F("Current tvoc:") ;
          log += UserVar[event->BaseVarIndex + 3];
        }
        addLog(LOG_LEVEL_DEBUG, log);
#endif
        success = DataReady;
        values_received = false;
        isModeChanged = false;
        break;
      }
    case PLUGIN_WRITE: //Stoney: 处理命令
      {
        String tmpString  = string;
        int argIndex = tmpString.indexOf(',');
        if (argIndex)
          tmpString = tmpString.substring(0, argIndex);
        if (tmpString.equalsIgnoreCase(F("PTQSCMD")))
        {
          success = true;
          argIndex = string.lastIndexOf(',');
          tmpString = string.substring(argIndex + 1);
          if (tmpString.equalsIgnoreCase(F("Off")) || tmpString.equalsIgnoreCase(F("Standby")))
            PtqsCmd_Standby();
          else if (tmpString.equalsIgnoreCase(F("On")) || tmpString.equalsIgnoreCase(F("Normal")))
            PtqsCmd_Normal();
          else if ( tmpString.equalsIgnoreCase(F("Wakeup")) )
            PtqsCmd_Wakeup();
        }

        break;
      }
    //    case PLUGIN_CLOCK_IN:
    //    {
    //      addLog(LOG_LEVEL_DEBUG_MORE, F("PTQS1005 : Clock In!"));
    //      break;
    //    }
    case PLUGIN_WEBFORM_LOAD:
      {
        //  Since there's no DEVICE_TYPE_QUARD available, if the device has 4 digital IO, the following line should be useful.
        // addFormPinSelect(string, F("GPIO &rarr; SET"), F("SETPIN"), (int)(Settings.TaskDevicePin[3][event->TaskIndex]));

        addFormSubHeader(string, F("Intermittent Use"));

        addFormNumericBox(string, F("Work Interval"), F("WORKINTERVAL"), Settings.TaskDevicePluginConfig[event->TaskIndex][0]);
        addUnit(string, F("seconds"));

        String noteStr = F("Zero interval set the device works continuously. To extend sensor's lifetime, suggested interval is greater than 85 seconds, set device works less than 1/4 of the power on time.");
        noteStr += F("<br>To get steady data the sensor will work for ") ;
        noteStr += STEADY_PERIOD;
        noteStr += F(" seconds and sleep for the internval seconds.");
        addFormNote(string, noteStr );


        warmupSeconds = Settings.TaskDevicePluginConfig[event->TaskIndex][1];
        warmupSeconds = warmupSeconds <= 0 ? 10 : warmupSeconds;
        warmupSeconds = warmupSeconds > 30 ? 30 : warmupSeconds;
        addFormNumericBox(string, F("Warm-up Time"), F("WARMUPTIME"), warmupSeconds);
        addUnit(string, F("seconds"));
        addFormNote(string, F("Increase this value if TVOC and HCHO get zero value. MUST be LESS than 30 or your will get NO data from PTQS1005.") );
        success = true;
        break;
      }
    case PLUGIN_WEBFORM_SAVE:
      {
        // get the 4th GPIO pin number.
        // Settings.TaskDevicePin[3][event->TaskIndex] = (int8_t)getFormItemInt(F("SETPIN"));

        Settings.TaskDevicePluginConfig[event->TaskIndex][0] = getFormItemInt(F("WORKINTERVAL"));
        Settings.TaskDevicePluginConfig[event->TaskIndex][1] = getFormItemInt(F("WARMUPTIME"));
        workInterval = Settings.TaskDevicePluginConfig[event->TaskIndex][0];
        if (workInterval == 0) {
          // Continuous mode
          PtqsSetWorkMode(true);
        }
        warmupSeconds = Settings.TaskDevicePluginConfig[event->TaskIndex][1];
        success = true;
        break;
      }

  }
  return success;
}
#endif // PLUGIN_BUILD_PTQS
