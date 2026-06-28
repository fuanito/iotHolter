/*
 * IoTHolter Firmware
 *
 * Copyright (c) 2026 Ruth Escobedo-Carranza et al.
 *
 * This firmware incorporates components derived from:
 *
 * 1. Walter Modem Library and example code
 *    Copyright (C) 2026 DPTechnics bv
 *    Distributed under the DPTechnics license.
 *
 * 2. Simplest 3-Lead 24-bit ECG with an ads1293 and arduino Mega 2560
 *    Copyright (c) 2018 Sergio Rivera
 *    Licensed under the MIT License.
 *
 * Original copyright notices and license terms have been preserved.
 *
 * The remaining source code, including the ECG acquisition workflow,
 * MQTT message generation, buffering strategy, dual-core implementation,
 * and platform integration, was developed as part of the IoTHolter project.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <pgmspace.h>
#include <esp_mac.h>
#include <math.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include <SPI.h>

#define MQTTS_PORT 8883
#define MQTTS_HOST "mqtt.iotholter.com"
#define MQTTS_TOPIC "iotholter/device/002/ecg"
#define MQTTS_USERNAME "USER_HERE"
#define MQTTS_PASSWORD "PASSWORD_HERE" 

const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

//Variables buffers
const int BLOCK_SIZE = 50;
int32_t bufferA[BLOCK_SIZE][3];
int32_t bufferB[BLOCK_SIZE][3];
int32_t (*acqBuffer)[3] = bufferA;
int32_t (*txBuffer)[3]  = bufferB;
volatile bool txReady = false;
uint64_t txT0 = 0;
uint32_t txSeq = 0;
QueueHandle_t mqttQueue;
typedef struct{int32_t (*buffer)[3]; uint64_t t0; uint32_t seq;} ECGBlock;

//spi
const int pin_DRDYB = 6;  // data ready
const int VSPI_MISO = 13;
const int VSPI_MOSI = 11;
const int VSPI_SCLK = 12;
const int VSPI_SS = 10;//csb

int32_t ecgTmp = 0;
int32_t ecgTmp2 = 0;
int32_t ecgTmp02 = 0;
int32_t ecgTmp22 = 0;
int32_t ecgTmp03 = 0;
int32_t ecgTmp23 = 0;
int32_t ecgVal, ecgVal2, ecgVal3;

//json
uint16_t sampleIndex = 0;
uint32_t seq = 0;
uint64_t t0 = 0;
float phase = 0.0;
int32_t canal1, canal2, canal3;
//timestamp
uint64_t baseEpochMs = 0;
uint32_t bootMillis = 0;

uint64_t epochMillis()
{
    return baseEpochMs + (millis() - bootMillis);
}

#define MQTTS_TLS_PROFILE 2
WalterModem modem;
WalterModemRsp rsp;
bool mqtt_connected = false;
uint8_t out_buf[32] = { 0 };
uint8_t in_buf[4096] = { 0 };

bool lteConnected(){
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME || regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool waitForNetwork(int timeout_sec = 300){
  Serial.print("Connecting to the network...");
  int time = 0;
  while(!lteConnected()) {
    Serial.print(".");
    delay(1000);
    time++;
    if(time > timeout_sec)
      return false;
  }
  Serial.println();
  Serial.println("Connected to the network");
  return true;
}

bool lteDisconnect(){
  /* Set the operational state to minimum */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Successfully set operational state to MINIMUM");
  } else {
    Serial.println("Error: Could not set operational state to MINIMUM");
    return false;
  }
  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.println("Disconnected from the network");
  return true;
}

bool lteConnect(){
  /* Set the operational state to NO RF */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }
  /* Create PDP context */
  if(modem.definePDPContext()) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }
  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }
  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }
  return waitForNetwork();
}

bool setupTLSProfile(void){
  if(!modem.tlsWriteCredential(false, 12, ca_cert)) {
    Serial.println("Error: CA cert upload failed");
    return false;
  }
  if(modem.tlsConfigProfile(MQTTS_TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_NONE,WALTER_MODEM_TLS_VERSION_12, 12)){
    Serial.println("TLS profile configured");
  }
  else {
    Serial.println("Error: TLS profile configuration failed");
    return false;
  }
  return true;
}

static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData* data,void* args){
  if(event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE) {
    switch(data->cereg.state) {
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
      Serial.println("Network registration: Registered (home)");
      break;
    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
      Serial.println("Network registration: Registered (roaming)");
      break;
    case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
      Serial.println("Network registration: Not searching");
      break;
    case WALTER_MODEM_NETWORK_REG_SEARCHING:
      Serial.println("Network registration: Searching");
      break;
    case WALTER_MODEM_NETWORK_REG_DENIED:
      Serial.println("Network registration: Denied");
      break;
    case WALTER_MODEM_NETWORK_REG_UNKNOWN:
      Serial.println("Network registration: Unknown");
      break;
    default:
      break;
    }
  }
}

static void myMQTTEventHandler(WMMQTTEventType event, const WMMQTTEventData* data, void* args)
{
  switch(event) {
  case WALTER_MODEM_MQTT_EVENT_CONNECTED:
    if(data->rc != 0) {
      Serial.printf("MQTT: Connection could not be established. (code: %d)\r\n", data->rc);
    } else {
      Serial.printf("MQTT: Connected successfully\r\n");
      /* Subscribe to the test topic */
      
      if(modem.mqttSubscribe(MQTTS_TOPIC)) {
        Serial.printf("Subscribing to '%s'...\r\n", MQTTS_TOPIC);
      } else {
        Serial.println("Subscribing failed");
      }
    }
    break;
  case WALTER_MODEM_MQTT_EVENT_DISCONNECTED:
    if(data->rc != 0) {
      Serial.printf("MQTT: Connection was interrupted (code: %d)\r\n", data->rc);
    } else {
      Serial.printf("MQTT: Disconnected\r\n");
    }
    mqtt_connected = false;
    break;   
  case WALTER_MODEM_MQTT_EVENT_SUBSCRIBED:
    if(data->rc != 0) {
      Serial.printf("MQTT: Could not subscribe to topic. (code: %d)\r\n", data->rc);
    } else {
      Serial.printf("MQTT: Successfully subscribed to topic '%s'\r\n", data->topic);
      mqtt_connected = true;
    }
    break;
  default:
    break;
  }
}

static bool mqttPublishMessage(const char* topic, const char* message){
  if(!modem.mqttPublish(topic, (uint8_t*) message, strlen(message))){
    Serial.println("Publishing failed");
    return false;
  }
  return true;
}

void enviarBloqueMQTT(int32_t buffer[][3],uint64_t t0,uint32_t seq){
    StaticJsonDocument<1000> doc;
    doc["seq"] = seq;
    doc["fs"] = 250;
    doc["t0"] = t0;
    JsonArray samples = doc.createNestedArray("samples");
    for(int i=0; i<BLOCK_SIZE; i++){
        JsonArray fila = samples.createNestedArray();
        fila.add(buffer[i][0]);
        fila.add(buffer[i][1]);
        fila.add(buffer[i][2]);
    }
    String payload;
    serializeJson(doc, payload);
    if (!mqttPublishMessage(MQTTS_TOPIC, payload.c_str())){
      if(!lteConnected()) {
        if(!lteConnect()) {
          Serial.println("Error: Failed to connect to network");
          delay(1000);
          ESP.restart();
        }
        mqtt_connected = false;
      }
    }
}


void taskMQTT(void *pvParameters){
    ECGBlock block;
    while(true) {
      if(xQueueReceive(mqttQueue,&block,portMAX_DELAY)){
        enviarBloqueMQTT(block.buffer,block.t0,block.seq);
      }
    }
}

void setup(){
  pinMode(pin_DRDYB, INPUT);
  pinMode(VSPI_SS,   OUTPUT);
  
  Serial.begin(115200);
  delay(2000);
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  setup_ECG();
  mqttQueue = xQueueCreate(4,sizeof(ECGBlock));
  Serial.printf("\r\n\r\n=== WalterModem MQTTS example (Arduino v1.5.0) ===\r\n\r\n");
  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sprintf((char*) out_buf, "walter-%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
          mac[4], mac[5]);
  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } 
  else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }
  /* Set the network event handler (optional) */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);
  /* Set the MQTT event handler */
  modem.setMQTTEventHandler(myMQTTEventHandler, NULL);
  /* Set up the TLS profile */
  if(setupTLSProfile()) {
    Serial.println("TLS Profile setup succeeded");
  } 
  else {
    Serial.println("Error: TLS Profile setup failed");
    return;
  }
  /* Configure the MQTTS client */
  if(modem.mqttConfig((char*) out_buf, MQTTS_USERNAME, MQTTS_PASSWORD, MQTTS_TLS_PROFILE)) {
    Serial.println("Successfully configured the MQTT client");
  } else {
    Serial.println("Error: Failed to configure MQTT client");
    return;
  }
  if(!lteConnected()) {
    if(!lteConnect()) {
      Serial.println("Error: Failed to connect to network");
      delay(1000);
      ESP.restart();
  }}

  if(modem.getClock(&rsp)){
      baseEpochMs = (uint64_t)rsp.data.clock.epochTime * 1000ULL;
      bootMillis = millis();
  }
 if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,&rsp)){
    Serial.println("Informacion obtenida");
    Serial.print("RSSI: ");
    Serial.println(rsp.data.cellInformation.rssi);
    Serial.print("RSRP: ");
    Serial.println(rsp.data.cellInformation.rsrp);
    Serial.print("RSRQ: ");
    Serial.println(rsp.data.cellInformation.rsrq);
  }
  else{
    Serial.println("Error getCellInformation");
  }
  xTaskCreatePinnedToCore(taskMQTT,"taskMQTT",10000,NULL,1,NULL,1); 
}

void loop(){
  /* Connect to a public MQTT broker */
  if(!mqtt_connected) {
    Serial.println(MQTTS_HOST);
    Serial.println(MQTTS_PORT);
    if(modem.mqttConnect(MQTTS_HOST, MQTTS_PORT)) {
      Serial.println("Connecting to MQTT broker...");
    } else {
      Serial.println("Error: Failed to connect to MQTT broker");
    }
    delay(5000);
    return;
  }

  if(sampleIndex == 0){
    t0 = epochMillis();
    //Serial.println(millis());
  }
  if (digitalRead(pin_DRDYB) == false) {
  //CH1
  // sampled data is located at 3 8-bit registers 
  byte x1 = readRegister(0x37);
  byte x2 = readRegister(0x38);
  byte x3 = readRegister(0x39);

  // 3 8-bit registers combination on a 24 bit number
  ecgVal = x1;
  ecgVal = (ecgVal << 8) | x2;
  ecgVal = (ecgVal << 8) | x3;

  // exponential smoothing as LPF
  //    1) short range smoothing
  ecgTmp = ecgTmp * .5 + ecgVal * .5;
  //    2) Baseline
  ecgTmp2 = ecgTmp2 * .90 + ecgVal * .10;

  //CH2 
  byte x12 = readRegister(0x3A);
  byte x22 = readRegister(0x3B);
  byte x32 = readRegister(0x3C);
  ecgVal2 = x12;
  ecgVal2 = (ecgVal2 << 8) | x22;
  ecgVal2 = (ecgVal2 << 8) | x32;
  ecgTmp02 = ecgTmp02 * .5 + ecgVal2 * .5;
  ecgTmp22 = ecgTmp22 * .90 + ecgVal2 * .10;
  
  //CH3
  byte x13 = readRegister(0x3D);
  byte x23 = readRegister(0x3E);
  byte x33 = readRegister(0x3F);
  ecgVal3 = x13;
  ecgVal3 = (ecgVal3 << 8) | x23;
  ecgVal3 = (ecgVal3 << 8) | x33;
  ecgTmp03 = ecgTmp03 * .5 + ecgVal3 * .5;
  ecgTmp23 = ecgTmp23 * .90 + ecgVal3 * .10;

  int32_t canal1 = ecgTmp - ecgTmp2;
  int32_t canal2 = ecgTmp02 - ecgTmp22;
  int32_t canal3 = ecgTmp03 - ecgTmp23;

  acqBuffer[sampleIndex][0] = canal1;
  acqBuffer[sampleIndex][1] = canal2;
  acqBuffer[sampleIndex][2] = canal3;

  sampleIndex++;
  phase += 0.05;
  if (phase > 2.0 * PI){
    phase -= 2.0 * PI;
  }
  if(sampleIndex >= BLOCK_SIZE){
      txT0 = t0;
      txSeq = seq;
      int32_t (*tmp)[3] = txBuffer;
      txBuffer = acqBuffer;
      acqBuffer = tmp;
      ECGBlock block;
      block.buffer = txBuffer;
      block.t0 = txT0;
      block.seq = txSeq;
      xQueueSend(mqttQueue,&block,0);
      sampleIndex = 0;
      seq++;
  }
  delay(4);
  } 
  /*
  else{
    Serial.println("DRDYB error");
    delay(2000);
  }*/
}

void setup_ECG() { // datasheet ads1293
  //Follow the next steps to configure the device for this example, starting from default registers values.
  //1. Set address 0x01 = 0x11: Connect channel 1’s INP to IN2 and INN to IN1.
  writeRegister(0x01, 0x11);
  //2. Set address 0x02 = 0x19: Connect channel 2’s INP to IN3 and INN to IN1.
  writeRegister(0x02, 0x19);
  //3. Set addres 0x03: Connect channel 3’s INP to IN5 and INN to IN6.
  writeRegister(0x03, 0x2E); 
  //4. Set address 0x0A = 0x07: Enable the common-mode detector on input pins IN1, IN2 and IN3.
  writeRegister(0x0A, 0x07);
  //5. Set address 0x0C = 0x04: Connect the output of the RLD amplifier internally to pin IN4.
  writeRegister(0x0C, 0x04);
  //6. Set adresses 0x0D, 0x0E, 0x0F: Connects the firt buffer of the Wilaon reference to IN1 pin, the second buffer to the IN2 pin and the third buffer to the IN3 pin.
  writeRegister(0x0D, 0x01);
  writeRegister(0x0E, 0x02);
  writeRegister(0x0F, 0x03);
  //7. Set address 0x10: Connects the output of the Wilson reference internally to IN6.
  writeRegister(0x10, 0x01);//output wilson
  //8. Set address 0x12 = 0x04: Use external crystal and feed the internal oscillator's output to the digital.
  writeRegister(0x12, 0x04);
  writeRegister(0x14, 0x00);
  //9. Set address 0x21 = 0x02: Configures the R2 decimation rate as 5 for all channels.
  writeRegister(0x21, 0x02);
  //10. Set address 0x22 = 0x02: Configures the R3 decimation rate as 6 for channel 1.
  writeRegister(0x22, 0x02);
  //11. Set address 0x23 = 0x02: Configures the R3 decimation rate as 6 for channel 2.
  writeRegister(0x23, 0x02);
  //12. Set address 0x24: Configures the R3 decimation rate as 6 for channel 3.
  writeRegister(0x24, 0x02);
  //13. Set address 0x27 = 0x08: Configures the DRDYB source to channel 1 ECG (or fastest channel).
  writeRegister(0x27, 0x08);
  //14. Set address 0x2F = 0x30: Enables channel 1 ECG, channel 2 ECG and channel 3 ECG for loop read-back mode.
  writeRegister(0x2F, 0x70);
  //15. Set address 0x00 = 0x01: Starts data conversion.
  writeRegister(0x00, 0x01);
}

byte readRegister(byte reg) {
  byte data;
  reg |= 1 << 7;
  digitalWrite(VSPI_SS, LOW);
  SPI.transfer(reg);
  data = SPI.transfer(0);
  digitalWrite(VSPI_SS, HIGH);
  return data;
}

void writeRegister(byte reg, byte data) {
  reg &= ~(1 << 7);
  digitalWrite(VSPI_SS, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(VSPI_SS, HIGH);
}
