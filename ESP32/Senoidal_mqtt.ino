/*
 * IoTHolter ESP32 Simulator
 *
 * Copyright (c) 2026 Ruth Escobedo-Carranza et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include <WiFiClientSecure.h>
#include <math.h>

const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";
const char* mqtt_server = "mqtt.iotholter.com";//broker
const int mqtt_port = 8883;
const char* mqtt_user = "USER_HERE";
const char* mqtt_password = "PASSWORD_HERE";
const char* topic= "iotholter/device/003/ecg";
WiFiClientSecure espClient;
PubSubClient client(espClient);
const int BLOCK_SIZE = 50;
int32_t bufferECG[BLOCK_SIZE][3];
uint16_t sampleIndex = 0;
uint32_t seq = 0;
uint64_t t0 = 0;
float phase = 0.0;
int32_t canal1, canal2, canal3;

uint64_t epochMillis(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((uint64_t)tv.tv_sec * 1000ULL) +
           (tv.tv_usec / 1000ULL);
}

void enviarBloqueMQTT(){
    StaticJsonDocument<8192> doc;
    doc["seq"] = seq;
    doc["fs"] = 250;
    doc["t0"] = t0;
    JsonArray samples = doc.createNestedArray("samples");
    for(int i=0; i<BLOCK_SIZE; i++){
        JsonArray fila = samples.createNestedArray();
        fila.add(bufferECG[i][0]);
        fila.add(bufferECG[i][1]);
        fila.add(bufferECG[i][2]);
    }
    String payload;
    serializeJson(doc, payload);
    bool ok = client.publish(topic, payload.c_str()); //topico
}

void setup_wifi() {
  Serial.print("setup");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado");
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(topic, mqtt_user, mqtt_password)) {
      Serial.println("MQTT conectado");
    } 
    else {
      Serial.print("Error MQTT: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.print("inicio");
  setup_wifi();
  espClient.setInsecure();
  configTime(0, 0, "pool.ntp.org");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)){
      Serial.println("Esperando NTP...");
      delay(1000);
  }
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(4096);
}

void loop() {
  if (!client.connected())
    reconnect();
  client.loop();
  if(sampleIndex == 0){
      t0 = epochMillis();
  }
  canal1=(int)(1000.0 * sin(phase));
  canal2=(int)(800.0 * sin(phase + 0.5));
  canal3=(int)(600.0 * sin(phase + 1.0));
  bufferECG[sampleIndex][0] = canal1;
  bufferECG[sampleIndex][1] = canal2;
  bufferECG[sampleIndex][2] = canal3;
  sampleIndex++;
  phase += 0.05;
  if (phase > 2.0 * PI){
    phase -= 2.0 * PI;
  }
  if(sampleIndex >= BLOCK_SIZE){
      enviarBloqueMQTT();
      sampleIndex = 0;
      seq++;
  }
  delay(4); 
}
