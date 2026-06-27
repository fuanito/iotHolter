 # Copyright (c) 2026 Ruth Escobedo-Carranza et al.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in all
 # copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 # SOFTWARE.

import json
import time
import paho.mqtt.client as mqtt
import numpy as np

# CONFIGURACIÓN MQTT
MQTT_BROKER = "mqtt.iotholter.com"
MQTT_PORT = 8883
MQTT_USER = "USER_HERE"
MQTT_PASSWORD = "PASSWORD_HERE"
TOPIC = "iotholter/device/004/ecg"
# CONFIGURACIÓN ECG
FS = 250                 # Hz
BLOCK_SIZE = 50          # muestras por paquete
INTERVAL = BLOCK_SIZE / FS   # 0.2 segundos
# MQTT CLIENT
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(MQTT_USER,MQTT_PASSWORD)
client.tls_set()
result = client.connect(MQTT_BROKER,MQTT_PORT,60)
print (result)
client.loop_start()
print("Conectado a MQTT")
# HOLTER desde numpy
data = np.load("ecg.npy")

while(True):
    seq = 0
    for d in range(1000):
        t0_ms = int(time.time() * 1000)
        samples = []
        for i in range(BLOCK_SIZE):
            ch1, ch2, ch3 = data[50*d+i]
            samples.append([int(ch1),int(ch2),int(ch3)])
        payload = {
            "seq": seq,
            "fs": FS,
            "t0": t0_ms,
            "samples": samples
        }
        client.publish(TOPIC,json.dumps(payload),qos=1)
        print(f"Enviado paquete {seq} "f"({len(samples)} muestras)")
        seq += 1
        time.sleep(INTERVAL)
    #for i in range(60):
    #    time.sleep(1)
