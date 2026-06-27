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
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime, timezone

# MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USER = "USER_HERE"
MQTT_PASSWORD = "PASWORD_HERE"

MQTT_TOPIC = "iotholter/device/+/ecg"
# InfluxDB
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "RguJK5Z-EBVanFFl2_8sIvZ5on57skNov0jZMK1_Oa4lDYh9ayxsm0MFpzDHHuFUQDQ_hHRhZg-7_o3Ba2lnjw=="
INFLUX_ORG = "TecNM Celaya"
INFLUX_BUCKET = "holter"

#Statistics
last_seq = {}
lost_packets = {}
restart_count = {}
received_packets = {}

influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)
write_api = influx_client.write_api(
    write_options=SYNCHRONOUS
)

def on_connect(client, userdata, flags, rc, properties=None):
    print("MQTT conectado")
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(
            msg.payload.decode()
        )
        topic_parts = msg.topic.split("/")
        device = topic_parts[2]
        fs = payload["fs"]
        t0 = payload["t0"]
        seq = payload["seq"]
        #RECIEVED
        if device not in received_packets:
            received_packets[device] = 0
        received_packets[device] += 1
        if received_packets[device] % 100 == 0:
            loss_percent = (100.0 *lost_packets[device] /received_packets[device])
            stat_point = (Point("mqtt_stats").tag("device", device)
                                             .field("received_packets",received_packets[device])
                                             .field("lost_total",lost_packets[device])
                                             .field("restarts",restart_count[device])
                                             .field("loss_percent",loss_percent))
            write_api.write(bucket=INFLUX_BUCKET,record=stat_point)
        #LOST
       if device not in lost_packets:
            lost_packets[device] = 0
        if device not in restart_count:
            restart_count[device] = 0
        if device in last_seq:
            expected = last_seq[device] + 1
            if seq < last_seq[device]: #por reinicio
                print(f"INFO Device={device} "f"Reinicio detectado "f"Seq={seq}")
                restart_count[device] += 1
                #received_packets[device] = 0
                #lost_packets[device] = 0
            elif seq != expected: #por perdida
                lost = seq - expected
                lost_packets[device] += lost
                print(f"WARNING Device{device} "f"Esperando{expected} "f"Recibido{seq} "f"Perdidios={lost} "f"Total={lost_packets[device]}")
        last_seq[device]=seq

        samples = payload["samples"]
        points = []
        sample_period_ms = 1000.0 / fs

        for i, sample in enumerate(samples):

            ch1, ch2, ch3 = sample
            timestamp_ms = (t0 + i * sample_period_ms)
            timestamp = datetime.fromtimestamp(timestamp_ms / 1000.0,tz=timezone.utc)

            point = (
                Point("ecg")
                .tag("device", device)
                .field("ch1", int(ch1))
                .field("ch2", int(ch2))
                .field("ch3", int(ch3))
                .time(timestamp)
            )

            points.append(point)

        write_api.write(bucket=INFLUX_BUCKET,record=points)
        print(f"Device={device} "f"Seq={seq} "f"Muestras={len(samples)}")

    except Exception as e:
        print("Error:", e)

client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)

client.username_pw_set(
    MQTT_USER,
    MQTT_PASSWORD
)

client.on_connect = on_connect
client.on_message = on_message

client.connect(
    MQTT_BROKER,
    MQTT_PORT,
    60
)

client.loop_forever()
