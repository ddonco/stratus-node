#!/usr/bin/env python

import time
import Adafruit_BMP.BMP085 as BMP085
from paho.mqtt import client as mqtt_client


# MQTT broker constants
broker = "192.168.40.94"
port = 1883
topic = "sensors"
client_id = f"bmp180-{'living_room'}"
# username = 'emqx'
# password = 'public'

# Initialise the BMP180
bmp180 = BMP085.BMP085()


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("connected to mqtt broker")
        else:
            print("failed to connect to broker, return code: %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    factor = 1.2  # Smaller numbers adjust temp down, vice versa
    smooth_size = 10  # Dampens jitter due to rapid CPU temp changes
    compensate = False
    while True:
        temperature = (bmp180.read_temperature() * 9/5) + 32
        temperature_str = f"{temperature:.2f}"
        pressure = bmp180.read_pressure() / 100.0
        pressure_str = f"{pressure:.2f}"

        msg = f"climate_sensor,sensor_id=rpi2-bmp180,site=home,location=living_room temperature={temperature_str},pressure={pressure_str}"
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"publish success - topic: `{topic}` message: `{msg}`")
        else:
            print(f"publish failed - topic: {topic}")

        time.sleep(60)


def main():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    main()
