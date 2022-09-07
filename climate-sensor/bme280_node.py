#!/usr/bin/env python

import argparse
import datetime
import time
from bme280 import BME280
from subprocess import PIPE, Popen
from paho.mqtt import client as mqtt_client

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


# MQTT broker constants
broker = "192.168.40.94"
port = 1883
topic = "sensors"
client_id = f"bme280-{'office'}"
# username = 'emqx'
# password = 'public'

# Initialise the bme280
bus = SMBus(1)
bme280 = BME280(i2c_dev=bus, i2c_addr=0x77)


# Gets the CPU temperature in degrees C
def get_cpu_temperature():
    process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
    output, _error = process.communicate()
    return float(output[output.decode().index('=') + 1:output.decode().rindex("'")])


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


def publish(client, sensor_id, location, site="home"):
    factor = 1.2  # Smaller numbers adjust temp down, vice versa
    smooth_size = 10  # Dampens jitter due to rapid CPU temp changes
    cpu_temps = []
    compensate = False
    while True:
        cpu_temp = (get_cpu_temperature() * 9/5) + 32
        cpu_temps.append(cpu_temp)

        if len(cpu_temps) > smooth_size:
            cpu_temps = cpu_temps[1:]

        smoothed_cpu_temp = sum(cpu_temps) / float(len(cpu_temps))
        temperature = (bme280.get_temperature() * 9/5) + 32
        if compensate:
            temperature = temperature - \
                ((smoothed_cpu_temp - temperature) / factor)

        temperature_str = f"{temperature:.2f}"
        pressure = bme280.get_pressure()
        pressure_str = f"{pressure:.2f}"
        humidity = bme280.get_humidity()
        humidity_str = f"{humidity:.2f}"

        msg = f"climate_sensor,sensor_id={sensor_id},site={site},location={location} temperature={temperature_str},pressure={pressure_str},humidity={humidity_str}"
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

    parser = argparse.ArgumentParser(
        description='Stratus environmental node using a bme280 sensor to measure ambient temperature and pressure.')
    parser.add_argument('-l', '--location',
                        type=str,
                        help='location of the sensor',
                        required=True)
    parser.add_argument('-i', '--id',
                        type=str,
                        help='id of the sensor',
                        required=True)
    parser.add_argument('-s', '--site',
                        type=str,
                        help='site of the sensor')
    args = vars(parser.parse_args())
    location = args['location']
    id = args['id']
    if args['site']:
        site = args['site']
        publish(client, id, location, site)

    publish(client, id, location)


if __name__ == '__main__':
    main()
