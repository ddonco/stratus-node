#!/usr/bin/env python

import argparse
import bme280
import datetime
import time
from subprocess import PIPE, Popen
from paho.mqtt import client as mqtt_client

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


# Initialise the bme280
bus = SMBus(1)
address = 0x77
bme280.load_calibration_params(bus, address=address)


# Gets the CPU temperature in degrees C
def get_cpu_temperature():
    process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
    output, _error = process.communicate()
    return float(output[output.decode().index('=') + 1:output.decode().rindex("'")])


def connect_mqtt(broker, port, client_id):
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


def publish(client, topic, sensor_id, location, site="home"):
    factor = 1.2  # Smaller numbers adjust temp down, vice versa
    smooth_size = 10  # Dampens jitter due to rapid CPU temp changes
    cpu_temps = []
    compensate = False
    while True:
        data = bme280.sample(bus, address=address)
        temperature = (data.temperature * 9/5) + 32
        if compensate:
            cpu_temp = (get_cpu_temperature() * 9/5) + 32
            cpu_temps.append(cpu_temp)

            if len(cpu_temps) > smooth_size:
                cpu_temps = cpu_temps[1:]

            smoothed_cpu_temp = sum(cpu_temps) / float(len(cpu_temps))
            temperature = temperature - \
                ((smoothed_cpu_temp - temperature) / factor)

        temperature_str = f"{temperature:.2f}"
        pressure = data.pressure
        pressure_str = f"{pressure:.2f}"
        humidity = data.humidity
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
    # MQTT broker constants
    broker = "192.168.40.94"
    port = 1883
    topic = "sensors"
    # username = 'emqx'
    # password = 'public'

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

    client_id = f"{id}-{location}"
    client = connect_mqtt(broker, port, client_id)
    client.loop_start()

    if args['site']:
        site = args['site']
        publish(client, topic, id, location, site)

    publish(client, topic, id, location)


if __name__ == '__main__':
    main()
