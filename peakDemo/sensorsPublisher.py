# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client
import concurrent.futures
from threading import Thread

broker = 'broker.emqx.io'
port = 1883
# Generate a Client ID with the publish prefix.
client_id_base = 'publish-'
# username = 'emqx'
# password = 'public'

class MQTTPublisher:
    def __init__(self, client_id, topic):
        self.client_id = client_id
        self.topic = topic

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(self.client_id)
        # client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client

    def publish(self, client):
        #comentar esta linha
        msg_count = 1
        while True:
            time.sleep(1)
            report = str(random.randint(1000,5000))
            msg = f"messages: {report}"
            result = client.publish(self.topic, report)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send `{msg}` to topic `{self.topic}` using client ID: {self.client_id}")
            else:
                print(f"Failed to send message to topic {self.topic} using client ID: {self.client_id}")
            #comentar esta zona
            msg_count += 1
            if msg_count > 5:
                break

    def run(self):
        client = self.connect_mqtt()
        client.loop_start()
        self.publish(client)
        client.loop_stop()


if __name__ == '__main__':
    publishers = []
    topics = ["report/agent1", "report/agent2", "report/agent3", "report/agent4", "report/agentC"]
    for topic in topics:
        client_id = client_id_base + str(random.randint(0, 1000))
        publisher = MQTTPublisher(client_id, topic)
        publishers.append(publisher)

    threads = []
    for publisher in publishers:
        thread = Thread(target=publisher.run)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()