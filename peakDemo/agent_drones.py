from aioxmpp import JID
from peak import Agent, CyclicBehaviour
from spade.template import Template

# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client
import concurrent.futures
from threading import Thread

# Generate a Client ID with the publish prefix.
client_id_base = 'publish-'
# username = 'emqx'
# password = 'public'
def connect_mqtt(client_id):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id)
        # client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect('broker.emqx.io', 1883)
        return client

class MQTTPublisher:
    def __init__(self, client_id, topic):
        self.client_id = client_id
        self.topic = topic

    def publish(self, client, msg):
        while True:
            result = client.publish(self.topic, msg)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send `{msg}` to topic `{self.topic}` using client ID: {self.client_id}")
            else:
                print(f"Failed to send message to topic {self.topic} using client ID: {self.client_id}")
            #comentar esta zona
            # msg_count += 1
            # if msg_count > 5:
            #     break

    def run(self, client, msg):
        self.publish(client, msg)
        client.loop_stop()

class agent_drones(Agent):    
    client_id = client_id_base + str(random.randint(0, 1000))
    client = connect_mqtt(client_id)
        
    class ReceiveMessageFromManager(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                print(msg.body)
                sensor = msg.body.split(',')[0].split('-')[1]
                print(sensor)
                publisher = MQTTPublisher(agent_drones.client_id, "anomaly/drone")
                
                if sensor == "Sensor1":
                    publisher.run(agent_drones.client,"section1_1")
                elif sensor == "Sensor3":
                    publisher.run(agent_drones.client,"section1_2")
                elif sensor == "SensorC":
                    publisher.run(agent_drones.client,"section2_1")
                elif sensor == "Sensor2":
                    publisher.run(agent_drones.client,"section2_2")
                
                sensor = None
            else:
                print("Drones", "- Did not received any message after 10 seconds")

    async def setup(self):
        domain = "mas.gecad.isep.ipp.pt"
        template = Template()
        template.to = f"agent_drones@{domain}/ad"
        template.sender = f"agent_manager@{domain}/am"
        template.set_metadata("performative", "inform")
        behavior = self.ReceiveMessageFromManager()
        self.add_behaviour(behavior, template)
        
        agent_drones.client.loop_start()
