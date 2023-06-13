import random
from peak import Agent, OneShotBehaviour, Message, PeriodicBehaviour
import sensorSubscriber
from paho.mqtt import client as mqtt_client
import time
BROKER = 'broker.emqx.io'
PORT = 1883
TOPIC = "report/agent1"
CLIEND_ID = f'subscribe-agent1'
last_reading=None


def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    global last_reading
    last_reading = str(msg.payload)

class agent_sensor1(Agent):
    class SendSensorData(PeriodicBehaviour):
        async def run(self):
            self.agent.client.loop()
            time.sleep(1) 
            if last_reading is not None:
                msg = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
                msg.body = "Sensor1-"+str(last_reading)
                await self.send(msg)

    async def setup(self):
        period = 1
        behavior = self.SendSensorData(period=period)
        self.add_behaviour(behavior)
        self.client = mqtt_client.Client()
        self.client.on_connect = sensorSubscriber.on_connect
        self.client.on_message = on_message
        self.client.connect(BROKER, PORT)
        self.client.subscribe(TOPIC)
        
        