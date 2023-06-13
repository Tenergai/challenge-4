import random
from peak import Agent, OneShotBehaviour, Message, PeriodicBehaviour

from paho.mqtt import client as mqtt_client
import time
import sensorSubscriber
BROKER = 'broker.emqx.io'
PORT = 1883
TOPIC = "report/agent2"
CLIEND_ID = f'subscribe-agent2'
last_reading=None

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    global last_reading
    last_reading = str(msg.payload.decode("utf-8"))

class agent_sensor2(Agent):
    client=None
    class SendSensorData(PeriodicBehaviour):
        async def run(self):
            agent_sensor2.client.loop()
            if last_reading is not None:
                msg = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
                msg.body = "Sensor2-"+str(last_reading)
                await self.send(msg)

    async def setup(self):
        period = 1
        agent_sensor2.client = mqtt_client.Client()
        agent_sensor2.client.on_connect = sensorSubscriber.on_connect
        agent_sensor2.client.on_message = on_message
        agent_sensor2.client.connect(BROKER, PORT)
        agent_sensor2.client.subscribe(TOPIC)
        behavior = self.SendSensorData(period=period)
        self.add_behaviour(behavior)
