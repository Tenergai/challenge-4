import random
from peak import Agent, OneShotBehaviour, Message, PeriodicBehaviour
from paho.mqtt import client as mqtt_client
import time
BROKER = 'broker.emqx.io'
PORT = 1883
TOPIC = "report/agentC"
# Generate a Client ID with the subscribe prefix.
CLIEND_ID = f'subscribe-agentc'
# username = 'emqx'
# password = 'public'
# CLIENT=None
last_reading=None

def connect_mqtt(self) -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(CLIEND_ID)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(BROKER, PORT)
    return client


def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    global last_reading
    last_reading = str(msg.payload)


class agent_sensor_c(Agent):

    class SendSensorData(PeriodicBehaviour):                
           
        async def run(self):
            while self.client.loop() == 0:
                time.sleep(1) 
                if last_reading is not None:
                    # random_float = random.uniform(1, 10)
                    msg = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                    msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
                    msg.body = "SensorC-%f" % last_reading
                    # print("My own id",self.agent.jid)
                    await self.send(msg)
                    # await self.agent.stop()

    async def setup(self):
        period = 1
        self.client = connect_mqtt()
        self.client.subscribe(TOPIC)
        self.client.on_message = on_message
        behavior = self.SendSensorData(period=period)
        self.add_behaviour(behavior)




