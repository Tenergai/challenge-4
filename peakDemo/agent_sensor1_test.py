import random
from peak import Agent, OneShotBehaviour, Message, PeriodicBehaviour


class agent_sensor1_test(Agent):
    class SendSensorData(PeriodicBehaviour):
        async def run(self):
            random_float = random.uniform(1, 10)
            msg = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
            msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
            msg.body = "Sensor1-%f" % random_float
            # print("My own id",self.agent.jid)
            await self.send(msg)
            # await self.agent.stop()

    async def setup(self):
        period = 1
        behavior = self.SendSensorData(period=period)
        self.add_behaviour(behavior)