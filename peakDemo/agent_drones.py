import random
from peak import Agent, OneShotBehaviour, Message, PeriodicBehaviour, CyclicBehaviour
from spade.template import Template


class agent_drones(Agent):
    class ReceiveMessageFromManager(CyclicBehaviour):
        async def run(self):
            msg = await  self.receive(10)
            if msg:
                print(msg.body)
            else:
                print(self.name, "- Did not received any message after 10 seconds")

    async def setup(self):
        domain = "mas.gecad.isep.ipp.pt"
        template = Template()
        template.to = f"agent_drones@{domain}/ad"
        template.sender = f"agent_manager@{domain}/am"
        template.set_metadata("performative", "inform")
        behavior = self.ReceiveMessageFromManager()
        self.add_behaviour(behavior, template)
