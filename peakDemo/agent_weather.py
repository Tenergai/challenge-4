# agent_manager.py
from peak import Agent, OneShotBehaviour, CyclicBehaviour, Message
from spade.template import Template
import json


class agent_weather(Agent):
    weather_data = {
        "DateTime": "00:15:00",
        "Generated power": "0.0",
        "TemperatureC": "11.0",
        "PressurehPa": "1021.0",
        "WindDirectionDegrees": "128.33333333333334",
        "WindSpeedKMH": "8.666666666666666",
        "WindSpeedGustKMH": "17.0",
        "Humidity": "83.0",
        "HourlyPrecipMM": "0.0",
        "dailyrainMM": "0.0",
        "SolarRadiationWatts_m2": "0.0"
    }

    class ReceiveMessage(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                print("ReceiveMessage")
                print(f"Weather - {msg.sender} sent me a message: '{msg.body}")
                split_values = msg.body.split(',')
                fault_sensor = split_values[1].split("-")[1]
                print("fault sensor: ", fault_sensor)
                weather_report = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                weather_report.set_metadata("performative", "inform")
                current_weather_data = agent_weather.weather_data
                current_weather_data["sensor_at_fault"] = fault_sensor
                print(current_weather_data)
                weather_report.body = json.dumps(current_weather_data)
                await self.send(weather_report)
                # Request and response?
                # weather_response = await self.receive(5)
                # print(weather_response)
            # await self.agent.stop()
            else:
                print("Did not received any message after 10 seconds")

        async def on_end(self):
            # stop agent from behaviour
            print("Weather - I'm closing down")
            await self.agent.stop()

    async def setup(self):
        b = self.ReceiveMessage()
        template = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template.to = f"agent_weather@{domain}/aw"
        template.sender = f"agent_manager@{domain}/am"
        template.set_metadata("performative", "inform")
        self.add_behaviour(b, template)
