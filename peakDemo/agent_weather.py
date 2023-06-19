# agent_manager.py
from aioxmpp import JID
from peak import Agent, CyclicBehaviour, Message
from spade.template import Template
import pandas as pd
import json
class agent_weather(Agent):
    def __init__(self, jid: JID, cid: int = 0, verify_security: bool = False):
        super().__init__(jid, cid, verify_security)
        self.meteorologia = pd.read_csv("dataset-final.csv")
        self.last_index = 0

    class ReceiveMessage(CyclicBehaviour):
        def __init__(self, agent):
            super().__init__()
            self.agent = agent

        async def run(self):
            msg = await self.receive(10)
            if msg:
                print("Weather - ReceiveMessage")
                print(f"Weather - {msg.sender} sent me a message: '{msg.body}")
                split_values = msg.body.split(',')
                fault_sensor = split_values[1].split("-")[1]
                priority = split_values[2].split("-")[1]
                print("fault sensor: ", fault_sensor)
                weather_report = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                weather_report.set_metadata("performative", "inform")
                
                line = self.agent.meteorologia.iloc[self.agent.last_index]
                weather_data = {
                    'DateTime': int(line['hour']),
                    'Generated power': int(line['generated_power']),
                    'TemperatureC': int(line['temperatureC']),
                    'DewpointC': int(line['dewpointC']),
                    'PressurehPa': int(line['pressurehPa']),
                    'WindDirectionDegrees': int(line['wind_direction_degrees']),
                    'WindSpeedKMH': int(line['wind_speed_KMH']),
                    'WindSpeedGustKMH': int(line['wind_speed_gustKMH']),
                    'Humidity': int(line['Humidity']),
                    'HourlyPrecipMM': int(line['hourly_precipMM']),
                    'dailyrainMM': int(line['daily_rainMM']),
                    'SolarRadiationWatts_m2': int(line['solar_radiation_Watts_m2']),
                    'sensor_at_fault': fault_sensor,
                    'priority': priority
                }
                self.agent.last_index += 1
                weather_report.body = json.dumps(weather_data)
                await self.send(weather_report)
                
                # weather_response = await self.receive(5)
                # print(weather_response)
                # await self.agent.stop()
            else:
                print("Weather - Did not received any message after 10 seconds")

        async def on_end(self):
            # stop agent from behaviour
            print("Weather - I'm closing down")
            await self.agent.stop()

    async def setup(self):
        b = self.ReceiveMessage(agent=self)
        template = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template.to = f"agent_weather@{domain}/aw"
        template.sender = f"agent_manager@{domain}/am"
        template.set_metadata("performative", "inform")
        self.add_behaviour(b, template)
