# agent_manager.py
from peak import Agent, OneShotBehaviour, CyclicBehaviour, Message
from spade.template import Template
import pandas as pd
import json
class agent_weather(Agent):
    class ReceiveMessage(CyclicBehaviour):
        async def on_start(self):
            #Ler ficheiro metromaria.csv
            self.meteorologia = pd.read_csv("dataset-final.csv")
            self.last_index = 0
            

        async def run(self):
            msg = await self.receive(10)
            if msg:
                print("Weather - ReceiveMessage")
                print(f"Weather - {msg.sender} sent me a message: '{msg.body}")
                split_values = msg.body.split(',')
                fault_sensor = split_values[1].split("-")[1]
                print("fault sensor: ", fault_sensor)
                weather_report = Message(to=f"agent_manager@{self.agent.jid.domain}/am")
                weather_report.set_metadata("performative", "inform")
                
                line = self.meteorologia.iloc[self.last_index]
                weather_data = {
                    'DateTime': line['hour'],
                    'Generated power': line['generated_power'],
                    'TemperatureC': line['temperatureC'],
                    'DewpointC': line['dewpointC'],
                    'PressurehPa': line['pressurehPa'],
                    'WindDirectionDegrees': line['wind_direction_degrees'],
                    'WindSpeedKMH': line['wind_speed_KMH'],
                    'WindSpeedGustKMH': line['wind_speed_gustKMH'],
                    'Humidity': line['Humidity'],
                    'HourlyPrecipMM': line['hourly_precipMM'],
                    'dailyrainMM': line['daily_rainMM'],
                    'SolarRadiationWatts_m2': line['solar_radiation_Watts_m2']
                }
                self.last_index += 1
                
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
        b = self.ReceiveMessage()
        template = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template.to = f"agent_weather@{domain}/aw"
        template.sender = f"agent_manager@{domain}/am"
        template.set_metadata("performative", "inform")
        self.add_behaviour(b, template)
