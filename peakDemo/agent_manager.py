# agent_manager.py
from peak import Agent, OneShotBehaviour, CyclicBehaviour, PeriodicBehaviour, Message
from spade.template import Template
import json
import random


class agent_manager(Agent):
    # test_var = "Hello there"
    weather_data = dict()
    sensor_data = dict()

    def calculate_running_average(new_value, current_average, count):
        return ((current_average * count) + new_value) / (count + 1)

    def is_less_than_10_percent(value, average):
        threshold = 0.1 * average
        return value < threshold

    def update_values(sensor_name, sensor_value):
        count_key = sensor_name + "_count"
        average_key = sensor_name + "_average"

        if sensor_name not in agent_manager.sensor_data:
            agent_manager.sensor_data[sensor_name] = [sensor_value]

            agent_manager.sensor_data[count_key] = 1
            agent_manager.sensor_data[average_key] = sensor_value
        else:
            if agent_manager.sensor_data[count_key] >= 60:
                agent_manager.sensor_data[sensor_name][:-1] = agent_manager.sensor_data[sensor_name][1:]
                agent_manager.sensor_data[sensor_name].append(sensor_value)

            else:
                agent_manager.sensor_data[sensor_name].append(sensor_value)

            agent_manager.sensor_data[count_key] += 1
            agent_manager.sensor_data[average_key] = agent_manager.calculate_running_average(sensor_value,
                                                                                             agent_manager.sensor_data[
                                                                                                 average_key],
                                                                                             agent_manager.sensor_data[
                                                                                                 count_key])

    def treat_receive_message_from_sensor(msg):
        split_values = msg.body.split('-')
        sensor_name = split_values[0]
        sensor_value = float(split_values[1])
        average_key = sensor_name + "_average"
        return sensor_name, sensor_value, average_key

    def prepare_message_to_weather_agent(sensor_name, agent_domain):
        print(f"{sensor_name} - producing less than expected!")
        weather_request = Message(to=f"agent_weather@{agent_domain}/aw")
        weather_request.body = f"AgentManager-Request,SensorName-{sensor_name}"
        weather_request.set_metadata("performative", "inform")
        return weather_request

    async def handle_sensor_message(self, msg, sensor_name):
        print(f"{sensor_name} - {msg.sender} sent me a message: '{msg.body}'")
        sensor_name, sensor_value, average_key = agent_manager.treat_receive_message_from_sensor(msg)
        agent_manager.update_values(sensor_name, sensor_value)
        # TODO
        # Grab the current sensor average, and check if it's less than 10% of agent control current average
        if agent_manager.is_less_than_10_percent(sensor_value, agent_manager.sensor_data[average_key]) or True:
            weather_request = agent_manager.prepare_message_to_weather_agent(sensor_name, self.agent.jid.domain)
            await self.send(weather_request)

    class ReceiveMessageSensor1(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "Sensor1")
            else:
                print("Sensor1 - Did not received any message after 10 seconds")

        async def on_end(self):
            # stop agent from behaviour
            print("Sensor1 - I'm closing down, my dictionary:")
            for key, value in agent_manager.sensor_data.items():
                print("Sensor1 -", key, ":", value)
            await self.agent.stop()

    class ReceiveMessageSensor2(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "Sensor2")
            else:
                print("Sensor2 - Did not received any message after 10 seconds")

    class ReceiveMessageSensor3(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "Sensor3")
            else:
                print("Sensor3 - Did not received any message after 10 seconds")

    class ReceiveMessageSensor4(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "Sensor4")
            else:
                print("Sensor4 - Did not received any message after 10 seconds")

    class ReceiveMessageSensorControl(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "SensorControl")
            else:
                print("SensorControl - Did not received any message after 10 seconds")

    class ReceiveMessageWeatherAgent(CyclicBehaviour):
        # TODO
        # Finish this method
        def verify_weather_is_good(self, weather_data):
            temperature_c = float(weather_data["TemperatureC"])
            solar_radiation_m2 = float(weather_data["SolarRadiationWatts_m2"])
            print(f"Weather - Temperature: {temperature_c} Solar Radiation: {solar_radiation_m2}")

            return random.random() < 0.75

        # TODO
        # Finish this method
        def drone_management(self, weather_data, sensor_at_fault):
            if self.verify_weather_is_good(weather_data):
                print(f"Weather - Weather is good, I'm sending a message to drone agent to check {sensor_at_fault}.")
                # drone_message = Message(to=f"agent_drones@{self.agent.jid.domain}/ad")
                # drone_message.set_metadata("performative", "inform")
                # drone_message.body = f"Check-{sensor_at_fault}"
                # await self.send()
                return True
            else:
                print(f"Weather - Attributing bad performance of {sensor_at_fault} to bad weather conditions")
                return False

        async def run(self):
            msg = await self.receive(10)
            if msg:
                print(f"Weather - {msg.sender} sent me a message: '{msg.body}'")
                data_dict = json.loads(msg.body)
                agent_manager.weather_data = data_dict
                # print(data_dict)
                sensor_at_fault = data_dict["sensor_at_fault"]
                self.drone_management(data_dict, sensor_at_fault)
            else:
                print("Weather - Did not receive any message from Agent Weather after 10 seconds")

    class PrintDictionaryBehaviour(PeriodicBehaviour):
        async def run(self):
            print(agent_manager.sensor_data)

    async def setup(self):
        receiveMessageSensor1 = self.ReceiveMessageSensor1()
        template = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template.to = f"agent_manager@{domain}/am"
        template.sender = f"agent_sensor1@{domain}/ag1"
        template.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensor1, template)
        receiveMessageSensor2 = self.ReceiveMessageSensor2()
        template2 = Template()
        template2.to = f"agent_manager@{domain}/am"
        template2.sender = f"agent_sensor2@{domain}/ag2"
        template2.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensor2, template2)
        receiveMessageSensor3 = self.ReceiveMessageSensor3()
        template3 = Template()
        template3.to = f"agent_manager@{domain}/am"
        template3.sender = f"agent_sensor3@{domain}/ag3"
        template3.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensor3, template3)
        receiveMessageSensor4 = self.ReceiveMessageSensor4()
        template4 = Template()
        template4.to = f"agent_manager@{domain}/am"
        template4.sender = f"agent_sensor4@{domain}/ag4"
        template4.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensor4, template4)
        receiveMessageSensorControl = self.ReceiveMessageSensorControl()
        templateC = Template()
        templateC.to = f"agent_manager@{domain}/am"
        templateC.sender = f"agent_control@{domain}/ac"
        templateC.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensorControl, templateC)
        receiveMessageWeatherAgent = self.ReceiveMessageWeatherAgent()
        template_w = Template()
        template_w.to = f"agent_manager@{domain}/am"
        template_w.sender = f"agent_weather@{domain}/aw"
        template_w.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageWeatherAgent, template_w)
        self.add_behaviour(self.PrintDictionaryBehaviour(period=5))
