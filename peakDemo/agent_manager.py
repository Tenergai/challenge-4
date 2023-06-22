# agent_manager.py
from peak import Agent, OneShotBehaviour, CyclicBehaviour, PeriodicBehaviour, Message
from spade.template import Template
import json
import random


class agent_manager(Agent):
    weather_data = dict()
    sensor_data = dict()

    @staticmethod
    def calculate_running_average(new_value, current_average, count):
        return ((current_average * count) + new_value) / (count + 1)

    @staticmethod
    def is_less_than_10_percent(value, average):
        threshold = 0.1 * average
        return value < threshold

    @staticmethod
    def get_priority_from_percentage_difference(percentage_difference):
        if percentage_difference < 10:
            return 0
        else:
            return int(percentage_difference // 10)

    @staticmethod
    def compare_average_against_control_average(value, average):
        priority = 0
        percentage_difference = abs((value - average) / average) * 100
        print("Value: ", value, " Average: ", average, " Difference: ", percentage_difference)
        if value < average:
            print(f"The value is {percentage_difference:.2f}% less than the average.")
            priority = agent_manager.get_priority_from_percentage_difference(percentage_difference)
            return True, False, percentage_difference, priority
        elif value > average:
            print(f"The value is {percentage_difference:.2f}% greater than the average.")
            # If value is greater than average, we're going to check control sensor
            priority = agent_manager.get_priority_from_percentage_difference(percentage_difference)
            return True, True, percentage_difference, priority
        else:
            print("The value is equal to the average.")

        return False, percentage_difference, priority

    @staticmethod
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

    @staticmethod
    def treat_receive_message_from_sensor(msg):
        split_values = msg.body.split('-')
        sensor_name = split_values[0]
        sensor_value = float(split_values[1])
        average_key = sensor_name + "_average"
        return sensor_name, sensor_value, average_key

    @staticmethod
    def prepare_message_to_weather_agent(sensor_name, agent_domain, priority):
        print(f"{sensor_name} - producing less than expected!")
        weather_request = Message(to=f"agent_weather@{agent_domain}/aw")
        weather_request.body = f"AgentManager-Request,SensorName-{sensor_name},Priority-{priority}"
        weather_request.set_metadata("performative", "inform")
        return weather_request

    @staticmethod
    async def handle_sensor_message(self, msg, sensor_name):
        print(f"{sensor_name} - {msg.sender} sent me a message: '{msg.body}'")
        sensor_name, sensor_value, average_key = agent_manager.treat_receive_message_from_sensor(msg)
        agent_manager.update_values(sensor_name, sensor_value)
        if not str(msg.sender).strip() == "agent_control@mas.gecad.isep.ipp.pt/ac":
            # Grab the current sensor average, and check percentage difference against agent control current average
            try:
                perform_operation, check_control, percentage_difference, priority = agent_manager.compare_average_against_control_average(
                    agent_manager.sensor_data[average_key], agent_manager.sensor_data['SensorC_average'])
                if perform_operation:
                    if check_control:
                        sensor_name = "SensorC"
                    weather_request = agent_manager.prepare_message_to_weather_agent(sensor_name, self.agent.jid.domain,
                                                                                     priority)

                    await self.send(weather_request)
            except KeyError:
                print("Sensor Control isn't filled yet")
        else:
            print("Message was sent by control: ", msg.sender)

    class ReceiveMessageSensor(CyclicBehaviour):
        def __init__(self, name):
            super().__init__()
            self.name = name

        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, self.name)
            else:
                print(self.name, "- Did not received any message after 10 seconds")

        async def on_end(self):
            # stop agent from behaviour
            print(self.name, "- I'm closing down, my dictionary:")
            for key, value in agent_manager.sensor_data.items():
                print(self.name, "-", key, ":", value)
            await self.agent.stop()

    class ReceiveMessageSensorControl(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                await agent_manager.handle_sensor_message(self, msg, "SensorControl")
            else:
                print("SensorControl - Did not received any message after 10 seconds")

    class ReceiveMessageWeatherAgent(CyclicBehaviour):

        def verify_weather_is_good(self, weather_data):
            temperature_c = float(weather_data["TemperatureC"])
            solar_radiation_m2 = float(weather_data["SolarRadiationWatts_m2"])
            hour = int(weather_data["DateTime"])
            print(f"Weather - Temperature: {temperature_c} Solar Radiation: {solar_radiation_m2}")

            if temperature_c > 25.0:
                # https://www.eco-greenenergy.com/temperature-coefficient-of-solar-pv-module/#:~:text=Most%20solar%20PV%20modules%20have,%2D0.5%25%20%2F%20%C2%B0C.
                print(
                    f"Weather - Weather is currently {temperature_c}C, It's {temperature_c - 25.0} above 25 degrees, which will lower eneryg production according to the PV's coefficient")
                return False
            elif hour >= 21 or hour < 7:
                print(f"Weather - Current hour is {hour}. Due to nightime, EV generation has reduced.")
                return False
            elif solar_radiation_m2 < 1:
                print(f"Weather - Solar radiation at {solar_radiation_m2}w/m2. Impacting EV generation.")
                return False
            else:
                return True

        async def drone_management(self, weather_data, sensor_at_fault, priority):
            if self.verify_weather_is_good(weather_data):
                print(f"Weather - Weather is good, I'm sending a message to drone agent to check {sensor_at_fault}.")
                drone_message = Message(to=f"agent_drones@{self.agent.jid.domain}/ad")
                drone_message.set_metadata("performative", "inform")
                drone_message.body = f"Check-{sensor_at_fault},Priority-{priority}"
                await self.send(drone_message)
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
                priority = data_dict["priority"]
                await self.drone_management(data_dict, sensor_at_fault, priority)
            else:
                print("Weather - Did not receive any message from Agent Weather after 10 seconds")

    class PrintDictionaryBehaviour(PeriodicBehaviour):
        async def run(self):
            print(agent_manager.sensor_data)

    def define_behaviour(self, behaviour, agent_name, slash):
        template = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template.to = f"agent_manager@{domain}/am"
        template.sender = f"{agent_name}@{domain}/{slash}"
        template.set_metadata("performative", "inform")
        self.add_behaviour(behaviour, template)

    async def setup(self):
        self.define_behaviour(self.ReceiveMessageSensor(name="Sensor1"), "agent_sensor1", "ag1")
        self.define_behaviour(self.ReceiveMessageSensor(name="Sensor2"), "agent_sensor2", "ag2")
        self.define_behaviour(self.ReceiveMessageSensor(name="Sensor3"), "agent_sensor3", "ag3")
        self.define_behaviour(self.ReceiveMessageSensor(name="Sensor4"), "agent_sensor4", "ag4")
        self.define_behaviour(self.ReceiveMessageSensorControl(), "agent_control", "ac")
        self.define_behaviour(self.ReceiveMessageWeatherAgent(), "agent_weather", "aw")

        self.add_behaviour(self.PrintDictionaryBehaviour(period=5))
