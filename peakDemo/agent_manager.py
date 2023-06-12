# agent_manager.py
from peak import Agent, OneShotBehaviour, CyclicBehaviour, Message
from spade.template import Template


class agent_manager(Agent):
    # test_var = "Hello there"
    weather_data = dict()
    sensor_data = dict()
    counter = 0

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
        print(f"{sensor_name} is producing less than expected!")
        print("Retrieving weather data via request")
        weather_request = Message(to=f"agent_weather@{agent_domain}/aw")
        weather_request.body = "AgentManager-Request"
        weather_request.set_metadata("performative", "inform")
        return weather_request

    class ReceiveMessageSensor1(CyclicBehaviour):
        async def run(self):
            agent_manager.counter += 1

            msg = await self.receive(10)
            if agent_manager.counter > 10:
                for key, value in agent_manager.sensor_data.items():
                    print(key, ":", value)
            if msg:
                print("ReceiveMessage")
                print(f"Manager - {msg.sender} sent me a message: '{msg.body}'")
                sensor_name, sensor_value, average_key = agent_manager.treat_receive_message_from_sensor(msg)
                agent_manager.update_values(sensor_name, sensor_value)

                if agent_manager.is_less_than_10_percent(sensor_value, agent_manager.sensor_data[average_key]):
                    weather_request = agent_manager.prepare_message_to_weather_agent(sensor_name, self.agent.jid.domain)
                    await self.send(weather_request)
            else:
                print("Did not received any message after 10 seconds")

        async def on_end(self):
            # stop agent from behaviour
            print("Manager - I'm closing down, my dictionary:")
            for key, value in agent_manager.sensor_data.items():
                print(key, ":", value)
            await self.agent.stop()

    class ReceiveMessageSensor2(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                print("ReceiveMessage")
                print(f"Manager - {msg.sender} sent me a message: '{msg.body}'")
                sensor_name, sensor_value, average_key = agent_manager.treat_receive_message_from_sensor(msg)
                agent_manager.update_values(sensor_name, sensor_value)

                if agent_manager.is_less_than_10_percent(sensor_value, agent_manager.sensor_data[average_key]):
                    weather_request = agent_manager.prepare_message_to_weather_agent(sensor_name, self.agent.jid.domain)
                    await self.send(weather_request)
            else:
                print("Did not received any message after 10 seconds")

    class ReceiveMessageWeatherAgent(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(10)
            if msg:
                print(f"Manager - Received current weather status from: {msg.sender}")
                print(f"{msg.body}")

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
        template.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageSensor2, template2)
        receiveMessageWeatherAgent = self.ReceiveMessageWeatherAgent()
        template_w = Template()
        domain = "mas.gecad.isep.ipp.pt"
        template_w.to = f"agent_manager@{domain}/am"
        template_w.sender = f"agent_weather@{domain}/aw"
        template_w.set_metadata("performative", "inform")
        self.add_behaviour(receiveMessageWeatherAgent, template_w)
