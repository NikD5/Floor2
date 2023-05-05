from mesa import Agent
from mesa.space import ContinuousSpace
#  from Agents import PhysicalAgent


class MySensor(Agent):
    def __init__(self, unique_id, model, pos, sensor_radius):
        super().__init__(unique_id, model)
        self.speed_sensor_radius = sensor_radius
        self.speed_data = []
        self.pos = pos
        self.speed_agent = 0
        self.is_moving = False

    def sense_speed(self):
        # agents = mesa.space.ContinuousSpace.get_neighbors(self.pos, self.speed_sensor_radius, True)
        agents = self.model.space.get_neighbors(self.pos, self.speed_sensor_radius, False)
        for agent in agents:
            a = agent.speed_agent()
            self.speed_data.append(a)


    def speed_value(self):
        return self.speed_data

    def step(self):
        self.sense_speed()
        print(self.speed_value())
        print("there should be speed above")

    def get_points_to_show(self):
        return {'agent': self.pos, 'final_target': self.pos,
                'next_target': self.pos}

    def speed_agent(self):
        SP_A = 0
        return SP_A
