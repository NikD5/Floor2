import math

import matplotlib.pyplot as plt
from matplotlib import image
import numpy as np
import json
import re
import random
import math

from random import randint
from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import ContinuousSpace
from mesa.datacollection import DataCollector
from Sensors import MySensor

number_of_agents = 5


class AgentEnvironmentMap:
    def __init__(self, bw_image_path, dist_per_pix=1.0):
        self.img = image.imread(bw_image_path)
        self.shape = (self.img.shape[0], self.img.shape[1])
        self.dist_per_pix = dist_per_pix
        self.max_x = self.shape[1] * dist_per_pix - 1
        self.max_y = self.shape[0] * dist_per_pix - 1
        self.wall_mask = np.zeros(self.shape, dtype=bool)
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                self.wall_mask[i, j] = self.img[i, j, 0] < 0.1

    def is_wall(self, x, y):
        i = int(y / self.dist_per_pix)
        j = int(x / self.dist_per_pix)
        return self.wall_mask[i, j]

    def to_ascii1(self):
        return '\n'.join([''.join(['#' if m else '.' for m in ln]) for ln in self.wall_mask])


class PhysicalAgent(Agent):
    def __init__(self, unique_id, model):
        self.is_moving = True
        super().__init__(unique_id, model)

    def reset_waypoints(self, waypoints=None):
        if waypoints:
            self.waypoints = waypoints
        self.model.space.move_agent(self, self.waypoints[0])
        self.next_waypoint_index = 1
        self.is_moving = True

    def get_points_to_show(self):
        return {'agent': self.pos, 'final_target': self.waypoints[-1],
                'next_target': self.waypoints[self.next_waypoint_index]}

    def speed_agent(self):
        SP_A = randint(1, 5)
        return SP_A

    def step(self):

        TARGET_SENSITIVITY = 3
        search_target = True
        while search_target:
            dx = self.waypoints[self.next_waypoint_index][0] - self.pos[0]
            dy = self.waypoints[self.next_waypoint_index][1] - self.pos[1]
            dx = ((dx * (math.cos(math.radians(randint(-3, 3))))) + (dy * (math.sin(math.radians(randint(-3, 3))))))
            dy = ((dx * (-math.sin(math.radians(randint(-3, 3))))) + (dy * (math.cos(math.radians(randint(-3, 3))))))
            d = np.sqrt(dx * dx + dy * dy)
            if d < TARGET_SENSITIVITY:
                if self.next_waypoint_index < len(self.waypoints) - 1:
                    self.next_waypoint_index += 1
                else:
                    self.is_moving = False
                    return
            else:
                search_target = False
        new_x = self.pos[0] + self.speed_agent() * dx / d
        new_y = self.pos[1] + self.speed_agent() * dy / d
        if not self.model.env_map.is_wall(new_x, new_y):
            print(f'#{self.unique_id} is moving to ({new_x}, {new_y}) forwards waypoint #{self.next_waypoint_index} with the speed {self.speed_agent()}')

            self.model.space.move_agent(self, (new_x, new_y))

        my_file = open(r".\way_points_history_for_agents\agent" + str(self.unique_id) + ".txt", "a")
        my_file.write(str(new_x) + " " + str(new_y) + " ")
        my_file.close()


class IndoorModel(Model):
    def __init__(self, agents_json_path='agents.json', env_map_path='map_2floor_bw.png'):
        super().__init__()
        hard_dx = 235
        hard_dy = 76
        self.path = env_map_path
        self.env_map = AgentEnvironmentMap(env_map_path)
        self.space = ContinuousSpace(self.env_map.max_x, self.env_map.max_y, False)
        self.schedule = RandomActivation(self)

        with open(agents_json_path) as f:
            agent_json = json.load(f)
        for k in range(0, number_of_agents):
            my_file = open(r".\way_points_history_for_agents\agent" + str(k) + ".txt", "w+")
            my_file.close()
            for i, aj in enumerate(agent_json):
                with open(aj['waypoints_path']) as f:
                    lns = f.readlines()
                    waypoints = []
                    for ln in lns:
                        parts = re.findall('\d+', ln)
                        waypoints.append((int(parts[0].strip()) - hard_dx , int(parts[1].strip()) - hard_dy ))
            a = PhysicalAgent(k, self)
            self.schedule.add(a)
            self.space.place_agent(a, waypoints[0])
            a.reset_waypoints(waypoints)

        # добавлен один сенсор
        data_datchik = MySensor(555, self, (528.9826976383029, 177.194545804881), 10)
        self.schedule.add(data_datchik)
        self.space.place_agent(data_datchik, data_datchik.pos)

        self.data_collector = DataCollector({'moving_agents_num': 'moving_agents_num'},
                                            {'is_moving': 'is_moving', 'x': lambda a: a.pos[0],
                                             'y': lambda a: a.pos[1]})

        self.moving_agents_num = 0
        self.running = True
        self.data_collector.collect(self)

    def step(self):
        self.schedule.step()
        self.data_collector.collect(self)
        self.moving_agents_num = sum([a.is_moving for a in self.schedule.agents])
        self.running = self.moving_agents_num > 0


    def agents_neighbors(self):
        # get_neighbors() принимает координаты агента и радиус поиска
        neighbors = self.space.get_neighbors(self.pos, self.speed_sensor_radius, False)

    def plot_explicitly(self):
        plt.imshow(self.env_map.img)
        for a in self.schedule.agents:
            plt.plot(a.pos[0], a.pos[1], 'bo')
        # plt.plot(self.target[0], self.target[1], 'r+')


