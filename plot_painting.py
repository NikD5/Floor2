import matplotlib.pyplot as plt
from random import randint
import os
# переменная для задания кол-ва агентов на графике
from Agents import  number_of_agents


# функция берет из файла координаты агента и сохраняет их в два массива
def reading_coordinates(filename):
    x_coordinates_f = []
    y_coordinates_f = []
    text_file = open(r".\way_points_history_for_agents\agent" + str(filename) + ".txt", "r")
    lines = text_file.read().split(' ')
#    print(lines)
    for i in range(0, len(lines) - 1, 2):
        x_coordinates_f.append(float(lines[i]))
        y_coordinates_f.append(float(lines[i + 1]))
    text_file.close()
    return x_coordinates_f, y_coordinates_f


print("input number of agents for a plot, no more than ", number_of_agents)
number_of_agents_for_a_plot = int(input())
if number_of_agents_for_a_plot > number_of_agents:
    number_of_agents_for_a_plot = number_of_agents


colors = []
for i in range(number_of_agents):
    colors.append('#%06X' % randint(0, 0xFFFFFF))

img = plt.imread('map_2floor_bw.png')
fig, ax = plt.subplots()
ax.imshow(img)
for i in range(0, number_of_agents_for_a_plot):
    x_coordinates, y_coordinates = reading_coordinates(i)
    plt.scatter(x_coordinates, y_coordinates, color=str(colors[i]), s=10)
    # plt.plot(x_coordinates, y_coordinates, color=str(colors[i]))


plt.ylabel('y_coordinate_of_agent')
plt.xlabel('x_coordinate_of_agent')
plt.show()

print("Do you want to remove agents_history files? yes/no")
answer = input()
if answer == 'yes':
    for i in range(0, number_of_agents):
        os.remove(r".\way_points_history_for_agents\/agent" + str(i) + ".txt")
    print("files have been removed")
else:
    print("files still there")