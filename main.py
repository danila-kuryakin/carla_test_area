# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import random

import carla
import pygame
from carla import Transform, Location, Rotation

import cv2 as cv2

_HOST_ = '127.0.0.1'
_PORT_ = 2000

if __name__ == '__main__':
    client = carla.Client(_HOST_, _PORT_)
    # Get the buildings in the world
    world = client.get_world()
    env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

    # Access individual building IDs and save in a set
    building_01 = env_objs[0]
    building_02 = env_objs[1]
    objects_to_toggle = {building_01.id, building_02.id}

    # Toggle buildings off
    world.enable_environment_objects(objects_to_toggle, False)
    # Toggle buildings on
    world.enable_environment_objects(objects_to_toggle, True)




# See PyCharm help at https://www.jetbrains.com/help/pycharm/
