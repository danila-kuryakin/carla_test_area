
import glob
import json5 as json
import os
import sys
import argparse
import time
import numpy as np
import carla
import cv2 as cv
from matplotlib import pyplot as plt
from carla import Location, Rotation, Transform

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()


class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None


class RGBCamera:
    def __init__(self, world, display_man, transform, options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(transform, options)
        self.sensor_options = options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, transform, options):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        disp_size = self.display_man.get_display_size()
        camera_bp.set_attribute('image_size_x', str(disp_size[0]))
        camera_bp.set_attribute('image_size_y', str(disp_size[1]))

        camera_bp.set_attribute('fov', str(options['fov']))

        for key in options['distortion'].keys():
            camera_bp.set_attribute(key, str(options['distortion'][key]))

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=None)
        camera.listen(self.save_rgb_image)
        self.camera = camera

        return camera

    def get_sensor(self):
        return self.sensor

    def get_image(self):
        self.camera.listen(lambda image: cv.imshow('Frame', image))

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


def run_simulation(client):
    json_data = read_json("settings.json")

    display_manager = None

    try:
        # Getting the world and
        world = client.get_world()

        map = world.get_map()

        wpJunction = map.get_waypoint(Location(json_data['junction_coordinate']['x'],
                                               json_data['junction_coordinate']['y'],
                                               json_data['junction_coordinate']['z']))

        junction = wpJunction.get_junction()
        if junction:
            traffic_lights = world.get_traffic_lights_in_junction(junction.id)
            traffic_lights_len = len(traffic_lights)
            print('Number of traffic lights: ', traffic_lights_len)
        else:
            print('The junction not found')
            exit(0)

        display_manager = DisplayManager(grid_size=[traffic_lights_len, 4], window_size=[json_data['width'], json_data['height']])
        i = 0
        for tl in traffic_lights:
            if i >= traffic_lights_len:
                break
            print(tl.get_transform())

            transform = tl.get_transform()
            if -1.0 < transform.rotation.yaw < 1.0:
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x - 7, y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 0])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x - 7 - json_data['baseline'], y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 1])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x - 7 - json_data['baseline'], y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 2])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x - 7 , y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 3])

            elif 89.0 < transform.rotation.yaw < 91.0:
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y - 7, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 0])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y - 7 - json_data['baseline'], z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 1])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y - 7- json_data['baseline'], z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 2])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y - 7 , z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 3])

            elif 179.0 < transform.rotation.yaw < 181.0:
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x + 7, y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 0])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x + 7  + json_data['baseline'], y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 1])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x + 7+ json_data['baseline'], y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 2])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x + 7 , y=transform.location.y, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 3])

            else:
                RGBCamera(world, display_manager,
                        Transform(Location(x=transform.location.x, y=transform.location.y + 7, z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 0])
                RGBCamera(world, display_manager,
                        Transform(Location(x=transform.location.x, y=transform.location.y + 7 + json_data['baseline'], z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw + 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 1])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y + 7+ json_data['baseline'], z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 + json_data['angel'] + json_data['intrinsic']['left']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['left']['pitch'],
                                     roll=json_data['intrinsic']['left']['roll'])), json_data, display_pos=[i, 2])
                RGBCamera(world, display_manager,
                          Transform(Location(x=transform.location.x, y=transform.location.y + 7 , z=transform.location.z + 4),
                            Rotation(yaw=transform.rotation.yaw - 90 - json_data['angel'] + json_data['intrinsic']['right']['yaw'],
                                     pitch=json_data['orientation']["tilt"] + json_data['intrinsic']['right']['pitch'],
                                     roll=json_data['intrinsic']['right']['roll'])), json_data, display_pos=[i, 3])
            i = i + 1

        #Simulation loop
        call_exit = False
        while True:
            world.tick()

            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

    finally:
        if display_manager:
            display_manager.destroy()



def read_json(path):
    with open(path, "r") as read_file:
        json_data = ''.join(line for line in read_file if not line.startswith('//'))
        print(json_data)
        return json.loads(json_data)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()