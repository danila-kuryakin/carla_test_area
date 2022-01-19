
import glob
import os
import sys
import cv2 as cv2

from carla import Location, Rotation, Transform

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import time
import numpy as np


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
    def __init__(self, world, display_man, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, transform, attached, sensor_options):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        disp_size = self.display_man.get_display_size()
        camera_bp.set_attribute('image_size_x', str(disp_size[0]))
        camera_bp.set_attribute('image_size_y', str(disp_size[1]))

        for key in sensor_options:
            camera_bp.set_attribute(key, sensor_options[key])

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
        camera.listen(self.save_rgb_image)

        return camera

    def get_sensor(self):
        return self.sensor

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


def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    # display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:
        # Getting the world and
        world = client.get_world()

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)




        map = world.get_map()

        # wpJunction = map.get_waypoint(Location(-46.656982421875, 21.270511627197266, 4.308578014373779))
        # wpJunction = map.get_waypoint(Location(-50.80909729003906,-60.55316162109375,5.965755462646484))
        # wpJunction = map.get_waypoint(Location(-48.279052734375, 132.96006774902344, 6.757640838623047))
        #
        # junction = wpJunction.get_junction()
        # if junction:
        #     traffic_lights = world.get_traffic_lights_in_junction(junction.id)
        #     print('Traffic Lights: ', len(traffic_lights))
        # else:
        #     print('The junction not found')
        # # for tl in traffic_lights:
        # #     print(tl.get_transform())
        #
        # display_manager = DisplayManager(grid_size=[2, 2], window_size=[args.width, args.height])
        # i = 0
        # for tl in traffic_lights:
        #     print(tl.get_transform())
        #     if i > 4:
        #         break
        #
        #     transform = tl.get_transform()
        #     if -1.0 < transform.rotation.yaw < 1.0:
        #         RGBCamera(world, display_manager,
        #                   Transform( Location(x=transform.location.x - 6, y=transform.location.y, z=transform.location.z + 4),
        #                   Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {}, display_pos=[i // 2, i % 2])
        #     elif 89.0 < transform.rotation.yaw < 91.0:
        #         RGBCamera(world, display_manager,
        #                   Transform(Location(x=transform.location.x, y=transform.location.y - 6, z=transform.location.z + 4),
        #                   Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {}, display_pos=[i // 2, i % 2])
        #     elif 179.0 < transform.rotation.yaw < 181.0:
        #         RGBCamera(world, display_manager,
        #                   Transform(Location(x=transform.location.x + 6, y=transform.location.y , z=transform.location.z + 4),
        #                   Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {}, display_pos=[i // 2, i % 2])
        #
        #     else:
        #         RGBCamera(world, display_manager,
        #                   Transform(Location(x=transform.location.x, y=transform.location.y + 6, z=transform.location.z + 4),
        #                   Rotation(yaw=transform.rotation.yaw+90, pitch=0, roll=0)), vehicle, {}, display_pos=[i//2, i%2])
        #     i = i + 1

        display_manager = DisplayManager(grid_size=[4, 4], window_size=[args.width, args.height])
        landmarks = map.get_all_landmarks()
        print(len(landmarks))

        i = 0
        for lm in landmarks:
            if i > 16:
                break
            traffic_lights = world.get_traffic_light(lm)

            if traffic_lights:
                transform = traffic_lights.get_transform()
                # print(transform)
                if -1.0 < transform.rotation.yaw < 1.0:
                    RGBCamera(world, display_manager,
                              Transform(Location(x=transform.location.x - 6, y=transform.location.y,
                                                 z=transform.location.z + 4),
                                        Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {},
                              display_pos=[i // 4, i % 4])
                elif 89.0 < transform.rotation.yaw < 91.0:
                    RGBCamera(world, display_manager,
                              Transform(Location(x=transform.location.x, y=transform.location.y - 6,
                                                 z=transform.location.z + 4),
                                        Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {},
                              display_pos=[i // 4, i % 4])
                elif 179.0 < transform.rotation.yaw < 181.0:
                    RGBCamera(world, display_manager,
                              Transform(Location(x=transform.location.x + 6, y=transform.location.y,
                                                 z=transform.location.z + 4),
                                        Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {},
                              display_pos=[i // 4, i % 4])

                else:
                    RGBCamera(world, display_manager,
                              Transform(Location(x=transform.location.x, y=transform.location.y + 6,
                                                 z=transform.location.z + 4),
                                        Rotation(yaw=transform.rotation.yaw + 90, pitch=0, roll=0)), vehicle, {},
                              display_pos=[i // 4, i % 4])
                i = i + 1

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()
            # display_manager2.render()

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
        # if display_manager2:
        #     display_manager2.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])



def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
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
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()