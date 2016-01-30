#!python
"""
Common robot helpers
"""

import math
import time
from abc import ABCMeta, abstractmethod, abstractproperty
import weakref


RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180


class BotDriver:
    """
    Robot driver interface

    Used to drive robot movement in a generic manner.
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def drive(self, power=30, distance=None, turn_radius=0, wait=True):
        """
        Drive the robot at the given speed and heading until :func:`stop` is called
        :param power: desired motor power in range [-100, 100]
        :param distance: distance to drive in meters.  None => infinite distance
        :param turn_radius: turn radius in meters.  0 => drive straight (no turning)
        :param wait: if True, wait for operation to complete before returning.  Otherwise, returns immediately.
        """
        pass

    @abstractmethod
    def turn_in_place(self, power=30, degrees=None, wait=True):
        """
        Turn the robot in place
        :param power: desired motor power in range [-100, 100].  <0 => turn left, >0 => turn right
        :param degrees: number of degrees to rotate.  None => infinite degrees
        :param wait: if True, wait for operation to complete before returning.  Otherwise, returns immediately.
        """
        pass

    @abstractmethod
    def stop(self):
        """
        Stop all motors gracefully
        """
        pass

    @abstractmethod
    def brake(self):
        """
        Brake all motors
        """
        pass

    @abstractmethod
    def float(self):
        """
        Float all motors
        """
        pass

    @abstractmethod
    def hold(self):
        """
        Hold all motors at current position
        """
        pass


class DiffDescription:
    """
    Describes a differential drive robot

    :param left_motor: :class:`PiStorms.PiStormsMotor` instance for the motor on the left side of the robot
    :param right_motor: :class:`PiStorms.PiStormsMotor` instance for the motor on the right side of the robot
    :param axel_radius: half the distance between the two drive wheels in meters
    :param wheel_radius: radius of each of the two drive wheels in meters
    :param reverse_forward: negate motor speeds (set to True if your robot is driving backwards instead of forward)
    """

    def __init__(self, left_motor, right_motor, axel_radius=0.1, wheel_radius=0.05, reverse_forward=False):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.axel_radius = axel_radius
        self.wheel_radius = wheel_radius
        self.forward_coeff = -1 if reverse_forward else 1

    def left_pos(self):
        return self.forward_coeff * self.left_motor.pos()

    def right_pos(self):
        return self.forward_coeff * self.right_motor.pos()


class DiffDriver(BotDriver):
    """
    Differential drive robot driver

    :param desc: `DiffDescription` instance
    """

    def __init__(self, desc):
        super(DiffDriver, self).__init__()
        self.__desc = desc

    def drive(self, power=30, distance=None, turn_radius=0, wait=True):
        if turn_radius == 0:
            if distance is None:
                self.__desc.left_motor.setSpeed(self.__power_to_speed(power))
                self.__desc.right_motor.setSpeed(self.__power_to_speed(power))
            else:
                self.__desc.left_motor.runDegs(
                    int(self.__desc.forward_coeff * RAD_TO_DEG * distance / self.__desc.wheel_radius),
                    self.__power_to_speed(power),
                    True,
                    True
                )
                self.__desc.right_motor.runDegs(
                    int(self.__desc.forward_coeff * RAD_TO_DEG * distance / self.__desc.wheel_radius),
                    self.__power_to_speed(power),
                    True,
                    True
                )

                if wait:
                    self.__desc.left_motor.waitUntilNotBusy()
                    self.__desc.right_motor.waitUntilNotBusy()
            return

        if turn_radius < 0:
            inner_motor = self.__desc.left_motor
            outer_motor = self.__desc.right_motor
            turn_radius = -turn_radius
        else:
            inner_motor = self.__desc.right_motor
            outer_motor = self.__desc.left_motor

        outer_speed = (turn_radius + self.__desc.axel_radius) * power / turn_radius
        inner_speed = (turn_radius - self.__desc.axel_radius) * power / turn_radius

        scale = min(1.0, 100 / max(abs(outer_speed), abs(inner_speed)))

        outer_speed *= scale
        inner_speed *= scale

        if distance is None:
            outer_motor.setSpeed(outer_speed)
            inner_motor.setSpeed(inner_speed)
        else:
            outer_degs = self.__desc.forward_coeff * RAD_TO_DEG * (turn_radius + self.__desc.axel_radius) * distance / \
                         (self.__desc.wheel_radius * turn_radius)
            inner_degs = self.__desc.forward_coeff * RAD_TO_DEG * (turn_radius - self.__desc.axel_radius) * distance / \
                         (self.__desc.wheel_radius * turn_radius)
            outer_motor.runDegs(int(outer_degs), outer_speed, True, False)
            inner_motor.runDegs(int(inner_degs), inner_speed, True, False)

            if wait:
                self.__desc.left_motor.waitUntilNotBusy()
                self.__desc.right_motor.waitUntilNotBusy()

    def turn_in_place(self, power=30, degrees=None, wait=True):
        if degrees is None:
            self.__desc.left_motor.setSpeed(self.__power_to_speed(power))
            self.__desc.right_motor.setSpeed(self.__power_to_speed(-power))
        else:
            motor_degrees = int(self.__desc.axel_radius * degrees / self.__desc.wheel_radius)
            self.__desc.left_motor.runDegs(self.__desc.forward_coeff * motor_degrees, self.__power_to_speed(power),
                                           True, True)
            self.__desc.right_motor.runDegs(self.__desc.forward_coeff * -motor_degrees, self.__power_to_speed(power),
                                            True, True)

            if wait:
                self.__desc.left_motor.waitUntilNotBusy()
                self.__desc.right_motor.waitUntilNotBusy()

    def stop(self):
        self.brake()
        time.sleep(0.250)
        self.float()

    def brake(self):
        self.__desc.left_motor.brake()
        self.__desc.right_motor.brake()

    def float(self):
        self.__desc.left_motor.float()
        self.__desc.right_motor.float()

    def hold(self):
        self.__desc.left_motor.waitUntilNotBusy()
        self.__desc.right_motor.waitUntilNotBusy()
        self.__desc.left_motor.hold()
        self.__desc.right_motor.hold()

    def __power_to_speed(self, power):
        power *= self.__desc.forward_coeff
        power = max(power, -100)
        power = min(power, 100)
        return power

    def __set_power(self, motor, power):
        motor.setSpeed(self.__power_to_speed(power))


class EventLoopProcessor:
    """
    Runs code in the event loop
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def update(self):
        """
        :returns Whether the event loop should continue updating other processors
        """
        pass


class EventLoop:
    """
    Defines and runs a prioritized event loop

    :param processors: list of `EventLoopProcessor` instances to be updates on each iteraton of the loop in order
    """

    def __init__(self, processors=()):
        self.__processors = processors

    def step(self):
        """
        Executes one iteration of the event loop
        """
        for processor in self.__processors:
            if processor.update():
                break

    def run_forever(self):
        """
        Runs the event loop forever until the applicaton terminates
        """
        while True:
            self.step()


from flask import Flask, render_template, request, send_from_directory
from flask.views import View
from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer
import json
import threading


class WebServer:
    """
    Basic web server with WebSocket support

    Requires [Flask](http://flask.pocoo.org/) and [gevent-websocket](https://bitbucket.org/noppo/gevent-websocket).
    """

    def __init__(self, port=9000, screen=None, plugins=()):
        self.__ip_address = None
        self.__port = port
        self.__screen = screen
        self.__app = Flask(__name__)

        for plugin in plugins:
            plugin.configure(self.__app)

    @property
    def ip_address(self):
        return self.__ip_address

    def run(self, async=False):
        def run_server(port, app, screen, start_event):
            # self.app.run(host='0.0.0.0', port=self.__port, debug=True)
            http_server = WSGIServer(('', port), app, handler_class=WebSocketHandler)

            # get public IP address
            import socket

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('microsoft.com', 80))
            ip_address = s.getsockname()[0]
            s.close()

            self.__ip_address = ip_address

            if screen:
                screen.termPrintln('Server started.')
                screen.termPrintln('Connect at:')
                screen.termPrintln('http://%s:%d' % (ip_address, port))
                screen.termPrintln(' ')

            print 'Server started.  Connect at http://%s:%d' % (ip_address, port)
            start_event.set()
            http_server.serve_forever()

        start_event = threading.Event()
        server_thread = threading.Thread(target=run_server, args=(self.__port, self.__app, self.__screen, start_event))
        server_thread.daemon = True
        server_thread.start()
        start_event.wait()

        if not async:
            server_thread.join()


class WebServerPlugin:
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def configure(self, app):
        """
        Configure the web server
        :param app: the web server's `Flask` app instance
        """
        pass

    class BaseView(View):
        def __init__(self):
            pass

        @classmethod
        def as_view(cls, name, *class_args, **class_kwargs):
            """
            We override View's as_view method so we can construct the view class once
            """

            self = cls(*class_args, **class_kwargs)

            def view(*args, **kwargs):
                return self.dispatch_request(*args, **kwargs)

            if cls.decorators:
                view.__name__ = name
                view.__module__ = cls.__module__
                for decorator in cls.decorators:
                    view = decorator(view)

            # We attach the view class to the view function for two reasons:
            # first of all it allows us to easily figure out what class-based
            # view this thing came from, secondly it's also used for instantiating
            # the view class so you can actually replace it with something else
            # for testing purposes and debugging.
            view.view_class = cls
            view.view_class_instance = self
            view.__name__ = name
            view.__doc__ = cls.__doc__
            view.__module__ = cls.__module__
            view.methods = cls.methods

            return view

        pass

    class WebSocketView(BaseView):
        __metaclass__ = ABCMeta

        def __init__(self):
            self.__clients = set()
            self.__lock = threading.Lock()

        @abstractmethod
        def connected(self, ws):
            pass

        @abstractmethod
        def message_recv(self, ws, message):
            pass

        def dispatch_request(self):
            print 'setting up web socket'
            try:
                if request.environ.get('wsgi.websocket'):
                    ws = request.environ['wsgi.websocket']
                    self.connected(ws)
                    with self.__lock:
                        self.__clients.add(ws)

                    while True:
                        try:
                            message = ws.receive()
                        except:
                            break
                        self.message_recv(ws, message)

                    with self.__lock:
                        self.__clients.remove(ws)
            except Exception, e:
                print 'ERROR:', e.message

        def broadcast(self, message):
            with self.__lock:
                for ws in self.__clients:
                    try:
                        ws.send(message)
                    except:
                        pass


class RemoteControl(WebServerPlugin, EventLoopProcessor):
    """
    Enables remote control of the robot from a web browser

    :param bot_driver: `BotDriver` instance
    :param title: page title
    :param locator: locator used to report robot pose periodically
    :param path_follower: path follower used to follow user-provided paths
    :param broadcast_interval: time between broadcasts of state updates in seconds
    """

    def __init__(self, bot_driver, title='Robot Remote', locator=None, path_finder=None, path_follower=None, broadcast_interval=0.250):
        WebServerPlugin.__init__(self)
        EventLoopProcessor.__init__(self)
        self.__title = title
        self.__api_view = RemoteControl.ApiView.as_view(
            'api_remote',
            bot_driver=bot_driver,
            locator=locator,
            path_finder=path_finder,
            path_follower=path_follower,
            broadcast_interval=broadcast_interval)

    def configure(self, app):
        app.add_url_rule('/', view_func=RemoteControl.IndexView.as_view('index', title=self.__title))
        app.add_url_rule('/api/remote', view_func=self.__api_view)

    def update(self):
        self.__api_view.view_class_instance.update()

    class IndexView(WebServerPlugin.BaseView):
        def __init__(self, title):
            self.__title = title

        def dispatch_request(self):
            print 'remote control client connected'
            return render_template('index.html', title=self.__title)

    class ApiView(WebServerPlugin.WebSocketView):
        def __init__(self, bot_driver, locator, path_finder, path_follower, broadcast_interval):
            super(RemoteControl.ApiView, self).__init__()
            self.__bot_driver = bot_driver
            self.__locator = locator
            self.__path_finder = path_finder
            self.__path_follower = path_follower
            self.__broadcast_interval = broadcast_interval
            self.__broadcast_time = None

        def connected(self, ws):
            for message in self.get_state_update_messages():
                ws.send(message)

        def message_recv(self, ws, message):
            data = json.loads(message)

            if data is None or not 'action' in data:
                return

            action = data['action']

            if action == 'drive':
                self.__bot_driver.drive(power=data['params']['power'], wait=False)
            elif action == 'turn_in_place':
                self.__bot_driver.turn_in_place(power=data['params']['power'], wait=False)
            elif action == 'stop':
                self.__bot_driver.stop()
            elif action == 'float':
                self.__bot_driver.float()
            elif action == 'set_path':
                if self.__path_follower is not None:
                    self.__path_follower.path = data['params']['path']
            elif action == 'set_target':
                if self.__path_finder is not None:
                    self.__path_finder.target = data['params']['target']

        def update(self):
            now = time.time()
            if self.__broadcast_time is None or (now - self.__broadcast_time) > self.__broadcast_interval:
                for message in self.get_state_update_messages():
                    self.broadcast(message)
                self.__broadcast_time = now

        def get_state_update_messages(self):
            messages = []

            if self.__locator:
                (pose, confidence) = self.__locator.locate()
                messages.append(json.dumps({
                    'action': 'locator_update',
                    'pose': pose,
                    'confidence': confidence
                }))

            if self.__path_follower:
                messages.append(json.dumps({
                    'action': 'path_update',
                    'path': self.__path_follower.path
                }))

            if self.__path_finder:
                voxels = self.__path_finder.voxels
                messages.append(json.dumps({
                    'action': 'voxels_update',
                    'voxels': {
                        'offset': voxels.offset,
                        'scale': voxels.scale,
                        'grid': voxels.grid
                    }
                }))

            return messages


class Locator:
    """
    Determines an object's current pose (position and orientation) in the world
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def locate(self):
        """
        Get the object's pose relative to the world origin
        :return: `((x, y, theta), confidence)`, where:
            `x` is the x-coordinate
            `y` is the y-coordinate
            `theta` is the orientation in degrees.  0 => forward direction is +x, 90 => forward direction is +y
            `confidence` is the estimated accuracy of this pose in range [0, 1]
        """
        pass


class DiffLocator(Locator, EventLoopProcessor):
    """
    Estimates pose of a differential drive robot by monitoring motor tachometers

    :param desc: `DiffDescription` instance
    """

    def __init__(self, desc):
        Locator.__init__(self)
        EventLoopProcessor.__init__(self)
        self.__desc = desc
        self.__pose = (0, 0, 0)
        self.__confidence = 1
        self.__left_pos = desc.left_pos()
        self.__right_pos = desc.right_pos()

    def locate(self):
        return self.__pose, self.__confidence

    @staticmethod
    def update(self):

        # get previous values
        prev_x, prev_y, prev_theta = self.__pose
        prev_confidence = self.__confidence
        prev_left_pos = self.__left_pos
        prev_right_pos = self.__right_pos

        left_pos = self.__desc.left_pos()
        right_pos = self.__desc.right_pos()

        # change in motor positions
        left_pos_delta = left_pos - prev_left_pos
        right_pos_delta = right_pos - prev_right_pos

        if not (left_pos_delta == 0 and right_pos_delta == 0):

            if left_pos_delta == right_pos_delta:
                # straight
                x_local = DEG_TO_RAD * left_pos_delta * self.__desc.wheel_radius
                y_local = 0
                theta_delta_rad = 0
            else:
                # turn
                turn_radius = self.__desc.axel_radius * float(right_pos_delta + left_pos_delta) / (
                    right_pos_delta - left_pos_delta)
                left_pos_delta_rad = DEG_TO_RAD * left_pos_delta
                right_pos_delta_rad = DEG_TO_RAD * right_pos_delta
                if turn_radius == self.__desc.axel_radius:
                    theta_delta_rad = self.__desc.wheel_radius * right_pos_delta_rad / (
                        turn_radius + self.__desc.axel_radius)
                else:
                    theta_delta_rad = self.__desc.wheel_radius * left_pos_delta_rad / (
                        turn_radius - self.__desc.axel_radius)
                x_local = turn_radius * math.sin(theta_delta_rad)
                y_local = turn_radius - turn_radius * math.cos(theta_delta_rad)

            def matmult(a, b):
                zip_b = zip(*b)
                # uncomment next line if python 3 :
                # zip_b = list(zip_b)
                return [[sum(ele_a * ele_b for ele_a, ele_b in zip(row_a, col_b))
                         for col_b in zip_b] for row_a in a]

            prev_theta_rad = DEG_TO_RAD * prev_theta

            v_local = [[x_local], [y_local], [1.0]]
            m_rotate = [
                [math.cos(prev_theta_rad), math.sin(prev_theta_rad), 0],
                [-math.sin(prev_theta_rad), math.cos(prev_theta_rad), 0],
                [0, 0, 1.0]
            ]
            m_translate = [
                [1.0, 0, prev_x],
                [0, 1.0, prev_y],
                [0, 0, 1.0]
            ]

            v_world = matmult(m_translate, matmult(m_rotate, v_local))

            theta = prev_theta + RAD_TO_DEG * theta_delta_rad
            x = v_world[0][0]
            y = v_world[1][0]

            confidence = prev_confidence * 0.8

            self.__pose = (x, y, theta)
            self.__confidence = confidence
            self.__left_pos = left_pos
            self.__right_pos = right_pos


class RemoteLocator(Locator, WebServerPlugin):
    """
    Gets current location from a remote device using a web server
    """

    def __init__(self):
        Locator.__init__(self)
        WebServerPlugin.__init__(self)
        self.__lock = threading.Lock()
        self.__pose = (0, 0, 0)

    def configure(self, app):
        def set_pose(pose):
            with self.__lock:
                self.__pose = pose

        app.add_url_rule('/api/locator', view_func=RemoteLocator.ApiView.as_view('api_locator', set_pose=set_pose))

    def locate(self):
        with self.__lock:
            return self.__pose, 1

    class ApiView(WebServerPlugin.WebSocketView):
        def __init__(self, set_pose):
            super(RemoteLocator.ApiView, self).__init__()
            self.__set_pose = set_pose

        def connected(self, ws):
            pass

        def message_recv(self, ws, message):
            data = json.loads(message)

            if not data or not 'action' in data:
                return

            action = data['action']

            if action == 'set_pose':
                self.__set_pose(data['pose'])


class PathFollower(EventLoopProcessor):
    """
    Drives the robot along the given path

    :param bot_driver: drive control for the robot
    :param locator: locator that determines the robot's pose
    :param distance_tolerance: minimum distance from the target location
    """

    def __init__(self, bot_driver, locator, distance_tolerance=0.05, heading_tolerance=10, forward_power=50,
                 turn_power=30):
        super(PathFollower, self).__init__()
        self.__bot_driver = bot_driver
        self.__locator = locator
        self.__distance_tolerance = distance_tolerance
        self.__heading_tolerance = heading_tolerance
        self.__forward_power = forward_power
        self.__turn_power = turn_power
        self.__path = []
        self.__lock = threading.Lock()

    @property
    def path(self):
        with self.__lock:
            return self.__path[:]

    @path.setter
    def path(self, path):
        """
        Set target path
        :param path: list of (x, y) pairs that the robot should drive to in order
        """
        with self.__lock:
            print 'new path: ', path
            self.__path = path

    def update(self):
        with self.__lock:
            if self.__path:
                (x_dest, y_dest) = self.__path[0]
                (x, y, theta), confidence = self.__locator.locate()
                dx = x_dest - x
                dy = y_dest - y

                dist = math.sqrt(dx * dx + dy * dy)

                if dist < self.__distance_tolerance:
                    print 'destination reached: ', (x_dest, y_dest)
                    self.__bot_driver.float()
                    time.sleep(0.250)
                    del self.__path[0]
                    return

                theta_dest = RAD_TO_DEG * math.atan2(dy, dx)

                def canonical_angle(degrees):
                    while degrees >= 180:
                        degrees -= 360
                    while degrees < -180:
                        degrees += 360
                    return degrees

                dtheta = canonical_angle(canonical_angle(theta_dest) - canonical_angle(theta))

                if dtheta < -self.__heading_tolerance:
                    # turn right
                    self.__bot_driver.turn_in_place(power=self.__turn_power)
                elif dtheta > self.__heading_tolerance:
                    # turn left
                    self.__bot_driver.turn_in_place(power=-self.__turn_power)
                else:
                    # drive forward
                    self.__bot_driver.drive(power=self.__forward_power)


import os


class ConfigServer(WebServerPlugin):
    """
    Serves an opaque config blob to external clients
    """

    CONFIG_BLOB_FILENAME = 'bot_config.blob'

    def __init__(self, storage_folder='/tmp'):
        super(ConfigServer, self).__init__()
        self.__storage_folder = storage_folder

    def configure(self, app):
        app.add_url_rule('/config/blob', view_func=ConfigServer.ConfigBlobView.as_view(
            'config_blob',
            storage_folder=self.__storage_folder
        ))

    class ConfigBlobView(WebServerPlugin.BaseView):
        methods = ['GET', 'POST']

        def __init__(self, storage_folder):
            self.__storage_folder = storage_folder

        def dispatch_request(self):
            if request.method == 'GET':
                resp = send_from_directory(self.__storage_folder, ConfigServer.CONFIG_BLOB_FILENAME)
                resp.headers['Access-Control-Allow-Origin'] = '*'
                return resp
            else:
                f = request.files['file']
                f.save(os.path.join(self.__storage_folder, ConfigServer.CONFIG_BLOB_FILENAME), buffer_size=20*1024*1024)
                return 'Upload successful'


class PathFinder:
    """
    Generates a path to a target location

    :param path_changed_handler: callback function invoked when the path to the target changes
    """

    __metaclass__ = ABCMeta

    def __init__(self, path_changed_handler):
        pass

    @property
    @abstractmethod
    def target(self):
        """
        Current target location as an (x, y) tuple
        """
        pass

    @target.setter
    @abstractmethod
    def target(self, target):
        pass

    @property
    @abstractmethod
    def voxels(self):
        """
        Current voxel map as a `VoxelMap` instance
        """
        pass

    class VoxelMap:
        """
        Represents a voxel map that was used to compute a given path
        """
        offset = [0, 0]
        scale = 0
        grid = [[]]

        def __init__(self):
            pass


class RemotePathFinder(PathFinder, WebServerPlugin):
    def __init__(self, path_changed_handler):
        self.__voxels = PathFinder.VoxelMap()
        self.__api_view = RemotePathFinder.ApiView.as_view('api_pathfinder', path_changed_handler=path_changed_handler)

    @property
    def target(self):
        return self.__api_view.view_class_instance.target

    @target.setter
    def target(self, target):
        self.__api_view.view_class_instance.target = target

    @property
    def voxels(self):
        return self.__api_view.view_class_instance.voxels

    def configure(self, app):
        app.add_url_rule('/api/pathfinder', view_func=self.__api_view)

    class ApiView(WebServerPlugin.WebSocketView):
        def __init__(self, path_changed_handler):
            super(RemotePathFinder.ApiView, self).__init__()
            self.__path_changed_handler = path_changed_handler
            self.__target = None
            self.__path = []
            self.__voxels = PathFinder.VoxelMap()

        @property
        def target(self):
            return self.__target

        @target.setter
        def target(self, target):
            self.__target = target
            self.broadcast(json.dumps({ 'action': 'set_target', 'params': { 'target': target } }))

        @property
        def voxels(self):
            return self.__voxels

        def connected(self, ws):
            ws.send(json.dumps({ 'action': 'set_target', 'params': { 'target': self.__target } }))

        def message_recv(self, ws, message):
            data = json.loads(message)

            if not data or not 'action' in data:
                return

            action = data['action']

            if action == 'set_path':

                path = data['params']['path']
                voxels = PathFinder.VoxelMap()

                if 'voxels' in data['params']:
                    data_voxels = data['params']['voxels']
                    voxels.offset = data_voxels['offset']
                    voxels.scale = data_voxels['scale']
                    voxels.grid = data_voxels['grid']

                self.__voxels = voxels

                if self.__path != path:
                    self.__path = path
                    self.__path_changed_handler(path, voxels)
