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
            self.__desc.left_motor.runDegs(self.__desc.forward_coeff * motor_degrees, self.__power_to_speed(power), True, True)
            self.__desc.right_motor.runDegs(self.__desc.forward_coeff * -motor_degrees, self.__power_to_speed(power), True, True)

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


from flask import Flask, render_template, request
from flask.views import View
from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer
import json
import threading


class WebRemoteServer:
    """
    Runs a web server that you can use to remote-control the robot

    Requires [Flask](http://flask.pocoo.org/) and [gevent-websocket](https://bitbucket.org/noppo/gevent-websocket).

    :param bot_driver: `BotDriver` instance
    :param port: server port
    :param title: page title
    """

    def __init__(self, bot_driver, port=9000, title='Robot Remote', locator=None, path_follower=None, screen=None):
        self.__title = title
        self.__port = port
        self.__screen = screen

        self.__app = Flask(__name__)
        self.__app.add_url_rule('/', view_func=WebRemoteServer.IndexView.as_view('index', title=title))
        self.__app.add_url_rule('/api', view_func=WebRemoteServer.ApiView.as_view(
            'api',
            bot_driver=bot_driver,
            locator=locator,
            path_follower=path_follower))

    def run(self, async=False):
        """
        Run the web server

        :param async: if True, server will start on a separate thread
        """

        def run_server(port, app, screen):
            #self.app.run(host='0.0.0.0', port=self.__port, debug=True)
            http_server = WSGIServer(('', port), app, handler_class=WebSocketHandler)

            if screen:
                # get public IP address
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('microsoft.com', 80))
                ip_address = s.getsockname()[0]
                s.close()

                screen.termPrintln('Server started.')
                screen.termPrintln('Connect at:')
                screen.termPrintln('http://%s:%d' % (ip_address, port))
                screen.termPrintln(' ')

            print 'Server started on port %d' % (port,)
            http_server.serve_forever()

        server_thread = threading.Thread(target=run_server, args=(self.__port, self.__app, self.__screen))
        server_thread.daemon = True
        server_thread.start()

        if not async:
            server_thread.join()

    class IndexView(View):
        def __init__(self, title):
            self.__title = title

        def dispatch_request(self):
            print 'rendering index template'
            return render_template('index.html', title=self.__title)

    class ApiView(View):
        def __init__(self, bot_driver, locator, path_follower):
            self.__bot_driver = bot_driver
            self.__locator = locator
            self.__path_follower = path_follower

        def dispatch_request(self):
            print 'setting up web socket'
            if request.environ.get('wsgi.websocket'):
                ws = request.environ['wsgi.websocket']

                if self.__locator is not None:

                    def locator_thread_func(ws, locator):
                        while True:
                            time.sleep(0.250)

                            (pose, confidence) = locator.locate()

                            ws.send(json.dumps({
                                'action': 'locator_update',
                                'pose': pose,
                                'confidence': confidence
                            }))

                            if self.__path_follower is not None:
                                ws.send(json.dumps({
                                    'action': 'path_update',
                                    'path': self.__path_follower.get_path()
                                }))

                    locator_thread = threading.Thread(target=locator_thread_func, args=(ws, self.__locator))
                    locator_thread.daemon = True
                    locator_thread.start()

                while True:
                    message = ws.receive()
                    #print 'ws message receieved:', message
                    data = json.loads(message)

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
                            self.__path_follower.set_path(data['params']['path'])


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


class DiffLocator(Locator):
    """
    Attempts to estimate current pose of a differential drive robot by monitoring motor tachometers

    :param desc: `DiffDescription` instance
    """

    def __init__(self, desc):
        self.__desc = desc
        self.__pose = (0, 0, 0)
        self.__confidence = 1
        self.__left_pos = desc.left_pos()
        self.__right_pos = desc.right_pos()
        self.__lock = threading.Lock()

        self_weak = weakref.ref(self)
        worker = threading.Thread(target=DiffLocator.__thread_func, args=(self_weak,))
        worker.daemon = True
        worker.start()

    def locate(self):
        with self.__lock:
            return self.__pose, self.__confidence

    @staticmethod
    def __thread_func(self_weak):

        while True:
            self = self_weak()

            if self is None:
                break

            # do work

            # get previous values
            with self.__lock:
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
                    turn_radius = self.__desc.axel_radius * float(right_pos_delta + left_pos_delta) / (right_pos_delta - left_pos_delta)
                    left_pos_delta_rad = DEG_TO_RAD * left_pos_delta
                    right_pos_delta_rad = DEG_TO_RAD * right_pos_delta
                    if turn_radius == self.__desc.axel_radius:
                        theta_delta_rad = self.__desc.wheel_radius * right_pos_delta_rad / (turn_radius + self.__desc.axel_radius)
                    else:
                        theta_delta_rad = self.__desc.wheel_radius * left_pos_delta_rad / (turn_radius - self.__desc.axel_radius)
                    x_local = turn_radius * math.sin(theta_delta_rad)
                    y_local = turn_radius - turn_radius * math.cos(theta_delta_rad)

                def matmult(a,b):
                    zip_b = zip(*b)
                    # uncomment next line if python 3 :
                    # zip_b = list(zip_b)
                    return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b))
                             for col_b in zip_b] for row_a in a]

                prev_theta_rad = DEG_TO_RAD * prev_theta

                v_local = [ [x_local], [y_local], [1.0] ]
                m_rotate = [
                    [ math.cos(prev_theta_rad), math.sin(prev_theta_rad), 0 ],
                    [ -math.sin(prev_theta_rad), math.cos(prev_theta_rad), 0 ],
                    [ 0, 0, 1.0 ]
                ]
                m_translate = [
                    [ 1.0, 0, prev_x ],
                    [ 0, 1.0, prev_y ],
                    [ 0, 0, 1.0 ]
                ]

                v_world = matmult(m_translate, matmult(m_rotate, v_local))

                theta = prev_theta + RAD_TO_DEG * theta_delta_rad
                x = v_world[0][0]
                y = v_world[1][0]

                confidence = prev_confidence * 0.8

                with self.__lock:
                    self.__pose = (x, y, theta)
                    self.__confidence = confidence
                    self.__left_pos = left_pos
                    self.__right_pos = right_pos

            del self
            time.sleep(0.100)


class RemoteLocator(Locator):
    """
    Gets current location from a remote device using a web server
    """

    def __init__(self, port=9001):
        self.__port = port
        self.__app = Flask(__name__)
        self.__lock = threading.Lock()
        self.__pose = (0, 0, 0)

        def set_pose(pose):
            with self.__lock:
                self.__pose = pose

        self.__app.add_url_rule('/api', view_func=RemoteLocator.ApiView.as_view('api', set_pose=set_pose))

        def server_thread_func(port, app):
            http_server = WSGIServer(('', port), app, handler_class=WebSocketHandler)
            print 'Started remote locator server on port %d' % (port,)
            http_server.serve_forever()

        server_thread = threading.Thread(target=server_thread_func, args=(self.__port, self.__app))
        server_thread.daemon = True
        server_thread.start()

    def locate(self):
        with self.__lock:
            return self.__pose, 1

    class ApiView(View):
        def __init__(self, set_pose):
            self.__set_pose = set_pose

        def dispatch_request(self):
            print 'setting up web socket'
            if request.environ.get('wsgi.websocket'):
                ws = request.environ['wsgi.websocket']

                try:
                    while True:
                        message = ws.receive()
                        #print 'RemoteLocator message:', message

                        if not message:
                            continue

                        data = json.loads(message)
                        action = data['action']

                        if action == 'set_pose':
                            self.__set_pose(data['pose'])
                except Exception, e:
                    print 'ERROR:', e.message


class PathFollower:
    """
    Drives the robot along the given path

    :param bot_driver: drive control for the robot
    :param locator: locator that determines the robot's pose
    :param distance_tolerance: minimum distance from the target location
    """

    def __init__(self, bot_driver, locator, distance_tolerance=0.05, heading_tolerance=10, forward_power=50, turn_power=30):
        self.__bot_driver = bot_driver
        self.__locator = locator
        self.__distance_tolerance = distance_tolerance
        self.__heading_tolerance = heading_tolerance
        self.__forward_power = forward_power
        self.__turn_power = turn_power
        self.__path = []
        self.__lock = threading.Lock()

        self_weak = weakref.ref(self)

        thread = threading.Thread(target=PathFollower.__update_thread_func, args=(self_weak,))
        thread.daemon = True
        thread.start()

    def set_path(self, path):
        """
        Set target path
        :param path: list of (x, y) pairs that the robot should drive to in order
        """
        with self.__lock:
            print 'new path: ', path
            self.__path = path

    def clear_path(self):
        with self.__lock:
            self.__path = []

    def get_path(self):
        with self.__lock:
            return self.__path[:]

    @staticmethod
    def __update_thread_func(self_weak):
        while True:
            self = self_weak()
            with self.__lock:
                if self.__path:
                    (x_dest, y_dest) = self.__path[0]
                    (x, y, theta), confidence = self.__locator.locate()
                    dx = x_dest - x
                    dy = y_dest - y

                    dist = math.sqrt(dx*dx + dy*dy)

                    if dist < self.__distance_tolerance:
                        print 'destination reached: ', (x_dest, y_dest)
                        self.__bot_driver.float()
                        time.sleep(0.250)
                        del self.__path[0]
                        continue

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