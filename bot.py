#!python
"""
Common robot helpers
"""

import math
import time


class DiffDrive:
    """
    Differential drive

    :param left_motor: :class:`PiStorms.PiStormsMotor` instance for the motor on the left side of the robot
    :param right_motor: :class:`PiStorms.PiStormsMotor` instance for the motor on the right side of the robot
    :param axel_radius: half the distance between the two drive wheels in meters
    :param wheel_radius: radius of each of the two drive wheels in meters
    """

    def __init__(self, left_motor, right_motor, axel_radius=0.1, wheel_radius=0.05, reverse_forward=False):
        self.__left_motor = left_motor
        self.__right_motor = right_motor
        self.__axel_radius = axel_radius
        self.__wheel_radius = wheel_radius
        self.__forward_coeff = -1 if reverse_forward else 1

    def drive(self, power=30, turn_radius=0, distance=None, wait=True):
        """
        Drive the robot at the given speed and heading until :func:`stop` is called
        :param power: desired motor power in range [-100, 100]
        :param turn_radius: turn radius in meters.  0 => drive straight (no turning)
        :param distance: distance to drive in meters.  None => infinite distance
        """

        rad_to_deg = 180 / math.pi

        if turn_radius == 0:
            if distance is None:
                self.__left_motor.setSpeed(self.__power_to_speed(power))
                self.__right_motor.setSpeed(self.__power_to_speed(power))
            else:
                self.__left_motor.runDegs(int(self.__forward_coeff * rad_to_deg * distance / self.__wheel_radius),
                                          self.__power_to_speed(power),
                                          True,
                                          True)
                self.__right_motor.runDegs(int(self.__forward_coeff * rad_to_deg * distance / self.__wheel_radius),
                                           self.__power_to_speed(power),
                                           True,
                                           True)

                if wait:
                    self.__left_motor.waitUntilNotBusy()
                    self.__right_motor.waitUntilNotBusy()
            return

        if turn_radius < 0:
            inner_motor = self.__left_motor
            outer_motor = self.__right_motor
            turn_radius = -turn_radius
        else:
            inner_motor = self.__right_motor
            outer_motor = self.__left_motor

        outer_speed = (turn_radius + self.__axel_radius) * power / turn_radius
        inner_speed = (turn_radius - self.__axel_radius) * power / turn_radius

        scale = min(1.0, 100 / max(abs(outer_speed), abs(inner_speed)))

        outer_speed *= scale
        inner_speed *= scale

        if distance is None:
            outer_motor.setSpeed(outer_speed)
            inner_motor.setSpeed(inner_speed)
        else:
            outer_degs = self.__forward_coeff * rad_to_deg * (turn_radius + self.__axel_radius) * distance / \
                         (self.__wheel_radius * turn_radius)
            inner_degs = self.__forward_coeff * rad_to_deg * (turn_radius - self.__axel_radius) * distance / \
                         (self.__wheel_radius * turn_radius)
            outer_motor.runDegs(int(outer_degs), outer_speed, True, False)
            inner_motor.runDegs(int(inner_degs), inner_speed, True, False)

            if wait:
                self.__left_motor.waitUntilNotBusy()
                self.__right_motor.waitUntilNotBusy()

    def turn_in_place(self, power=30, degrees=None, wait=True):
        """
        Turn the robot in place
        :param power: desired motor power in range [-100, 100].  <0 => turn left, >0 => turn right
        :param degrees: number of degrees to rotate.  None => infinite degrees
        """
        if degrees is None:
            self.__left_motor.setSpeed(self.__power_to_speed(power))
            self.__right_motor.setSpeed(self.__power_to_speed(-power))
        else:
            motor_degrees = int(self.__axel_radius * degrees / self.__wheel_radius)
            self.__left_motor.runDegs(self.__forward_coeff * motor_degrees, self.__power_to_speed(power), True, True)
            self.__right_motor.runDegs(self.__forward_coeff * -motor_degrees, self.__power_to_speed(power), True, True)

            if wait:
                self.__left_motor.waitUntilNotBusy()
                self.__right_motor.waitUntilNotBusy()

    def stop(self):
        self.brake()
        time.sleep(0.250)
        self.float()

    def brake(self):
        """
        Brake all motors
        """
        self.__left_motor.brake()
        self.__right_motor.brake()

    def float(self):
        """
        Float all motors
        """
        self.__left_motor.float()
        self.__right_motor.float()

    def hold(self):
        """
        Float all motors
        """
        self.__left_motor.waitUntilNotBusy()
        self.__right_motor.waitUntilNotBusy()
        self.__left_motor.hold()
        self.__right_motor.hold()

    def __power_to_speed(self, power):
        power *= self.__forward_coeff
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

class WebRemoteServer:
    """
    Runs a web server that you can use to remote-control the robot

    Requires [Flask](http://flask.pocoo.org/) and [gevent-websocket](https://bitbucket.org/noppo/gevent-websocket).

    :param bot_driver: robot driver instance
    """

    def __init__(self, bot_driver, port=9000, title='Robot Remote'):
        self.__title = title
        self.__port = port
        self.__app = Flask(__name__)
        self.__app.add_url_rule('/', view_func=WebRemoteServer.IndexView.as_view('index', title=title))
        self.__app.add_url_rule('/api', view_func=WebRemoteServer.ApiView.as_view('api', bot_driver=bot_driver))

    def run(self):
        #self.app.run(host='0.0.0.0', port=self.__port, debug=True)
        print 'Server started on port %d' % (self.__port,)
        http_server = WSGIServer(('', self.__port), self.__app, handler_class=WebSocketHandler)
        http_server.serve_forever()

    class IndexView(View):
        def __init__(self, title):
            self.__title = title

        def dispatch_request(self):
            print 'rendering index template'
            return render_template('index.html', title=self.__title)

    class ApiView(View):
        def __init__(self, bot_driver):
            self.__bot_driver = bot_driver

        def dispatch_request(self):
            print 'setting up web socket'
            if request.environ.get('wsgi.websocket'):
                ws = request.environ['wsgi.websocket']
                while True:
                    message = ws.receive()
                    print 'ws message receieved:', message
                    data = json.loads(message)

                    if not 'action' in data:
                        continue

                    action = data['action']

                    if action == 'drive':
                        self.__bot_driver.drive(power=data['params']['power'], wait=False)
                    elif action == 'turn_in_place':
                        self.__bot_driver.turn_in_place(power=data['params']['power'], wait=False)
                    elif action == 'stop':
                        self.__bot_driver.stop()
                    elif action == 'float':
                        self.__bot_driver.float()