#!python
"""
DiffDrive robot example
"""

from PiStorms import PiStorms
import bot
import requests
import time

psm = PiStorms()

desc = bot.DiffDescription(
    psm.BAM1,
    psm.BBM1,
    axel_radius=20.47875/100/2,
    wheel_radius=3.6/100/2,
    reverse_forward=True
)

driver = bot.DiffDriver(desc)
#locator = bot.DiffLocator(desc)
locator = bot.RemoteLocator()

path_follower = bot.PathFollower(driver, locator)


def path_changed_handler(path, voxels):
    path_follower.path = path

path_finder = bot.RemotePathFinder(path_changed_handler)

remote = bot.RemoteControl(
    bot_driver=driver,
    title='Minion Remote',
    locator=locator,
    path_finder=path_finder,
    path_follower=path_follower)

server = bot.WebServer(9000, screen=psm.screen, plugins=(
    remote,
    locator,
    bot.ConfigServer(),
    path_finder,
))
server.run(async=True)


class FrontBumperProcessor(bot.EventLoopProcessor):
    def __init__(self, driver):
        self.__driver = driver

    def update(self):
        # if the front bumper hits something while driving, stop everything and drive backwards 10 cm
        if (psm.BAS1.isTouchedNXT() or psm.BBS1.isTouchedNXT()) \
                and (desc.left_motor.isBusy() or desc.right_motor.isBusy()):
            self.__driver.float()
            self.__driver.drive(-30, 0.10, wait=True)


def switchboard_register():
    registered = False

    try:
        resp = requests.get('http://gabey-dev:9000/register/robot')
        if resp and resp.status_code == 200:
            registered = True
    except:
        pass

    if registered:
        psm.screen.termPrintln('registered successfully')
    else:
        psm.screen.termPrintln('failed to register')

#switchboard_register()


print 'starting event loop'

event_loop = bot.EventLoop(processors=(
    FrontBumperProcessor(driver),
    remote,
    path_follower,
))

while True:
    if psm.isKeyPressed():
        break
    event_loop.step()
    time.sleep(0.001)

driver.float()

psm.screen.termPrintln("Exiting ...")