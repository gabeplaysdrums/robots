#!python
"""
DiffDrive robot example
"""

from PiStorms import PiStorms
import bot
import time

psm = PiStorms()

desc = bot.DiffDescription(
    psm.BAM1,
    psm.BBM1,
    axel_radius=15.24/100/2,
    wheel_radius=4.28625/100/2,
    reverse_forward=True
)

driver = bot.DiffDriver(desc)

"""
# drive in a square
for i in range(4):
    d.drive(power=50, distance=0.75)
    d.stop()
    d.turn_in_place(power=15, degrees=90)
    d.stop()
"""

#locator = bot.DiffLocator(desc)
locator = bot.RemoteLocator()

path_follower = bot.PathFollower(driver, locator)

remote = bot.WebRemoteServer(driver, title='Minion Remote', locator=locator, path_follower=path_follower, screen=psm.screen)
remote.run(async=True)

done = False

while not done:
    time.sleep(1)
    if psm.isKeyPressed():
        done = True

driver.float()

psm.screen.termPrintln("Exiting ...")