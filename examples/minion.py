#!python
"""
DiffDrive robot example
"""

from PiStorms import PiStorms
import bot

psm = PiStorms()

d = bot.DiffDrive(psm.BAM1, psm.BBM1, axel_radius=15.24/100/2, wheel_radius=4.28625/100/2, reverse_forward=True)

# drive in a square
for i in range(4):
    d.drive(power=50, distance=0.75)
    d.stop()
    d.turn_in_place(power=15, degrees=90)
    d.stop()