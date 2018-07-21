#!/usr/bin/python
###########################################################
#
# ik-range-plot.py
#
#   Test and plot arm reach based on the Inverse Kinematics class
#
#   July 7, 2018
#
###########################################################

import oarmik as ik
import numpy as np
import matplotlib.pyplot as plt

positions = []

model = ik.OARMIK()

#
# Visualize arm's gripper-end accessible region
#
for x in range(50,300,2):
    for y in range(-200,200,5):
        positions = model.get_positions(x,y,0)
        if positions[3]:
            plt.scatter(x, y, c="r")
        else:
            plt.scatter(x, y, c="b")

plt.show()

