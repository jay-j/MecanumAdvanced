#!/usr/bin/env python3
# mecanum control code
# Jay Jasper, 2019
# Key features
#      boost mode: choose between maximum speed and best joystick linearity
#      user-centric mode: if provided a robot heading, can give commands in world frame (vs. local robot frame)
#      frame selection: command robot rotation movement about arbitrary points (e.g. the end effector)
from math import sqrt, sin, cos

########## ROBOT PARAMETERS ##########
# coordinate system is standard; +x forward, +y right, +z down
# so a "right turn" is positive z rotation

# wheel locations #TODO robot geometry
wheel_x = 0.18 # m
wheel_y = 0.22 # m

wheel_loc = []
wheel_loc.append([ wheel_x, -wheel_y])
wheel_loc.append([ wheel_x,  wheel_y])
wheel_loc.append([-wheel_x, -wheel_y])
wheel_loc.append([-wheel_x,  wheel_y])

# which direction does the wheel push the robot when spun forwards?
c45 = sqrt(2.0)/2.0
wheel_push = []
wheel_push.append([c45, c45])  # 1 - front left
wheel_push.append([c45, -c45]) # 2 - front right
wheel_push.append([c45, -c45]) # 3 - rear left
wheel_push.append([c45, c45])  # 4 - rear right

########## JOYSTICK ##########
# joystick y is going to typically correspond to "forwards" to make that mapping here
# ASSUME joystick inputs are -1.0 to +1.0
joystick_fwd = 0.20
joystick_side = 0.20
joystick_spin = -0.5

# TODO update for robot stats
max_wheel_speed = 0.4 # maximum wheel tangent speed, meter/second

# set max rotate rate to be all out with full joystick input
max_rot = sqrt(2)*max_wheel_speed / sqrt(wheel_loc[0][0]**2 + wheel_loc[0][1]**2)
print("max:", max_wheel_speed, max_rot)

desired_v_global = [max_wheel_speed*joystick_fwd, max_wheel_speed*joystick_side, max_rot*joystick_spin]

# TODO boost mode button
# using boost mode achieves maximum speed in all directions but creates some joystick nonlinearities
# recommend to have a (held) button for this since precision manuvering will be easier with boost off 
boost_mode = True

# TODO button to change between user centric and robot centric drive modes
# TODO use some sort of held joystick button to change IC mode?
block_mode = False

########## USER CENTRIC MODE ##########
# [local] = R*[global]
# TODO get the desired angle from sensors. (or maybe also a simple "flip the robot" button press?)
th = 0
desired_v = desired_v_global
desired_v[0] = desired_v_global[0]*cos(th) + desired_v_global[1]*sin(th)
desired_v[1] = -desired_v_global[0]*sin(th) + desired_v_global[1]*cos(th)
print("desired_v after rotation:", desired_v)

########## DESIRED VELOCITY FRAME ##########
# do this after the user-centric mode transform since it is assumed the desired IC is always in a local robot frame
# for normal driving, just leave the desired velocity frame at the center!
desired_frame = [0.0, 0.0]

# TODO for driving about the block, move the desired velocity frame to the block location
if block_mode:
    desired_frame = [1.5*wheel_x, 0.0]
print("desired frame:", desired_frame)

# TODO add a little filter (alpha...fine...) to help smooth discontinuities when changing frames

########## GET EACH WHEEL FRAME ##########
# desired velocity of each wheel
w = [0.0, 0.0, 0.0, 0.0]

for i in range(4):
    # get local velocity at the wheel. translation plus cross product
    v_local = desired_v[0:2]
    v_local[0] += -desired_v[2]*(wheel_loc[i][1] - desired_frame[1])
    v_local[1] +=  desired_v[2]*(wheel_loc[i][0] - desired_frame[0]) 
    print("v_local:", v_local)

    # dot product to pull out wheel contribution
    w[i] = wheel_push[i][0]*v_local[0] + wheel_push[i][1]*v_local[1]

# desired wheel speeds in m/s
print("w:", w)

########## SPEED SCALINGS ##########
if boost_mode:
    print("boost mode: ON")
    # a nonlinear mapping to maximize speed    
    # multiply wheel speeds so the maximum magnitude is 1
    # then scale by the effort percentage

    # ASSUME joystick inputs are -1.0 to +1.0
    effort_current = sqrt(joystick_fwd**2 + joystick_side**2 + joystick_spin**2)

    # get max possible length of the joystick vector
    limit_den = max([ abs(joystick_fwd), abs(joystick_side), abs(joystick_spin) ])
    effort_max = sqrt((joystick_fwd / limit_den)**2 + (joystick_side / limit_den)**2 + (joystick_spin / limit_den)**2 )

    # scale desired twist by this - so the joystick at the edge always gets *some* motor to full velocity
    effort_lvl = effort_current / effort_max
    print(effort_current, "of", effort_max, "  :  ", effort_lvl)

    w_max = max([abs(w[0]), abs(w[1]), abs(w[2]), abs(w[3])])
    for i in range(4):
        w[i] *= max_wheel_speed * effort_lvl / w_max
else:
    print("boost mode: OFF")
    # linear mapping
    # reduced maximum speed 

    for i in range(4):
        w[i] /= sqrt(2)

print("w scaled:", w)

# TODO now convert the tangential wheel speeds [w] into whatever rad/s or encoder counts system you send to the motor controller


