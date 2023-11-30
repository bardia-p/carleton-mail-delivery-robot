#!/usr/bin/env python3

import turtle
import math

# Initialize turtle.
screen = turtle.Screen()
screen.title("Robot Simulator")
screen.screensize(800, 800)

# Create a turtle object for the robot
robot = turtle.Turtle()
robot.shape("turtle")
robot.radians()

ROBOT_SPEED = 0.4
SET_POINT = 0.5
AIM_ANGLE = 60
ERROR = ROBOT_SPEED * math.sin(AIM_ANGLE * math.pi / 180.0) / 2.0

ROBOT_INITIAL_DISTANCE = 3.0
ROBOT_INITIAL_ANGLE = 30

def move_robot(linear_velocity, angle):
    '''
    Function to move the robot based on the twist command.

    @param linear_velocity: the speed of the robot.
    @param angle: the robot's angle with the wall.
    '''
    # Move the robot
    print("OUTPUT: ANGLE:" + str(angle * 180.0 / math.pi))
    robot.right(-1 * angle)
    robot.forward(linear_velocity * 100)


def get_new_angle(cur_distance, cur_angle):
    '''
    Gets a new function for the robot based on its current distance and angle with the wall.
    This function was copied to contorl/action_translator.py

    @param cur_distance: robot's current distance with the wall.
    @param cur_angle: robot's current angle with the wall.
    '''
    if cur_angle > 180:
        cur_angle -= 360

    res_angle = 0
    print("INPUT: DISTANCE:" + str(cur_distance) + " ANGLE:" + str(cur_angle))
    if cur_distance > SET_POINT + ERROR:
        res_angle = -1 * AIM_ANGLE + cur_angle
    elif cur_distance < SET_POINT - ERROR:
        res_angle = AIM_ANGLE + cur_angle
    else:
        res_angle = cur_angle
    
    return res_angle * math.pi / 180.0

def init():
    '''
    Initializes the robot.
    '''
    # Draw the wall, the set point, and place the robot.
    robot.penup()
    robot.color("blue")
    robot.goto(-400,0)
    robot.pendown()
    robot.forward(800) 
    robot.penup()
    robot.color("red")
    robot.goto(-400, SET_POINT * 100)
    robot.pendown()
    robot.forward(800) 
    robot.penup()
    robot.color("black")
    robot.goto(-300, ROBOT_INITIAL_DISTANCE * 100)
    robot.right(ROBOT_INITIAL_ANGLE * math.pi / 180.0)
    robot.pendown()

init()
# Main simulation loop
while True:
    move_robot(ROBOT_SPEED, get_new_angle(robot.position()[1] / 100, (2 * math.pi - robot.heading()) * 180.0 / math.pi))
# Close the turtle graphics window when done
turtle.done()
