from nis import match
from turtle import Turtle, right, speed
from unittest import case
import RPi.GPIO as GPIO
import time
import move

'''
TODO:


Update move.py file - for radius to work, both motors need to fire
Change direction 'no' to 'both'
'''


'''

actions:
    types:
        line - on line following
        move - both motors forward/backwards
        turn - one or both motors right/left (can be either forward/backwards/ or both)
    
    action_attributes:
        direction - "forward/backward/opposites"
        turn_direction - "right/left/both"
        radius - How much to weaken the opposite motor when turning. 0 is running 1 motor, 1 is runnin both at full speed. .5
                 is running direction at full and opposite at speed*.5
        speed - how much force the motor should use (0 - 100; However double motor minimum is probably 50, 75 is probably for single) only required for single turn
        stop - TODO - gyroscope angle traveled, distance traveled, OFF_LINE, ON_LINE, duration, distance remaining to object
    
'''


class Action:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class Stop:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


main_line_off_line_stop = Stop(
    t="off_line"
)

line = Action(
    t="line",
    speed=50,
    turn_speed=60,
    stops=[Stop(t="duration", duration=7)]
)

p = Action(
    t="empty",
    stops=[Stop(t="duration", duration=1)]
)

backup = Action(
    t="move",
    direction="backward",
    speed=60,
    stops=[Stop(t="duration", duration=0.25)]
)

turn = Action(
    t="turn",
    direction="opposites",
    turn_direction="right",
    radius=1,
    speed=60,
    stops=[Stop(t="duration", duration=.8)]
)

forward = Action(
    t="move",
    direction="forward",
    speed=75,
    stops=[Stop(t="duration", duration=.6)]
)

turn_around = Action(
    t="turn",
    direction="opposites",
    turn_direction="right",
    radius=1,
    speed=60,
    stops=[Stop(t="duration", duration=2)]
)

second_line = Action(
    t="line",
)



#main_line_off_line_stop.action = turn_action
#turn_stop.action = main_line_action

class Robot:
    def __init__(self):
        self.line_pin_right = 19
        self.line_pin_middle = 16
        self.line_pin_left = 20

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.line_pin_right, GPIO.IN)
        GPIO.setup(self.line_pin_middle, GPIO.IN)
        GPIO.setup(self.line_pin_left, GPIO.IN)

        move.setup()

        self.running = True
        self.actions = [forward, p, turn, p, line, p , backup, p, turn_around, p, line]

    def start(self):
        while self.running:
            self.run()

    def run(self):
        right_line_sensor = GPIO.input(self.line_pin_right)
        middle_line_sensor = GPIO.input(self.line_pin_middle)
        left_line_sensor = GPIO.input(self.line_pin_left)

        if len(self.actions) == 0:
            self.running = False
            return

        action = self.actions[0]

        if not hasattr(action, "t"):
            print(
                "WARNING: action doesn't have type. Removing action")
            self.actions.pop(0)

        # run action (s)
        # check for early stoppings

        if action.t == "line":
            print(f"{left_line_sensor} {middle_line_sensor} {right_line_sensor}")
            if middle_line_sensor == 0:
                move.move(action.speed, 'forward', 'both')
            elif left_line_sensor == 1:
                move.move(action.turn_speed, 'forward', 'right', 0.5)
            elif right_line_sensor == 1:
                move.move(action.turn_speed, 'forward', 'left', 0.5)

        if action.t == "move":
            move.move(action.speed,
                      action.direction,
                      "both",
                      1)

        if action.t == "turn":
            move.move(action.speed,
                      action.direction,
                      action.turn_direction,
                      action.radius if hasattr(action, "radius") else 1)

        # early stopping
        if hasattr(action, "stops"):
            for current_stop in action.stops:  # iterate over copy so we can remove
                active = False
                if current_stop.t == "on_line":
                    if left_line_sensor + middle_line_sensor + right_line_sensor <= 2:
                        active = True
                if current_stop.t == "off_line":
                    if left_line_sensor and middle_line_sensor and right_line_sensor:
                        active = True
                if current_stop.t == "duration":
                    if not hasattr(current_stop, "start_time"):
                        current_stop.start_time = time.time()
                    if time.time() - current_stop.start_time > current_stop.duration:
                        active = True
                if active:
                    print("STOP HIT")
                    move.motorStop()
                    self.actions.pop(0)
                    if hasattr(current_stop, "start_time"):
                        delattr(current_stop, "start_time")
                    if hasattr(current_stop, "action"):
                        self.actions.insert(0, current_stop.action)

    def destroy(self):
        move.destroy()


if __name__ == '__main__':
    robot = Robot()
    try:
        robot.start()
    except KeyboardInterrupt:
        robot.destroy()
