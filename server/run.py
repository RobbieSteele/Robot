from nis import match
from tracemalloc import start
from turtle import Turtle, right, speed
from unittest import case
from RPIservo import ServoCtrl
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

swivel = Action(
    speed = 30, 

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
actions = [
    # Action(
    #     t="rotate_servo",
    #     servo_ids=[5],
    #     angle=0
    # ),
    # Action(
    #     t="move",
    #     direction="backward",
    #     speed=60,
    #     stops=[Stop(t="duration", duration=0.5)]
    # ),
    # Action(
    #     t="delay",
    #     stops=[Stop(t="duration", duration=0.5)]
    # ),
    # Action(
    #     t="turn",
    #     direction="opposites",
    #     turn_direction="right",
    #     radius=1,
    #     speed=60,
    #     stops=[Stop(t="duration", duration=0.8)]
    # ),
    # Action(
    #     t="rotate_servo",
    #     servo_ids=[1,2,3,4,5],
    #     angle=90
    # ),
    # Action(
    #     t="line",
    #     turn_speed=60,
    #     speed=75,
    #     stops=[Stop(t="off_line")]
    # ),
    # Action(
    #     t="move",
    #     direction="backward",
    #     speed=60,
    #     stops=[Stop(t="duration", duration=1)]
    # ),
    # Action(
    #     t="turn",
    #     direction="opposites",
    #     turn_direction="right",
    #     radius=0.25,
    #     speed=75,
    #     stops=[Stop(t="duration", duration=1)]
    # ),
    # Action(
    #     t="move",
    #     direction="backward",
    #     speed=60,
    #     stops=[Stop(t="duration", duration=4)]
    # ),
    # Action(
    #     t="turn",
    #     direction="opposites",
    #     turn_direction="right",
    #     radius=1,
    #     speed=60,
    #     stops=[Stop(t="duration", duration=1.5)]
    # ),
    #  Action(
    #     t="move",
    #     direction="backward",
    #     speed=60,
    #     stops=[Stop(t="duration", duration=4)]
    # ),
    # Action(
    #     t="rotate_servo",
    #     servo_ids=[1,2,3,4,5],
    #     angle=90
    # ),
    #   Action(
    #     t="rotate_servo",
    #     servo_ids=[1,2,3,4,5],
    #     angle=45
    # ),
    # Action(
    #     t="turn",
    #     direction="opposites",
    #     turn_direction="left",
    #     radius=1,
    #     speed=60,
    #     stops=[Stop(t="duration", duration=1.5)]
    # ),
    #  Action(
    #     t="move",
    #     direction="backward",
    #     speed=60,
    #     stops=[Stop(t="duration", duration=1)]
    # ),
    # Action(
    #     t="delay",
    #     stop=[Stop(t="duration", duration=1)]
    # ),
    # Action(
    #     t="rotate_servo",
    #     servo_ids=[3, 5],
    #     angle=-45
    # ),
]


class Robot:
    def __init__(self):
        self.sc = ServoCtrl()
        self.sc.start()

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
        self.actions = actions

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
            #print(f"{left_line_sensor} {middle_line_sensor} {right_line_sensor}")
            if middle_line_sensor == 0:
                move.move(action.speed, 'forward', 'both')
            elif left_line_sensor == 1:
                move.move(action.turn_speed, 'forward', 'right', 0.5)
            elif right_line_sensor == 1:
                move.move(action.turn_speed, 'forward', 'left', 0.5)
        elif action.t == "move":
            move.move(action.speed,
                      action.direction,
                      "both",
                      1)
        elif action.t == "turn":
            move.move(action.speed,
                      action.direction,
                      action.turn_direction,
                      action.radius if hasattr(action, "radius") else 1)
        elif action.t == "rotate_servo":
            all_finished = True
            for servo_id in action.servo_ids:
                all_finished &= self.sc.moveAngle(servo_id, action.angle)

            if all_finished:
                print("Finished servo")
                self.actions.pop(0)
        elif action.t == "repeat":
            if hasattr(action, "repeat_count"):
                action.repeat_count += 1
            else:
                action.repeat_count = 1

            if action.repeat_count <= action.times:
                self.actions = action.actions.copy() + self.actions
            else:
                self.actions.pop(0)

        # early stopping
        # added check to see make sure action wasn't removed above and is still the current action
        if hasattr(action, "stops") and self.actions[0] == action:
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
                        print("STARTED TIMER")
                        current_stop.start_time = time.time()
                    if time.time() - current_stop.start_time > current_stop.duration:
                        active = True
                if active:
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
    robot.destroy()
    exit()
