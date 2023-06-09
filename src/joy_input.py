from inputs import get_gamepad
import math
import threading

from joystick_input.msg import JoyStickOut

class JoyStickController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self.msg = JoyStickOut
        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(
        self,
    ):   # return the buttons/triggers that you care about in this methode
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.X   # b=1, x=2
        rb = self.RightBumper
        lb = self.LeftBumper

        rt = self.RightTrigger
        lt = self.LeftTrigger

        return [x, y, a, b, rb, rt]
    
    def read_ros(self):
        
        self.msg.triangle = self.Y
        self.msg.circle = self.B
        self.msg.cross = self.A
        self.msg.square = self.X

        self.msg.d_down = self.DownDPad
        self.msg.d_up = self.UpDPad
        self.msg.d_right = self.RightDPad
        self.msg.d_left = self.LeftDPad

        self.msg.right_trigger=self.RightTrigger
        self.msg.right_bumper =self.RightBumper
        self.msg.left_trigger=self.LeftTrigger
        self.msg.left_bumper=self.LeftBumper

        self.msg.start = self.Start
        self.msg.select = self.Back

        self.msg.left_joy_x=self.LeftJoystickX
        self.msg.left_joy_y=self.LeftJoystickY
        self.msg.left_joy_in=self.LeftThumb

        self.msg.right_joy_x=self.RightJoystickX
        self.msg.right_joy_y=self.RightJoystickY
        self.msg.right_joy_in=self.RightThumb



    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = (
                        event.state / JoyStickController.MAX_JOY_VAL
                    )   # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = (
                        event.state / JoyStickController.MAX_JOY_VAL
                    )   # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = (
                        event.state / JoyStickController.MAX_JOY_VAL
                    )   # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = (
                        event.state / JoyStickController.MAX_JOY_VAL
                    )   # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = (
                        event.state / JoyStickController.MAX_TRIG_VAL
                    )   # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = (
                        event.state /JoyStickController.MAX_TRIG_VAL
                    )   # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state   # previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state   # previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state


if __name__ == '__main__':
    joy = JoyStickController()
    while True:
        print(joy.read())
