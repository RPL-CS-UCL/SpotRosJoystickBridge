#!/usr/bin/env python3
import sys
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

from joystick_input.msg import JoyStickOut
# import std_srvs.srv
def val_clamp( n, clamp_range):
    minn = clamp_range[0]
    maxn = clamp_range[1]
    return max(min(maxn, n), minn)

def array_clamp(n_arr, clamp_range):
    for i in range(len(n_arr)):
        n_arr[i] = val_clamp(n_arr[i], clamp_range)   
    return n_arr

class SpotBodyPose:

    # Body pose limits
    BODY_HEIGHT_LIMITS = (-0.1, 0.1)
    ROLL_LIMITS = (-20.0, 20.0)
    PITCH_LIMITS = (-30.0, 30.0)
    YAW_LIMITS = (-30.0, 30.0)

    def __init__(self) -> None:
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def clamp(self, n, clamp_range):
        minn = clamp_range[0]
        maxn = clamp_range[1]
        return max(min(maxn, n), minn)

    # HEIGHT 
    def set_height(self, height):
        self.z = self.clamp(height, SpotBodyPose.BODY_HEIGHT_LIMITS)
    def alter_height(self, change_amount):
        self.z += change_amount
        self.z = self.clamp(self.z, SpotBodyPose.BODY_HEIGHT_LIMITS)

    # roll 
    def set_roll(self, roll):
        self.roll = self.clamp(roll, SpotBodyPose.ROLL_LIMITS)
    def alter_roll(self, change_amount):
        self.roll += change_amount
        self.roll = self.clamp(self.roll, SpotBodyPose.ROLL_LIMITS)
        
    # pitch 
    def set_pitch(self, pitch):
        self.pitch= self.clamp(pitch, SpotBodyPose.PITCH_LIMITS)
    def alter_pitch(self, change_amount):
        self.pitch += change_amount
        self.pitch = self.clamp(self.pitch, SpotBodyPose.PITCH_LIMITS)
    
    # yaw 
    def set_yaw(self,yaw):
        self.yaw= self.clamp(yaw, SpotBodyPose.YAW_LIMITS)
    def alter_yaw(self, change_amount):
        self.yaw += change_amount
        self.yaw = self.clamp(self.yaw, SpotBodyPose.YAW_LIMITS)

class SpotBodyMovement:

    # Motion (velocity) limits
    MOTION_LIMITS = (0.0, 2.0)
    OBSTACLE_PADDING_LIMITS = (0.0, 20.0) 

    def __init__(self) -> None:
        self.linear = [0,0,0]
        self.angular = [0,0,0]


    def set_linear(self, x=0,y=0,z=0):
        self.linear = array_clamp([x,y,z], SpotBodyMovement.MOTION_LIMITS)
    def alter_linear(self, x=0,y=0,z=0):
        alter_list = [x,y,z]
        temp_linear_velocs = [self.linear[i] + alter_list[i] for i in range(len(alter_list))] 
        self.linear = array_clamp(temp_linear_velocs, SpotBodyMovement.MOTION_LIMITS)
        

    def set_angular(self, x=0,y=0,z=0):
        self.angular = array_clamp([x,y,z], SpotBodyMovement.MOTION_LIMITS)
    def alter_angular(self, x=0,y=0,z=0):
        alter_list = [x,y,z]
        temp_angular = [self.angular[i] + alter_list[i] for i in range(len(alter_list))] 
        self.angular = array_clamp(temp_angular, SpotBodyMovement.MOTION_LIMITS)

class SpotState:
    
    def __init__(self) -> None:
        self.has_lease = False
        self.motor_power_on = False

        self.is_sitting = True

        # puts spot in sit position
        self.gentle_e_stop_enabled = False

        # kills power to motors
        self.hard_e_stop_enabled = False

        # prvents any command until allow_motion called
        self.command_motion_blocking = False

        self.body_pose = SpotBodyPose()
        self.body_movement = SpotBodyMovement()


class InputHandler:
    

    
    def __init__(self) -> None:
        pass

    def convert_message_to_list(self, data):
        # data_list = []
        #
        # data.triangle
        # data.circle
        # data.cross
        # data.square
        #
        # data.d_up
        # data.d_right
        # data.d_down
        # data.d_left
        #
        # data_list.append(data.right_trigger
        # data_list.append(data.right_bumper
        # data_list.append(data.left_trigger
        # data_list.append(data.left_bumper
        #
        # data_list.append(data.start
        # data_list.append(data.select
        #
        # data_list.append(data.left_joy_x
        # data_list.append(data.left_joy_y
        # data_list.append(data.left_joy_in
        #
        #
        # data_list.append(data.right_joy_x
        # data_list.append(data.right_joy_y
        # right_joy_in
        pass


class SpotController:


    HELD_DURATION = 3.0
    def __init__(self) -> None:
        

        # init spot variables
        self.current_state = SpotState()

        self.init_serv_proxies()

        # init rospy node
        rospy.init_node("spot_joystick_bridge")
        self.joystick_reset_pub = rospy.Publisher('JoyStickReset',
                Bool,
                queue_size=10)
        rospy.Subscriber('JoyStickOut', JoyStickOut,self.get_joystick_input)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def get_joystick_input(self, data):
        # print("trying to read input")

        # ALWAYS READ FOR E STOPS FIRST
        if not self.current_state.hard_e_stop_enabled:
            if self.key_combo_pressed([data.left_bumper,data.left_trigger,data.right_bumper,data.right_trigger, data.d_up, data.triangle]):
                self.hard_e_stop_proxy()
                print("HARD E STOP PRESSED")
                self.current_state.hard_e_stop_enabled = True
                return
        else:

            if self.key_combo_pressed([data.left_bumper,data.left_trigger,data.right_bumper,data.right_trigger, data.d_up, data.triangle]):
                print("unlocking hard e stop")
                self.hard_e_stop_release_proxy()

                self.current_state.hard_e_stop_enabled = False

        # gentle e stop

        # if not self.current_state.gentle_e_stop_enabled:
        if self.key_combo_pressed([data.left_bumper,data.left_trigger,data.right_bumper,data.right_trigger ]):
            # self.current_state.gentle_e_stop_enabled = True
            # self.gentle_e_stop_proxy()
            self.current_state.is_sitting = True
            self.soft_e_stop_proxy()

        # stop all commands
        if self.key_combo_pressed([data.right_joy_in, data.left_joy_in]):
            print("STOPPING ALL COMMANDS")
            self.stop_command_proxy()





        
        # Handle inititilisation and registration inputs 
        if not self.handle_input_for_initialisation(data):
            return

        # We should have the lease and motor power.
        # print("We now have the lease and motors are powered on")

        # detecting a sitrequest
        if not self.current_state.is_sitting and self.key_combo_held([data.select_HeldDur,data.left_trigger_HeldDur]):
            print("Putting spot in sit")
            self.current_state.is_sitting = True
            _ = self.sit_proxy()

        # detect stand request
        if self.current_state.is_sitting and self.key_combo_held([data.select_HeldDur,data.left_bumper_HeldDur]):
            print("Putting spot in stand")
            self.current_state.is_sitting = False 
            _ = self.stand_proxy()
        # detect a mode change; static pose or movement mode

        # FROM HERE WE CAN JUST READ ANY INPUT NORMALLY

        # if we are in static pose handle input for that 


        # if we are in movement mode handle input for that


        # /spot/stop can be called at the end here as no input was found




    def handle_input_for_initialisation(self, data):
        
        #check if we have the lease before doing anything
        if not self.current_state.has_lease:
            
            if self.key_combo_held([data.select_HeldDur, data.start_HeldDur]):
                # claim lease and update state
                _ = self.obtain_lease_proxy()
                self.current_state.has_lease = True
                print("Obtaining spot lease")

                self.joystick_reset_pub.publish(True)

            #we have now either got the lease, need to retrieve new inputs so return false

            return False

        # if we get to here, we have control of the lease
        
        # Now check if we have motor power
        if not self.current_state.motor_power_on:
            if self.key_combo_held([data.start_HeldDur]):
                print("powering on motors")
                self.current_state.motor_power_on = True
                # power on motor and update state
                _ = self.power_on_proxy()
                
            # motors not requested power on, leave method
            else:
               return False
        return True

    # initialise all service proxies
    def init_serv_proxies(self):

        self.obtain_lease_proxy = rospy.ServiceProxy('/spot/claim', Trigger) 
        self.power_on_proxy = rospy.ServiceProxy('/spot/power_on', Trigger)

        self.sit_proxy = rospy.ServiceProxy('/spot/sit', Trigger)
        self.stand_proxy = rospy.ServiceProxy('/spot/stand', Trigger)


        # estops
        self.hard_e_stop_proxy = rospy.ServiceProxy('/spot/estop/hard', Trigger)
        self.hard_e_stop_release_proxy = rospy.ServiceProxy('/spot/estop/release', Trigger)

        self.soft_e_stop_proxy = rospy.ServiceProxy('/spot/estop/soft', Trigger)
    # Get/release control of spot
    # def obtain_lease(self):
    #     command = '/spot/claim'
    #     obtain_lease_proxy = rospy.ServiceProxy('/spot/claim', Trigger) 
    #     resp1 = obtain_lease_proxy()
        self.stop_command_proxy = rospy.ServiceProxy('/spot/stop', Trigger)


    def key_combo_pressed(self, keys_to_verify):
        for key in keys_to_verify:
            if not key:
                return False
        return True

    def key_combo_held(self,key_durs_to_verify):
        for key_dur in key_durs_to_verify:
            if key_dur < SpotController.HELD_DURATION:
                return False
        return True

    def release_lease(self):
        command = '/spot/release'

    # # Power on motors
    # def power_on(self):
    #     command = '/spot/power_on'

    # stand
    # def stand(self):
    #     command = '/spot/stand'
    #
    # def sit(self):
    #     command = '/spot/sit'

    # E stops
    # Gentle
    # def gentle_e_stop(self):
    #     command = '/spot/estop/gentle'

    # Hard
    # def hard_e_stop(self):
    #     command = '/spot/estop/hard'
    #
    # def release_hard_e_stop(self):
    #     command = '/spot/estop/release'

    # stop related commands
    def command_stop(self):
        command = '/spot/stop'

    def command_block(self):
        command = '/spot/locked_stop'

    def command_block_release(self):
        command = '/spot/allow_motion'

    # Self righting and roll over
    def self_right(self):
        command = '/spot/self_right'
        return command

    def roll_over(self, direction):
        command = None
        if direction == 'LEFT':
            command = '/spot/roll_over_left'
        if direction == 'RIGHT':

            command = '/spot/roll_over_right'
        return command


if __name__ == "__main__":
    spot_bridge = SpotController()

