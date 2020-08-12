#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
from casadi import *
import casadi as ca 

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self._vehicle_length     = 3

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    # def update_desired_speed(self):
    #     min_idx       = 0
    #     min_dist      = float("inf")
    #     desired_speed = 0
    #     for i in range(len(self._waypoints)):
    #         dist = np.linalg.norm(np.array([
    #                 self._waypoints[i][0] - self._current_x,
    #                 self._waypoints[i][1] - self._current_y]))
    #         if dist < min_dist:
    #             min_dist = dist
    #             min_idx = i
    #     if min_idx < len(self._waypoints)-1:
    #         desired_speed = self._waypoints[min_idx][2]
    #     else:
    #         desired_speed = self._waypoints[-1][2]
    #     self._desired_speed = desired_speed


    

    # def update_desired_speed(self):
    #     min_idx       = 0
    #     min_dist      = float("inf")
    #     desired_speed = 0
    #     x,y = self.front_axle_coord()
    #     for i in range(len(self._waypoints)):
    #         dist = np.linalg.norm(np.array([
    #                 self._waypoints[i][0] - x,
    #                 self._waypoints[i][1] - y]))
    #         if dist < min_dist:
    #             min_dist = dist
    #             min_idx = i
    #     if min_idx < len(self._waypoints)-1:
    #         desired_speed = self._waypoints[min_idx][2]
    #         self._min_idx = min_idx
    #     else:
    #         desired_speed = self._waypoints[-1][2]
    #         self._min_idx = -1
    #     self._desired_speed = desired_speed

    def front_axle_coord(self,x,y):
        x = self._current_x + self._vehicle_length*np.cos(self._current_yaw)/2
        y = self._current_y + self._vehicle_length*np.sin(self._current_yaw)/2

        return x,y

    # def get_min_index(self,x,y):
        
    #     min_dist = float("inf")
    #     min_idx  = 0

    #     for i in range(len(self._waypoints)):
    #         dist = np.linalg.norm(np.array([
    #                 self._waypoints[i][0] - x,
    #                 self._waypoints[i][1] - y]))
    #         if dist < min_dist:
    #             min_dist = dist
    #             min_idx = i
    #     print('min_idx',min_idx)
    #     return min_idx



    def get_target_speed(self,x,y):

        desired_speed = 0
        x,y = self.front_axle_coord(x,y)
        
        min_idx = self.get_min_index(x,y)

        if min_idx < len(self._waypoints)-1:
            target_speed = self._waypoints[min_idx][2]
            
        else:
            target_speed = self._waypoints[-1][2]
            
        return target_speed

    # def get_target_coord(self, min_idx):

    #     target_x = self._waypoints[min_idx][0]
    #     target_y = self._waypoints[min_idx][1]
    #     target_x_n = self._waypoints[min_idx+1][0]
    #     target_y_n = self._waypoints[min_idx+1][1]
        
    #     return target_x, target_y, target_x_n, target_y_n

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake


    # def get_cte(self,x,y):

    #     x, y = self.front_axle_coord(x, y)
    #     i = self.get_min_index(x, y)
    #     cte = np.sqrt((self._waypoints[i][1]-y)**2+(self._waypoints[i][0]-x)**2)
    #     return cte 


    # def get_head_err(self, x, y, yaw):

    #     x,y = self.front_axle_coord(x, y)
        
    #     min_idx = self.get_min_index(x, y)
    #     target_x, target_y, target_x_n, target_y_n = self.get_target_coord(min_idx)
 
    #     try:
    #         delta_y = target_y_n - target_y
    #         delta_x = target_x_n - target_x
    #     except:
    #         delta_x = 0
    #         delta_y = 0

    #     head = np.arctan2(delta_y, delta_x)
    #     delta = head - yaw
       
        

    #     # if delta > :
    #     #     delta = delta - self._2pi
    #     # if delta < -np.pi:
    #     #     delta = delta + self._2pi

    #     return delta


    def colloc_constraints(self, x_f, y_f, v_f, steering, yaw, throttle):

        delta_t = 0.05

        x_f_next = x_f + v_f*np.cos(yaw)*delta_t

        y_f_next = y_f + v_f*np.sin(yaw)*delta_t

        v_next = v_f + throttle*delta_t
        
        print(throttle.shape())
        yaw_next = yaw + v_f*steering/self._vehicle_length

        # head_err_c = self.get_head_err(x_c, y_c, yaw)

        # cte = self.get_cte(x_c, y_c)

        # cte_next = cte + v_c*np.sin(head_err_c-steering)*delta_t
        
        return x_f_next, y_f_next, yaw_next, v_next, 
    
    
    def mpc(self):
        
        N = 40
    	
        opti = ca.Opti()

        x = opti.variable(N)
        y = opti.variable(N)
        v = opti.variable(N)
        yaw = opti.variable(N)
        # head_err = opti.variable(N)
        steering = opti.variable(N)
        # cte = opti.variable(N)
        throttle = opti.variable(N)

        p_x = opti.parameter()  # parameter for the cross track error
        opti.set_value(p_x, 1)

        p_y = opti.parameter()   # parameter for the heading error
        opti.set_value(p_y, 1)

        p_steer = opti.parameter() # parameter for the steering
        opti.set_value(p_steer, 1)

        p_sr = opti.parameter() # parameter for the steering rate
        opti.set_value(p_sr, 1)
        
        p_vel = opti.parameter() # parameter for the target velocity
        opti.set_value(p_vel, 1)

        p_thr = opti.parameter() # parameter for the throttle
        opti.set_value(p_thr, 1)

        p_thr_r = opti.parameter() # parameter for the throttle rate 
        opti.set_value(p_thr_r, 1)

        
        v_target = []

        for i in range(0, N-1):

            x_next, y_next, yaw_next, v_next = self.colloc_constraints(x[i], y[i], v[i], \
                                                                     steering[i], yaw[i], throttle[i])

            opti.subject_to(x[i+1] == x_next)
            opti.subject_to(y[i+1] == y_next)
            opti.subject_to(v[i+1] == v_next)
            opti.subject_to(yaw[i+1] == yaw_next)
            opti.subject_to(cte[i+1] == cte_next)
            
            
            x_f, y_f = self.front_axle_coord(x[i], y[i])
            speed = self.get_target_speed(x_f, y_f)
            v_target.append(speed)
        
        
        x_f, y_f = self.front_axle_coord(x[i], y[i])
        speed = self.get_target_speed(x_f, y_f)
        v_target.append(speed)
        

        # cte_cost = sumsqr(p_cte*cte[:])
        # steer_cost = sumsqr(p_steer*steering[:])
        # steer_rate_cost = sumsqr(p_sr*(steering[1:N]-steering[0:N-1]))
        # velocity_cost = sumsqr(p_vel**(v_target-v))
        # throttle_cost = sumsqr(p_thr*(throttle[:]))
        # throttle_rate_cost = sumsqr(p_thr_r*(throttle[1:N]-throttle[0:N-1]))
        # head_err_cost = sumsqr(p_he*(head_err))
        

        # cost_function = cte_cost + steer_rate_cost + steer_cost + velocity_cost + throttle_cost + \
        #                                             throttle_rate_cost + head_err_cost   
        

        x_f_i, y_f_i = front_axle_coord(self._current_x, self._current_y)

        opti.subject_to(x[0] == x_f_i)
        opti.subject_to(y[0] == y_f_i)
        
        opti.subject_to(opti.bounded(-1.22, steering, 1.22))
        opti.subject_to(opti.bounded(0, throttle, 1.5))
        

        opti.set_initial(throttle, 1)

        p_opts = {"expand":True}
        s_opts = {"max_iter": 200}
        opti.solver("ipopt", p_opts, s_opts)


        opti.minimize(cost_function)

        # opti.solver("ipopt")
        sol = opti.solve()

        throttle_output = sol.value(throttle)
        steer_output = sol.value(steering)

        return steer_output[0], throttle_output[0]

        # print(sol.value(v))
        # print(sol.value(x))
        # print(sol.value(steering))
        


    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        # self.update_desired_speed()
        # v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            throttle_output = 0
            brake_output = 0
            steer_output = 0

            
            # print(self._current_timestamp)
            steer_output, throttle_output = self.mpc()

            # print(throttle_output)
            # print(self._current_timestamp)
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
