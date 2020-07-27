#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

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
        self._min_idx            = -1


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

    def rear_axle_coord(self):
        l = 1.5 #(in meters)
        rear_axle_x = self._current_x - l*np.cos(self._current_yaw)
        rear_axle_y = self._current_y - l*np.sin(self._current_yaw)
        return rear_axle_x, rear_axle_y 


    def front_axle_coord(self):
        x = self._current_x + self._vehicle_length*np.cos(self._current_yaw)/2
        y = self._current_y + self._vehicle_length*np.sin(self._current_yaw)/2

        return x,y

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        x,y = self.front_axle_coord()
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - x,
                    self._waypoints[i][1] - y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
            self._min_idx = min_idx
        else:
            desired_speed = self._waypoints[-1][2]
            self._min_idx = -1
        self._desired_speed = desired_speed

    def min_index_pp(self, look_ahead_distance):
        dist = 0
        min_dist = float("inf")
        rear_axle_x, rear_axle_y = self.rear_axle_coord() 
        
        for i in range(len(self._waypoints)):
            dist = np.sqrt((self._waypoints[i][0]-rear_axle_x)**2+(self._waypoints[i][1]-rear_axle_y)**2)
            diff_dist = np.abs(dist - look_ahead_distance)
            if diff_dist< min_dist:
                min_dist = diff_dist
                min_idx = i


        return min_idx

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


    

    def get_cte(self,x,y,target_x,target_y,target_x_n,target_y_n):

        try:
            delta_y = target_y_n - target_y
            delta_x = target_x_n - target_x
        except:
            delta_x = 0
            delta_y = 0

        slope = np.tan(delta_y/delta_x)
        cte = np.abs(y - x*slope-(target_y - \
            slope*target_x))/(np.sqrt(1+slope**2))
        return cte 

    def get_head_err(self,current_yaw,target_x,target_y,target_x_n,target_y_n):

        try:
            delta_y = target_y_n - target_y
            delta_x = target_x_n - target_x
        except:
            delta_x = 0
            delta_y = 0

        head = np.arctan2(delta_y, delta_x)
        delta = head - current_yaw
       

        if delta > np.pi:
            delta = delta - self._2pi
        if delta < -np.pi:
            delta = delta + self._2pi

        return delta

 
    def get_cte_head_err(self):

        x,y = self.front_axle_coord()

        target_x = self._waypoints[self._min_idx][0]
        target_y = self._waypoints[self._min_idx][1]
        target_x_n = self._waypoints[self._min_idx+1][0]
        target_y_n = self._waypoints[self._min_idx+1][1]


        head_err = self.get_head_err(self._current_yaw,target_x,target_y,target_x_n,target_y_n)  
        cte      = self.get_cte(x,y,target_x,target_y,target_x_n,target_y_n)

        return cte, head_err


    def lateral_pid(self,kp=1.0,ki=1.0,kd=0.0):

        self.vars.create_var('prev_cte',0)
        self.vars.create_var('cte',0)
        self.vars.create_var('sum_cte',0)
        
        


        delta_t = 0.033
        desired_cte = 0

        cte, head_err = self.get_cte_head_err()

         
        if  head_err < 0:
            cte   = -1*cte

        diff_err = cte - self.vars.cte
        self.vars.sum_cte =+ cte  
        
        steer_output = kp*cte + ki*self.vars.sum_cte*delta_t + kd*(diff_err/delta_t)
        
        self.vars.prev_cte = cte
        
        
       
        # print("kp",kp,"ki",ki,"kd",kd)
        return steer_output


    def stanley(self,k = 0.75,ks = 0.00):
        min_idx = self._min_idx
        
        current_speed = self._current_speed

        cte, head_err = self.get_cte_head_err()

        if  head_err < 0:
            cte   = -1*cte

        steer_output =  head_err + np.arctan2(k*cte,(ks+current_speed))

        # print(k,ks)
        return steer_output

    def pure_pursuit(self ,x ,y ,yaw ,waypoints,Kpp = 1.4):

        L = self._vehicle_length
       

        look_ahead_distance = Kpp * self._current_speed 
        min_idx = self.min_index_pp(look_ahead_distance)
     
        target_x = waypoints[min_idx][0]
        target_y = waypoints[min_idx][1]

        cte = ((target_y-y)*np.cos(yaw)-(target_x-x)*np.sin(yaw))  ###cross_track_error 
             
            
        k = 2*cte/(look_ahead_distance**2) 
        steer_output = np.arctan(k*L)
        
        # print(Kpp)
        return steer_output



    def longitudinal_pid(self,v_desired,kp=0.9,kd=0.2,ki=0.9):

        self.vars.create_var('err_previous',0)
        self.vars.create_var('sum_err',0)

        delta_t = 0.033

 
        error = v_desired - self._current_speed 
        sum_err = self.vars.sum_err + error
        diff_err = error - self.vars.err_previous

        
        throttle_output = kp*(error) + kd*(diff_err/delta_t) +ki*(sum_err*delta_t)

        self.vars.err_previous = error # storing the error for next step 
        self.vars.sum_err = sum_err 

        # print("kp",kp,"ki",ki,"kd",kd)

        return throttle_output 


    def average_displacement_error(self):

        self.vars.create_var('disp_error',0)
        self.vars.create_var('counter',0)
        

        target_x = self._waypoints[self._min_idx][0]
        target_y = self._waypoints[self._min_idx][1]

        current_x = self._current_x
        current_y = self._current_y

        dist = np.linalg.norm(np.array([target_x - self._current_x,
                    target_y - self._current_y]))

        self.vars.counter += 1 
        self.vars.disp_error = self.vars.disp_error + dist
        
        average_displacement_error = self.vars.disp_error/(self.vars.counter)
        return average_displacement_error

    


    def update_controls(self,option):
       ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
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
            
            # kp=0.9
            # kd=0.2
            # ki=0.9
            # throttle_output = self.longitudinal_pid(v_desired,kp,kd,ki)
            
            throttle_output = self.longitudinal_pid(v_desired)

            if throttle_output < 0:
                brake_output = -1*throttle_output

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
            # steer_output    = 0
            
            
            if option == 'lp':
                print("using lateral pid")
                # kp=1.0
                # ki=0.0
                # kd=0.0
                # steer_output = self.lateral_pid(kp,ki,kd)

                steer_output = self.lateral_pid()

            elif option == 'sc':
                print("using stanley controller")
                # k = 0.75
                # ks = 0.00
                # steer_output = self.stanley(k,ks)

                steer_output = self.stanley()  

            elif option == 'pp':
                print("using pure pursuit")
                # look ahead distance constant ---Kpp 
                # Kpp = 1.5
                # steer_output = self.pure_pursuit(x,y,yaw,waypoints,Kpp)
                
                steer_output = self.pure_pursuit(x,y,yaw,waypoints)
            

            print(steer_output)

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
