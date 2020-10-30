

import numpy  as np
import math as mt
# import pandas as pd
def map2car(yaw, current_x, current_y, x_world):

      
      yaw = -yaw
      T_c_r = [[mt.cos(yaw), -mt.sin(yaw), 0, 0],
               [mt.sin(yaw), mt.cos(yaw), 0, 0],   # current yaw = -1*yaw (car frame rotation matrix)(because we changed from right hand to left convention)
               [0,  0, 1, 0],
               [0,  0, 0, 1]]

      T_c_t = [[1, 0, 0, -1*current_x],
               [0, 1, 0, -1*current_y],   # car frame translation matrix
               [0,  0, 1, 0],
               [0,  0, 0, 1]]
      
     
      X_t = np.matmul(T_c_t, x_world.transpose())       # multiplying by translation matrix and translating the points

      # T3_1 = np.matmul(T1,T3)
      # print(T3_1)
      # Tc_w = np.matmul(T3_1, T_c_r)  

        
      
      X= np.matmul(T_c_r, X_t)               # mulitplying by the rotation matrix
      # print(X)
      return X
      
def main():

      current_x = 0
      current_y = 0

      x = np.array([ 3, 4 , 0, 1])
      
      map2car(0,current_x,current_y, x)
      
if __name__=="__main__": 
    main() 

# X = [[4],[4],[0],[1]] 
# x = np.matmul(T,X)
# print(x)