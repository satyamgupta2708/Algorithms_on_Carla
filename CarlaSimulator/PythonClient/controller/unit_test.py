 #!/usr/bin/env python3


import numpy as np 

from controller2d import Controller2D
# from Controller2D import get_head_err
# from controller2d import get_head_err




def main():

	obj = Controller2D([])
	target_x = -181.37
	target_y = 63.180   #18

	target_x_n = -181.38
	target_y_n = 62.13

	current_x = -183.803
	current_y = 79.984

	current_yaw = -1.57

	cte_calc = obj.get_cte(current_x,current_y,target_x,target_y,target_x_n,target_y_n)
	print(cte_calc)
    
	he_calc = obj.get_head_err(current_yaw,target_x,target_y,target_x_n,target_y_n)
	print(he_calc)

	cte_man = 2.649
	he_man = np.arctan2(-1.05,-0.0099) - current_yaw
	print(he_man)

	if (cte_man - cte_calc)<10**(-3):
		print("unit_test 1 passed")
	if (he_man - he_calc)<10**(-3):
		print("unit test 2 passed")


if __name__ == '__main__':

	main()
