bool    is_car_available
# Car state related
bool    is_position_left 	# Y position too small
bool    is_position_right	# Y position too big
bool    is_position_front	# X position too big
bool    is_position_back	# X position too small
bool    is_yaw_left			# yaw angle is too small
bool    is_yaw_right		# yaw angle is too larg
bool    is_steering_left	# front wheel steering angle is too large (left)
bool    is_steering_right	# front wheel steering angle is too small (right)
# Car related
bool    is_wheelbase_short 	# wheelbase length too short
bool    is_wheelbase_long	# wheelbase length too large
bool    is_wheel_distance_short 	# wheel distance too short
bool    is_wheel_distance_long 		# wheel distance too large
# Car access point (middle point of the front edge of front wheels)
float64 aX                   
float64 aY
float64 aYaw