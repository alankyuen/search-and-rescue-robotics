def waypointGen():
	S_R = 12.5
	waypoint = [S_R,0]
	print(waypoint)

	delta_wp = [0,200-S_R] #(+)
	waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
	print(waypoint)
	negate = 1
	i = 1
	while(abs(100-waypoint[0]) > S_R and abs(100-waypoint[1]) > S_R):
		delta_wp = [negate*(200-(2*i*S_R)),0] #(+)
		waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
		print(waypoint)

		if(abs(100-waypoint[0]) > S_R or abs(100-waypoint[1]) > S_R):
			delta_wp = [0,negate*((2*i*S_R)-200)] #(+)
			waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
			print(waypoint)
		negate *= -1
		i+= 1

waypointGen()