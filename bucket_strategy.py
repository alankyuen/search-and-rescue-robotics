    # send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # condition_yaw - send condition_yaw mavlink command to vehicle so it points at specified heading (in degrees)
    def condition_yaw(self, heading):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     heading, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
