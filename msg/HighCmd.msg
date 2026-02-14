# uint8[2] head # Not in use
#uint8 level_flag #levelFlag # Not in use
# uint8 frame_reserve #frameReserve # Not in use

# uint32[2] sn #SN  -- Dont want to change the SN
# uint32[2] version -- Dont want to change the version
# uint16 bandwidth #bandWidth # Not in use

uint8 mode  #// 0. idle, default stand
            #// 1. force stand (controlled by dBodyHeight + ypr)
            #// 2. target velocity walking (controlled by velocity + yawSpeed)
            #// 3. target position walking (controlled by position + ypr[0]), reserve
            #// 4. path mode walking (reserve for future release), reserve
            #// 5. position stand down.
            #// 6. position stand up
            #// 7. damping mode
            #// 8. recovery stand
            #// 9. backflip, reserve
            #// 10. jumpYaw, only left direction. Note, to use this mode, you need to set mode = 1 first.
            #// 11. straightHand. Note, to use this mode, you need to set mode = 1 first.
            #// 12. Dance 1. Note, to use this mode, you need to set mode = 1 first.
            #// 13. Dance 3. Note, to use this mode, you need to set mode = 1 first.

uint8 gait_type #gaitType		// 0.idle
                           #// 1.trot
                           #// 2.trot running
                           #// 3.climb stair
                           #// 4.trot obstacle   

# uint8 speed_level #speedLevel # Not in use	

float32 foot_raise_height #footRaiseHeight		   
float32 body_height #bodyHeight	   

float32[2] position #(unit: m), desired position in inertial frame. linear [x, y]

float32[3] euler #(unit: rad), roll pitch yaw in stand mode
                 #// (unit: rad), roll pitch yaw in stand mode
                 #// (range: roll : -0.75~0.75rad)
                 #// (range: pitch: -0.75~0.75rad)
                 #// (range: yaw  : -0.6~0.6rad)
                 #// [roll, pitch, yaw]

float32[2] velocity #(unit: m/s), [forwardSpeed, sideSpeed] in body frame

float32 yaw_speed # yawSpeed	// (unit: rad/s), rotateSpeed in body frame
                             #// (range: trot : -4.0~4.0rad/s)
                             #// (range: run  : -4.0~4.0rad/s)
                             #// (range: stair: -0.7~0.7rad/s)	

# BmsCmd bms # Not in use
# LED[4] led # Not in use
# uint8[40] wireless_remote #wirelessRemote # Not in use
# uint32 reserve # Not in use

# uint32 crc # Not in use