
# uint8[2] head # Not in use
# uint8 level_flag #levelFlag # Not in use
# uint8 frame_reserve #frameReserve # Not in use

uint32[2] sn #SN
uint32[2] version
uint16 bandwidth #bandWidth
# IMU imu # Not in use
# MotorState[20] motor_state #motorState # Not in use
#BmsState bms # Excluded due to dedicated BMS topic
int16[4] foot_force #footForce
# int16[4] foot_force_est #footForceEst # Not in use

uint8 mode #// 0. idle, default stand
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

# float32 progress # Not in use

uint8 gait_type #gaitType 	// 0.idle
                           #// 1.trot
                           #// 2.trot running
                           #// 3.climb stair
                           #// 4.trot obstacle 

float32 foot_raise_height #footRaiseHeight	

float32[3] position #(unit: m), position in inertial frame. linear [x, y, z?]

float32 body_height #bodyHeight			

float32[3] velocity #(unit: m/s), [forwardSpeed, sideSpeed] in body frame

float32 yaw_speed #yawSpeed		// (unit: rad/s), rotateSpeed in body frame
                             #// (range: trot : -4.0~4.0rad/s)
                             #// (range: run  : -4.0~4.0rad/s)
                             #// (range: stair: -0.7~0.7rad/s)	
                             			   
# float32[4] range_obstacle #rangeObstacle # Not in use
# Cartesian[4] foot_position_to_body #footPosition2Body # Not in use
# Cartesian[4] foot_speed_to_body #footSpeed2Body	# Not in use
# uint8[40] wireless_remote #wirelessRemote # Not in use
# uint32 reserve # Not in use

# uint32 crc # Not in use