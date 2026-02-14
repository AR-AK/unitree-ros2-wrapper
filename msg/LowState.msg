
uint8[2] head
uint8 level_flag #levelFlag
uint8 frame_reserve #frameReserve

uint32[2] sn #SN
uint32[2] version
uint16 bandwidth #bandWidth
IMU imu
MotorState[20] motor_state #motorState
BmsState bms
int16[4] foot_force #footForce	
int16[4] foot_force_est #footForceEst	
uint32 tick						
uint8[40] wireless_remote #wirelessRemote 
uint32 reserve
uint32 crc

# Old version Aliengo does not have:
Cartesian[4] ee_force_raw #eeForceRaw
Cartesian[4] ee_force #eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w            # will delete

