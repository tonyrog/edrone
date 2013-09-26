%% Current, filtered state of the navboard sensors.
-record(nav_state, {
	  timestamp = 0             :: integer(),  %% Timestamp in monotonic microseconds

	  ax = 0.0                  :: float(),    %% X-axis acceleration (G) front facing up is positive
	  ay = 0.0                  :: float(),    %% Y-axis acceleration (G) left facing up is positive
	  az = 0.0                  :: float(),    %% Z-axis acceleration (G) top facing up is positive

	  gx = 0.0                  :: float(),    %% X-axis gyro (deg/sec) roll right is positive
	  gy = 0.0                  :: float(),    %% Y-axis gyro (deg/sec) pitch down positive
	  gz = 0.0                  :: float(),    %% Z-axis gyro (deg/sec) yaw left is positive

	  height = 0                :: float(),    %% height above ground (cm)
	  new_height = false        :: boolean(),  %% true if new height sample, else false.

	  mx = 0.0                  :: float(),    %% X plane compass (This is the one we want)
	  my = 0.0                  :: float(),    %% Y plane compass (Not sure what to use this for)
	  mz = 0.0                  :: float()     %% Z plane compass (Not sure what to use this for)
	 }).



%%
%% Position of drone, relative to north facing, flat drone at starting point
%% x < 0 -> Drone is left of its starting point. (cm)
%% x > 0 -> Drone is right of its starting point.(cm)
%% y < 0 -> Drone is behind its starting point. (cm)
%% y > 0 -> Drone is in front of its starting point. (cm)
%% height = Height (cm). Continuously calibrated against sonar to manage terrain.
%%
%% yaw -> Heading, in offset from absolute north (-1.0 to 1.0). 0.0 = start heading. mirrored heading.
%% pitch -> Nose Pitch, in from  flat (-1.0 to 1.0).  0.0 = flat. -1.0, 1.0 = inverted
%% roll -> Roll, from flat. (-1.0 to 1.0). 0.0 = flat. -1.0, 1.0 = inverted
%% heading -> Compass heading of the nose. (0-360)
%% direction -> Compass heading of the movement (0-360)
%% speed -> Speed (cm/sec) that drone is moving in 'direction'
%% climb -> Speed (cm/sec) that drone is climbing (positive) or falling (negative)
-record(position, {
	  timestamp = 0   :: integer(),   %% Timestamp of position
	  x = 0.0         :: float(),    %% Lateral position, relative start
	  y = 0.0         :: float(),    %% Longitudinal position, relative startg
	  height = 0.0    :: float(),    %% Height position, relative sonar ground
	  yaw = 0.0       :: float(),   %% Yaw
	  pitch = 0.0     :: float(), %% Pitch
	  roll = 0.0      :: float(),  %% Roll
	  heading = 0.0   :: float(), 
	  direction = 0.0 :: float(),  
	  speed = 0.0     :: float(),  
	  climb = 0.0     :: float(),  
	  last_st         :: #nav_state{}  %% Last known state.
	 }).

%%
%% Decoded raw record,  as it is received from the nav board
%%
-record(nav_frame, {
	  length  ::integer(),              %%  length
	  seq_nr  ::integer(),              %%  Sequence number for the frame
	  acc_x   ::integer(),              %%  Raw X accelerometer (10 bit, multiplied by 4)
	  acc_y   ::integer(),              %%  Raw Y accelerometer (10 bit, multiplied by 4)
	  acc_z   ::integer(),              %%  Raw Z accelerometer (10 bit, multiplied by 4)
	  gyro_x  ::integer(),              %%  Raw X gyro (16 bit).
	  gyro_y  ::integer(),              %%  Raw Y gyro (16 bit).
	  gyro_z  ::integer(),              %%  Raw Z gyro (16 bit).

	  unknown_1  ::integer(),           %% unknown_ 1 (16 bit)
	  unknown_2  ::integer(),           %% unknown_ 2 (16 bit)

	  us_echo_new  ::integer(),         %% 1 bit - New measurement or incremental(?) measurement
	  us_echo  ::integer(),             %% 15 bit - first echo. Value 30 = 1cm. min value: 784 = 26cm

	  us_echo_start  ::integer(),         %% Array with starts of echos 
	  %% (8 array values @ 25Hz, 9 values @ 22.22Hz)

	  us_echo_end  ::integer(),           %% Array with ends of echos 
	  %% (8 array values @ 25Hz, 9 values @ 22.22Hz)

	  us_association_echo  ::integer(),   %% Ultrasonic parameter -- 
	  %% echo number starting with 0. max value 3758. 
	  %% examples: 0,1,2,3,4,5,6,7  ; 0,1,2,3,4,86,6,9

	  us_distance_echo  ::integer(),      %%  Ultrasonic parameter -- no clear pattern
	  us_courbe_temps  ::integer(),       %%  Ultrasonic parameter -- 
	  %% counts up from 0 to approx 24346 in 192 sample cycles 
	  %% of which 12 cylces have value 0

	  us_courbe_value  ::integer(),       %% Ultrasonic parameter 
	  %% value between 0 and 4000, no clear pattern. 
	  %% 192 sample cycles of which 12 cylces have value 0

	  unknown_3  ::integer(),            %% unknown_ +0x1E Checksum = sum of all values 
	  %% except checksum (22 values)

	  us_number_echo  ::integer(),        %%  Number of selected echo for height measurement (1,2,3)
	  us_sum_echo_1  ::integer(),        %%
	  us_sum_echo_2  ::integer(),        %%
	  unknown_4  ::integer(),            %% unknown +0x26 Ultrasonic parameter -- no clear pattern
	  us_initialized  ::integer(),       %% always 1
	  unknown_5  ::integer(),            %%
	  unknown_6  ::integer(),            %%
	  unknown_7  ::integer(),            %%
	  mag_x  ::integer(),                %% Magnetic X (This is the compass we want)
	  mag_y  ::integer(),                %% Magnetic Y
	  mag_z  ::integer(),                %% Magnetic Z

	  unknown_8 ::integer()            %% 
	 }).

%% One earth gravity, in m/s^2
-define(G, 9.80665).
