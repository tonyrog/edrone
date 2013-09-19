%%% @author Magnus Feuer <magnus.feuer@feuerlabs.se>
%%% @copyright (C) 2013, Feuerlabs, Inc
%%% @doc
%%%    Motorboard control
%%% @end
%%% Created :  17 Sep 2013 by Magnus Feuer <magnus.feuer@feuerlabs.com>


-module(edrone_navboard).

-export([init/0,
	 decode_nav_frame/1,
	 flat_trim/1,
	 flat_trim/3,
	 read_raw_nav_frame/1,
	 read_raw_nav_frame/2,
	 read_nav_state/2,
	 read_nav_state/3,
	test/1]).

-define(NAV_UART, "/dev/ttyO1").
-define(CALIBRATE_TIMEOUT, 500).
-define(CALIBRATE_ITERATIONS, 200).
-define(NAV_GPIO, 132).
-define(NAV_START_CMD, <<16#01:8>>).
-define(NAV_FRAME_SIZE, 60).
-define(DEF_TOLERANCE, 1200.0).
-define(DEF_DEVIATION, 10.0).

%% Number of accerometer units per G (9.81 m/s^2).
%% 1G = 512 units.
-define(ACC_UNITS_PER_G, 5022.72). %% Number of accelerometer increments per G.

%% Number of Gs per accelerometer unit
-define(ACC_G_PER_UNIT,0.001953125 ). %% Number of accelerometer increments per G.

%% Number of degrees per gyro rotational increment
%%
%% Max raw value read from gyro: 32767
%% Max number of degrees reported by gyro: 2000
%% DEG_GYRO_CONST = 32767 / 2000
-define(GYRO_DEG_PER_UNIT, 0.061037018951994385). 

%% Number of compass units per degree
%% 65537 / 360
-define(MAG_UNITS_PER_DEG, 182.04722222). 

%% Number of degrees per compass unit.
%% 360 / 65537
-define(MAG_DEG_PER_UNIT, 0.0054930802447472424). 

%% Measured offsets for accelrometers gyros and compass while the drone
%% is flat and stationary.
%% Used to calculate absolute values while the drone is in motion.
-record(flat_trim, {
	  ax_offset = 2048.0 :: float(),    %% X accelerometer offset with horizontal drone
	  ay_offset = 2048.0 :: float(),    %% Y accelerometer offset with horizontal drone
	  az_offset = 2048.0 :: float(),    %% Z accelerometer offset with horizontal drone
	  gx_offset = 0.0    :: float(),    %% X gyro offset with horizontal drone
	  gy_offset = 0.0    :: float(),    %% Y gyro offset with horizontal drone
	  gz_offset = 0.0    :: float(),    %% Z gyro offset with horizontal drone
	  mx_offset = 0.0    :: float(),    %% X compass offset (always 0)
	  my_offset = 0.0    :: float(),    %% Y compass offset (always 0)
	  mz_offset = 0.0    :: float()     %% Z compass offset (always 0)
	 }).

%% Measurements made during flat trimming process.
-record(calibration, {
	  count = 0          :: integer(),  %% Numbber of samples made during trim
	  ax_sum = 0.0       :: float(),    %% Sum of all X accelerometer samples.
	  ay_sum = 0.0       :: float(),    %% Sum of all Y accelerometer samples.
	  az_sum = 0.0       :: float(),    %% Sum of all Z accelerometer samples.
	  gx_sum = 0.0       :: float(),    %% Sum of all X gyro samples.
	  gy_sum = 0.0       :: float(),    %% Sum of all Y gyro samples.
	  gz_sum = 0.0       :: float(),    %% Sum of all Z gyro samples.
	  ax_sqr = 0.0       :: float(),    %% Square of all X accelerometer samples.
	  ay_sqr = 0.0       :: float(),    %% Square of all Y accelerometer samples.
	  az_sqr = 0.0       :: float(),    %% Square of all Z accelerometer samples.
	  gx_sqr = 0.0       :: float(),    %% Square of all X gyro samples.
	  gy_sqr = 0.0       :: float(),    %% Square of all Y gyro samples.
	  gz_sqr = 0.0       :: float()     %% Square of all Z gyro samples.
}).

-record(nb_state, {

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
%% Decoded raw record,  as it is received from the nav board
%%
-record(nb_raw, {
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



read_raw_nav_frame(Uart) ->
    read_raw_nav_frame(Uart,infinity).

read_raw_nav_frame(Uart, Timeout) ->
    case uart:recv(Uart, ?NAV_FRAME_SIZE, Timeout) of
	{ok, Frame} -> decode_nav_frame(Frame);
	Other -> Other
    end.

%% Read a nav frame and convert it to actual values.
%% Use FlatTrim as offset values to get the absolute values
%% right.
read_nav_state(Uart, #flat_trim {} = FlatTrim) ->
    read_nav_state(Uart,infinity, FlatTrim).

read_nav_state(Uart, Timeout, #flat_trim {} = FlatTrim) ->
    case read_raw_nav_frame(Uart, Timeout) of
	{ ok, Frame } -> { ok,  nav_frame_to_state(Frame, FlatTrim) };
	Other -> Other
    end.
	
%% FIXME: Unknown3 & 0x1E contains sum of all values except checksum. 
%%        For now, we just check that length is 58.
validate_frame(<< (?NAV_FRAME_SIZE-2): 16/little-integer, _/binary >>) -> true;
validate_frame(_) -> false.
    

decode_nav_frame(FrameBin) ->
    << Length               :16/little-integer,
       SeqNr                :16/little-integer,
       AccX                 :16/little-integer,
       AccY                 :16/little-integer,
       AccZ                 :16/little-integer,
       GyroX                :16/little-integer-signed,
       GyroY                :16/little-integer-signed,
       GyroZ                :16/little-integer-signed,
       Unknown1             :16/little-integer, 
       Unknown2             :16/little-integer, 
       UsEcho               :16/little-integer,
       UsEchoStart          :16/little-integer,
       UsEchoEnd            :16/little-integer,
       UsAssociationEcho    :16/little-integer,
       UsDistanceEcho       :16/little-integer,
       UsCourbeTemps        :16/little-integer,
       UsCourbeValue        :16/little-integer,
       Unknown3             :16/little-integer,
       UsNumberEcho         :16/little-integer,
       UsSumEcho1           :16/little-integer,
       UsSumEcho2           :16/little-integer,
       Unknown4             :16/little-integer,
       UsInitialized        :16/little-integer,
       Unknown5             :16/little-integer,
       Unknown6             :16/little-integer,
       Unknown7             :16/little-integer,
       MagX                 :16/little-integer,
       MagY                 :16/little-integer,
       MagZ                 :16/little-integer,
       _Unknown8             :16/little-integer
    >> = FrameBin,

    %% io:format("L(~5B) S(~5B) AX(~5B) AY(~5B) AZ(~5B) GX(~6B) GY(~6B) GZ(~6B) UN(~1B) UE(~6B) " ++
    %% 		  "MX(~6B), MY(~6B) MZ(~6B)~n",
    %% 	      [ Length, SeqNr, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, 
    %% 		UsEcho bsr 15, UsEcho band 16#7FFF, MagX, MagY, MagZ ]),

    case validate_frame(FrameBin) of
	true -> { ok, #nb_raw {
		 length               = Length,
		 seq_nr               = SeqNr,
		 acc_x                = AccX,
		 acc_y                = AccY,
		 acc_z                = AccZ,
		 gyro_x               = GyroX,
		 gyro_y               = GyroY,
		 gyro_z               = GyroZ,
		 unknown_1            = Unknown1, 
		 unknown_2            = Unknown2, 
		 us_echo_new          = UsEcho bsr 15,       %% Couldn't figure out the bit syntax for this
		 us_echo              = UsEcho band 16#7FFF, %% and this.
		 us_echo_start        = UsEchoStart,
		 us_echo_end          = UsEchoEnd,
		 us_association_echo  = UsAssociationEcho,
		 us_distance_echo     = UsDistanceEcho,
		 us_courbe_temps      = UsCourbeTemps,
		 us_courbe_value      = UsCourbeValue,
		 unknown_3            = Unknown3,
		 us_number_echo       = UsNumberEcho,
		 us_sum_echo_1        = UsSumEcho1,
		 us_sum_echo_2        = UsSumEcho2,
		 unknown_4            = Unknown4,
		 us_initialized       = UsInitialized,
		 unknown_5            = Unknown5,
		 unknown_6            = Unknown6,
		 unknown_7            = Unknown7,
		 mag_x                = MagX,
		 mag_y                = MagY,
		 mag_z                = MagZ 
		}
	      };
	_ -> { error, checksum }
    end.

nav_frame_to_state(#nb_raw {} = F, #flat_trim {} = FT) ->
    #nb_state {
	       timestamp = edrone_lib:timestamp(),
	       ax = -acc_value_to_g(F#nb_raw.acc_x - FT#flat_trim.ax_offset),
	       ay = acc_value_to_g(F#nb_raw.acc_y - FT#flat_trim.ay_offset),
	       az = acc_value_to_g(F#nb_raw.acc_z - FT#flat_trim.az_offset),

	       gx = (F#nb_raw.gyro_x - FT#flat_trim.gx_offset) * ?GYRO_DEG_PER_UNIT,
	       gy = (F#nb_raw.gyro_y - FT#flat_trim.gy_offset) * ?GYRO_DEG_PER_UNIT,
	       gz = (F#nb_raw.gyro_z - FT#flat_trim.gz_offset) * ?GYRO_DEG_PER_UNIT,
	       
	       mx = (F#nb_raw.mag_x - FT#flat_trim.mx_offset) * ?MAG_DEG_PER_UNIT,
	       my = (F#nb_raw.mag_y - FT#flat_trim.my_offset) * ?MAG_DEG_PER_UNIT,
	       mz = (F#nb_raw.mag_z - FT#flat_trim.mz_offset) * ?MAG_DEG_PER_UNIT,
	       
	       %% FIXME: Break us_echo up into us_echo_new (bit 15) and us_echo (bit 0-14).
	       height = F#nb_raw.us_echo,
	       new_height = case F#nb_raw.us_echo_new of
				1 -> true;
				0 -> false
			    end
	      }.




%%
%% Read in frames and validate them. 
%% Each failed validation generates a one byte read and then the read of another
%% frame. We do this 60 times (the frame size)
%%
sync_stream(Uart, Timeout) ->
    io:format("Stream sync start~n"),
    sync_stream(Uart, Timeout, ?NAV_FRAME_SIZE).

%% If we have tried shifting 60 times and still have no frame,
%% we give up.
sync_stream(_Uart, _Timeout, 0) ->
    io:format("Stream sync failed~n"),
    { error, sync_failed };

%% Read a complete frame and attempt to decode it.
%% If we fail, read a single byte to shift the stream.
%% and try again.
sync_stream(Uart, Timeout, Count) ->
    case read_raw_nav_frame(Uart, Timeout) of
	{ok, _ } ->    
	    io:format("Stream sync done~n"),
	    ok;
	_ ->  %% Frame could not be read for some reason.
	    %% Read a byte to shift input stream.
	    case uart:recv(Uart, 1, Timeout) of
		{ok, _ } ->  %% Shift successful. Call self.
		    sync_stream(Uart, Timeout, Count - 1);

		Err -> Err %% Could not read byte. Error out
	    end
    end.
    


%%
%% Sync the data stream and calibrate the sensors.
%%
%% The data stream sync gets the frame reading in phase.  
%%
%% We then read "Iterations" frames to build up a #calibration record,
%% which is used by the caller, flat_trim(), to do average and
%% standard deviation in order to get sensor offsets used to generate
%% adjusted absolute accelerometer, gyro and compass values
%%
calibrate(Uart, Iterations, Timeout) ->
    %% First sync the data stream.
    case sync_stream(Uart, Timeout) of
	error -> { error, sync_failed };
	ok -> calibrate_(Uart, #calibration {}, Iterations, Timeout)
    end.
	     

calibrate_(_Uart, #calibration{ } = Calibration, 0, _) ->
    {ok, Calibration };

calibrate_(Uart, #calibration{ } = C, IterationsLeft, Timeout) ->
    case read_raw_nav_frame(Uart, Timeout) of
	{ ok, Fr } ->
	    calibrate_(Uart, #calibration {
			count = C#calibration.count + 1,
			ax_sum = C#calibration.ax_sum + Fr#nb_raw.acc_x,
			ay_sum = C#calibration.ay_sum + Fr#nb_raw.acc_y,
			az_sum = C#calibration.az_sum + Fr#nb_raw.acc_z,

			gx_sum = C#calibration.gx_sum + Fr#nb_raw.gyro_x,
			gy_sum = C#calibration.gy_sum + Fr#nb_raw.gyro_y,
			gz_sum = C#calibration.gz_sum + Fr#nb_raw.gyro_z,

			ax_sqr = C#calibration.ax_sqr + Fr#nb_raw.acc_x * Fr#nb_raw.acc_x,
			ay_sqr = C#calibration.ay_sqr + Fr#nb_raw.acc_y * Fr#nb_raw.acc_y,
			az_sqr = C#calibration.az_sqr + Fr#nb_raw.acc_z * Fr#nb_raw.acc_z ,

			gx_sqr = C#calibration.gx_sqr + Fr#nb_raw.gyro_x * Fr#nb_raw.gyro_x,
			gy_sqr = C#calibration.gy_sqr + Fr#nb_raw.gyro_y * Fr#nb_raw.gyro_y,
			gz_sqr = C#calibration.gz_sqr + Fr#nb_raw.gyro_z * Fr#nb_raw.gyro_z
		       }, IterationsLeft - 1, Timeout);

	%% Failed to read frame. Should not happen since we are synced.
	Other -> Other
    end.


%%
%% Get offset data for accelerometers, gytos and compass when
%% the drona is flat and stationary.
%%
%% These will be used as reference offsets when calculating
%% data during drone operation.
%%
std_deviation(Sum, Square, Count) ->
    (Square - Sum * Sum / Count) / (Count - 1).

validate_deviation([], _) ->
    true;

validate_deviation([H | _T], Deviation) when H >= Deviation  ->
    io:format("F: validate_deviation(H(~p) Deviation(~p)~n",
	      [ H, Deviation]),
    out_of_range;

validate_deviation([H | T], Deviation) ->
    io:format("S: validate_deviation(H(~p) Deviation(~p)~n",
	      [ H, Deviation]),
    validate_deviation(T, Deviation).

%% Convert raw accelerometer value to G (as in gravity).
acc_value_to_g(Value) ->
    Value / ?ACC_UNITS_PER_G.

%% Check that we are within tolerances of a flat trim for accelerometers
validate_tolerances(AxAvg, AyAvg, AzAvg, Tolerance) ->
    io:format("validate_tolerance(AxAvg(~p) AyAvg(~p) AzAvg(~p) Tolerance(~p))~n",
	      [ AxAvg, AyAvg, AzAvg, Tolerance]),

    if AxAvg < 2048.0 - Tolerance -> io:format("axmin~n"), false;
       AxAvg > 2048.0 + Tolerance -> io:format("axmax~n"), false;
       AyAvg < 2048.0 - Tolerance -> io:format("aymin~n"), false;
       AyAvg > 2048.0 + Tolerance -> io:format("aymax~n"), false;
       AzAvg < 2048.0 - ?ACC_UNITS_PER_G - Tolerance-> io:format("azmin~n"),  false; %% 1G earth gravity pulling Z axis down
       AzAvg > 2048.0 + ?ACC_UNITS_PER_G + Tolerance-> io:format("azmax~n"),false; %% 1G earth gravity pulling Z axis down
       true -> true
    end.
        

flat_trim(Uart) ->
    flat_trim(Uart, ?DEF_DEVIATION, ?DEF_TOLERANCE).

flat_trim(Uart, Deviation, Tolerance) ->
    case calibrate(Uart, ?CALIBRATE_ITERATIONS, ?CALIBRATE_TIMEOUT) of
	{ok, C = #calibration { count = Cnt } } ->


	    %% Calc avg and standard deviation
	    AxAvg = C#calibration.ax_sum / Cnt,
	    AyAvg = C#calibration.ay_sum / Cnt,
	    AzAvg = C#calibration.az_sum / Cnt,

	    GxAvg = C#calibration.gx_sum / Cnt,
	    GyAvg = C#calibration.gy_sum / Cnt,
	    GzAvg = C#calibration.gz_sum / Cnt,

	    AxStd = std_deviation(C#calibration.ax_sum, C#calibration.ax_sqr, Cnt),
	    AyStd = std_deviation(C#calibration.ay_sum, C#calibration.ay_sqr, Cnt),
	    AzStd = std_deviation(C#calibration.az_sum, C#calibration.az_sqr, Cnt),

	    GxStd = std_deviation(C#calibration.gx_sum, C#calibration.gx_sqr, Cnt),
	    GyStd = std_deviation(C#calibration.gy_sum, C#calibration.gy_sqr, Cnt),
	    GzStd = std_deviation(C#calibration.gz_sum, C#calibration.gz_sqr, Cnt),

	    AxStd1 = if AxStd =< 0.0 -> 0.0; true -> math:sqrt(AxStd) end,
	    AyStd1 = if AyStd =< 0.0 -> 0.0; true -> math:sqrt(AyStd) end,
	    AzStd1 = if AzStd =< 0.0 -> 0.0; true -> math:sqrt(AzStd) end,

	    GxStd1 = if GxStd =< 0.0 -> 0.0; true -> math:sqrt(GxStd) end,
	    GyStd1 = if GyStd =< 0.0 -> 0.0; true -> math:sqrt(GyStd) end,
	    GzStd1 = if GzStd =< 0.0 -> 0.0; true -> math:sqrt(GzStd) end,

	    case 
		validate_deviation([AxStd1, AyStd1, AzStd1, GxStd1, GyStd1, GzStd1], Deviation) andalso
		validate_tolerances(AxAvg, AyAvg, AzAvg, Tolerance) of
		true -> 
		    {ok, #flat_trim {
		       ax_offset = AxAvg,
		       ay_offset = AyAvg,
		       az_offset = AzAvg - ?ACC_UNITS_PER_G,
		       gx_offset = GxAvg,
		       gy_offset = GyAvg,
		       gz_offset = GzAvg,
		       %% FIXME: Calibrate compass as well.
		       mx_offset = 0.0,
		       my_offset = 0.0,
		       mz_offset = 0.0
		      }};
		_ ->
		    { error, flattrim_failed }
	    end;
	Err -> Err
    end.


init() ->
    {ok,U} = uart:open(?NAV_UART, [{baud,460800},{mode,binary}]),
    %% set /MCLR pin
    gpio:init(?NAV_GPIO),
    gpio:set_direction(?NAV_GPIO, low),
    gpio:set(?NAV_GPIO),
    
    %% Start acquisition
    uart:send(U, ?NAV_START_CMD),
    
    {ok, U}.


test(U) ->
    {ok, FT} = flat_trim(U),
    io:format("FlatTrim: ax_off(~p) ay_off(~p) az_off(~p) gx_off(~p) gy_off(~p) gz_off(~p)~n",
	      [ FT#flat_trim.ax_offset, FT#flat_trim.ay_offset, FT#flat_trim.az_offset, 
		FT#flat_trim.gx_offset, FT#flat_trim.gy_offset, FT#flat_trim.gz_offset]),
    test_(U, FT).
    
test_(U, FT) ->
    {ok, St} = edrone_navboard:read_nav_state(U, FT),
    io:format("nav: Acc(g)={x(~-10.4f) y(~-10.4f) z(~-10.4f)} Gyro(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)} Height(cm)=~B Compass(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)}~n", 
	      [ St#nb_state.ax, St#nb_state.ay, St#nb_state.az, 
		St#nb_state.gx, St#nb_state.gy, St#nb_state.gz, 
		St#nb_state.height,
		St#nb_state.mx, St#nb_state.my, St#nb_state.mz ]),

    test_(U, FT).
    

