%%% @author Magnus Feuer <magnus.feuer@feuerlabs.se>
%%% @copyright (C) 2013, Feuerlabs, Inc
%%% @doc
%%%    Motorboard control
%%% @end
%%% Created :  17 Sep 2013 by Magnus Feuer <magnus.feuer@feuerlabs.com>


-module(edrone_navboard).

-export([init/0,
	 flat_trim/1,
	 flat_trim/3,
	 sync_stream/2,
	 enable_frame_report/1,
	 decode_nav_data/1,
	 process_nav_frame/2]).

-include("nav.hrl").

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

%% Number of sonar units per cm.
-define(SONAR_CM_PER_UNITS, 0.000340)

%% Cm per sonar unit.
-define(SONAR_UNITS_PER_CM, 30).


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


read_raw_nav_frame(Uart, Timeout) ->
    case uart:recv(Uart, ?NAV_FRAME_SIZE, Timeout) of
	{ok, Frame} -> decode_nav_data(Frame);
	Other -> Other
    end.

%% FIXME: Unknown3 & 0x1E contains sum of all values except checksum. 
%%        For now, we just check that length is 58.
validate_frame(<< (?NAV_FRAME_SIZE-2): 16/little-integer, _/binary >>) -> true;
validate_frame(_) -> false.
    

decode_nav_data(Data) ->
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
    >> = Data,

    %% io:format("L(~5B) S(~5B) AX(~5B) AY(~5B) AZ(~5B) GX(~6B) GY(~6B) GZ(~6B) UN(~1B) UE(~6B) " ++
    %% 		  "MX(~6B), MY(~6B) MZ(~6B)~n",
    %% 	      [ Length, SeqNr, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, 
    %% 		UsEcho bsr 15, UsEcho band 16#7FFF, MagX, MagY, MagZ ]),

    case validate_frame(Data) of
	true -> { ok, #nav_frame {
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


process_nav_frame(#nav_frame {} = Frame, #flat_trim {} = FT) ->
    #nav_state {
		timestamp = edrone_lib:timestamp(),
		ax = -acc_value_to_g(Frame#nav_frame.acc_x - FT#flat_trim.ax_offset),
		ay = acc_value_to_g(Frame#nav_frame.acc_y - FT#flat_trim.ay_offset),
		az = acc_value_to_g(Frame#nav_frame.acc_z - FT#flat_trim.az_offset),

		gx = (Frame#nav_frame.gyro_x - FT#flat_trim.gx_offset) * ?GYRO_DEG_PER_UNIT,
		gy = (Frame#nav_frame.gyro_y - FT#flat_trim.gy_offset) * ?GYRO_DEG_PER_UNIT,
		gz = (Frame#nav_frame.gyro_z - FT#flat_trim.gz_offset) * ?GYRO_DEG_PER_UNIT,

		mx = (Frame#nav_frame.mag_x - FT#flat_trim.mx_offset) * ?MAG_DEG_PER_UNIT,
		my = (Frame#nav_frame.mag_y - FT#flat_trim.my_offset) * ?MAG_DEG_PER_UNIT,
		mz = (Frame#nav_frame.mag_z - FT#flat_trim.mz_offset) * ?MAG_DEG_PER_UNIT,

		%% FIXME: Break us_echo up into us_echo_new (bit 15) and us_echo (bit 0-14).
		height = Frame#nav_frame.us_echo / 30 ,
		new_height = case Frame#nav_frame.us_echo_new of
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
			ax_sum = C#calibration.ax_sum + Fr#nav_frame.acc_x,
			ay_sum = C#calibration.ay_sum + Fr#nav_frame.acc_y,
			az_sum = C#calibration.az_sum + Fr#nav_frame.acc_z,

			gx_sum = C#calibration.gx_sum + Fr#nav_frame.gyro_x,
			gy_sum = C#calibration.gy_sum + Fr#nav_frame.gyro_y,
			gz_sum = C#calibration.gz_sum + Fr#nav_frame.gyro_z,

			ax_sqr = C#calibration.ax_sqr + Fr#nav_frame.acc_x * Fr#nav_frame.acc_x,
			ay_sqr = C#calibration.ay_sqr + Fr#nav_frame.acc_y * Fr#nav_frame.acc_y,
			az_sqr = C#calibration.az_sqr + Fr#nav_frame.acc_z * Fr#nav_frame.acc_z ,

			gx_sqr = C#calibration.gx_sqr + Fr#nav_frame.gyro_x * Fr#nav_frame.gyro_x,
			gy_sqr = C#calibration.gy_sqr + Fr#nav_frame.gyro_y * Fr#nav_frame.gyro_y,
			gz_sqr = C#calibration.gz_sqr + Fr#nav_frame.gyro_z * Fr#nav_frame.gyro_z
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
    case flat_trim(Uart, ?DEF_DEVIATION, ?DEF_TOLERANCE) of 
	{ ok, FT } ->
	    io:format("FlatTrim: ax_off(~p) ay_off(~p) az_off(~p) gx_off(~p) gy_off(~p) gz_off(~p)~n",
		      [ FT#flat_trim.ax_offset, FT#flat_trim.ay_offset, FT#flat_trim.az_offset, 
			FT#flat_trim.gx_offset, FT#flat_trim.gy_offset, FT#flat_trim.gz_offset]),
	    {ok, FT};
	Err -> Err
    end.
		


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


%% Enable a single frame report
enable_frame_report(Uart) ->
%%    DOES NOT WORK! Does not send message to inbox.
%%    uart:setopts(Uart, [{packet, {size, ?NAV_FRAME_SIZE}, { active, once }}]),
    uart:async_recv(Uart, ?NAV_FRAME_SIZE, 100).

init() ->
    %% Open the uart with re-arm
    {ok,U} = uart:open(?NAV_UART, [{baud,460800},
				   {mode,binary}]),
    %% set /MCLR pin
    gpio:init(?NAV_GPIO),
    gpio:set_direction(?NAV_GPIO, low),
    gpio:set(?NAV_GPIO),
    
    %% Start acquisition
    uart:send(U, ?NAV_START_CMD),
    
    {ok, U}.
