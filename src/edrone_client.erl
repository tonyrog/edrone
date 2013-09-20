%%%-------------------------------------------------------------------
%%% @author Tony Rogvall <tony@rogvall.se>
%%% @copyright (C) 2013, Tony Rogvall
%%% @doc
%%%    Send commands to AR drone
%%% @end
%%% Created : 11 Sep 2013 by Tony Rogvall <tony@rogvall.se>
%%%-------------------------------------------------------------------
-module(edrone_client).

-behaviour(gen_server).

-compile(export_all).
%% API
-export([start_link/0]).

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
	 terminate/2, code_change/3]).

-define(SERVER, ?MODULE). 

-define(UDP_COMMAND_PORT, 5556).
-define(UDP_NAVDATA_PORT, 5554).
-define(VIDEO_STREAM,     5555).
-define(TCP_CONFIG_PORT,  5559).  %% read config (once)

-define(DRONE_IP, {192,168,1,1}).

-define(INHIBIT_TIME, 30).
-define(WDT_TIME,     1970). %% 2s com timeout

-define(NAVDATA_MAGIC, 16#55667788).

-define(NAVDATA_DEMO_TAG, 0).
-define(NAVDATA_TIME_TAG, 1).
-define(NAVDATA_RAW_MEASURES_TAG, 2).
-define(NAVDATA_PHYS_MEASURES_TAG, 3).
-define(NAVDATA_GYROS_OFFSETS_TAG, 4).
-define(NAVDATA_EULER_ANGLES_TAG, 5).
-define(NAVDATA_REFERENCES_TAG, 6).
-define(NAVDATA_TRIMS_TAG, 7).
-define(NAVDATA_RC_REFERENCES_TAG, 8).
-define(NAVDATA_PWM_TAG, 9).
-define(NAVDATA_ALTITUDE_TAG, 10).
-define(NAVDATA_VISION_RAW_TAG, 11).
-define(NAVDATA_VISION_OF_TAG, 12).
-define(NAVDATA_VISION_TAG, 13).
-define(NAVDATA_VISION_PERF_TAG, 14).
-define(NAVDATA_TRACKERS_SEND_TAG, 15).
-define(NAVDATA_VISION_DETECT_TAG, 16).
-define(NAVDATA_WATCHDOG_TAG, 17).
-define(NAVDATA_ADC_DATA_FRAME_TAG, 18).
-define(NAVDATA_VIDEO_STREAM_TAG, 19).
-define(NAVDATA_CKS_TAG, 16#FFFF).

%% Control names
-define(ACCS_OFFSET, "control:accs_offset").
-define(ACCS_GAINS, "control:accs_gains").
-define(GYROS_OFFSET, "control:gyros_offset").
-define(GYROS_GAINS, "control:gyros_gains").
-define(GYROS110_OFFSET, "control:gyros110_offset").
-define(GYROS110_GAINS, "control:gyros110_gains").
-define(GYRO_OFFSET_THR_X, "control:gyro_offset_thr_x").
-define(GYRO_OFFSET_THR_Y, "control:gyro_offset_thr_y"). 
-define(GYRO_OFFSET_THR_Z, "control:gyro_offset_thr_z").
-define(PWM_REF_GYROS, "control:pwm_ref_gyros").
-define(CONTROL_LEVEL, "control:control_level").
-define(SHIELD_ENABLE, "control:shield_enable"). 
-define(EULER_ANGLE_MAX, "control:euler_angle_max").
-define(ALTITUDE_MAX, "control:altitude_max").
-define(ALTITUDE_MIN, "control:altitude_min").
-define(CONTROL_TRIM_Z, "control:control_trim_z").
-define(CONTROL_IPHONE_TILT, "control:control_iphone_tilt").
-define(CONTROL_VZ_MAX, "control:control_vz_max").
-define(CONTROL_YAW, "control:control_yaw").
-define(OUTDOOR, "control:outdoor").
-define(FLIGHT_WITHOUT_SHELL, "control:flight_without_shell").
-define(BRUSHLESS, "control:brushless").
-define(AUTONOMOUS_FLIGHT, "control:autonomous_flight").
-define(MANUAL_TRIM, "control:manual_trim").
-define(INDOOR_EULER_ANGLE_MAX, "control:indoor_euler_angle_max").
-define(INDOOR_CONTROL_VZ_MAX, "control:indoor_control_vz_max").
-define(INDOOR_CONTROL_YAW, "control:indoor_control_yaw").
-define(OUTDOOR_EULER_ANGLE_MAX, "control:outdoor_euler_angle_max").
-define(OUTDOOR_CONTROL_VZ_MAX, "control:outdoor_control_vz_max").
-define(OUTDOOR_CONTROL_YAW, "control:outdoor_control_yaw").

-define(FLIGHT_ANIM, "control:flight_anim").
-define(LEDS_ANIM,   "leds:leds_anim").
%%
%% video_channel selection
%% 0=horizontal, 
%% 1=vertical, 
%% 2=large_horizontal/small_vertical
%% 3=large_vertical/small_horizontal
-define(VIDEO_CHANNEL, "video:video_channel").

%% navdata reception
%% "FALSE" all navigation data
%% "TRUE"  demo navigation data
-define(NAVDATA_DEMO, "general:navdata_demo").

%%
%% nav_state stored in 32 bit little endian
%% that is
%% D0 D1 D2 D3
%% erlang naturally read big bits from left to right as stored as D3 D2 D1 D0
%% <<B31:1,B30:1,...B0:1>>
%% For little endian this is instead
%% << B7-B0,  B15-B8, B23-B16,  B31-B24 >>
%% We rearrange the record fields so we can read(32) straigt in from bit
%% fields like in C :-)
%%

-record(nav_state, 
	{
	  trimReceived          :: boolean(), %% bit 7
	  controlReceived       :: boolean(), %% bit 6
	  userFeedbackOn        :: boolean(), %% bit 5
	  altitudeControlActive :: boolean(), %% bit 4
	  controlAngularSpeed   :: boolean(), %% bit 3 (else euler_angles)
	  visionEnabled         :: boolean(), %% bit 2
	  videoEnabled          :: boolean(), %% bit 1
	  flying                :: boolean(), %% bit 0

	  batteryTooLow         :: boolean(), %% bit 15
	  gyrometersDown        :: boolean(), %% bit 14
	  comLost               :: boolean(), %% bit 13
	  motorsDown            :: boolean(), %% bit 12
	  navDataBootstrap      :: boolean(), %% bit 11
	  navDataDemoOnly       :: boolean(), %% bit 10
	  trimSucceeded         :: boolean(), %% bit 9
	  trimRunning           :: boolean(), %% bit 8

	  picVersionNumberOK    :: boolean(), %% bit 23
	  cutoutSystemDetected  :: boolean(), %% bit 22
	  ultrasonicSensorDeaf  :: boolean(), %% bit 21
	  tooMuchWind           :: boolean(), %% bit 20
	  angelsOutOufRange     :: boolean(), %% bit 19
	  notEnoughPower        :: boolean(), %% bit 18
	  timerElapsed          :: boolean(), %% bit 17
	  batteryTooHigh        :: boolean(), %% bit 16

	  emergency             :: boolean(), %% bit 31
	  communicationProblemOccurred :: boolean(), %% bit 30
	  adcWatchdogDelayed    :: boolean(), %% bit 29
	  controlWatchdogDelayed :: boolean(), %% bit 28
	  acquisitionThreadOn   :: boolean(), %% bit 27
	  videoThreadOn         :: boolean(), %% bit 26
	  navDataThreadOn       :: boolean(), %% bit 25
	  atCodedThreadOn       :: boolean()  %% bit 24
	}).


-record(state, {
	  socket    :: port(),
	  seq  = 1  :: integer(),
	  port = ?UDP_COMMAND_PORT :: inet:ip_port(),
	  ip   = ?DRONE_IP    :: inet:ip_address(),
	  queue = queue:new() :: queue(),
	  inhibit, %% undefined | reference()
	  wdt,     %% undefined | reference()
	  nav_state = #nav_state{} :: #nav_state{} 
	 }).

-record(ctrl, {
	  state,
	  battery,
	  pitch,
	  roll,
	  yaw,
	  altitude,
	  vx,
	  vy,
	  vz
	 }).

-record(vision, {
	  type,
	  xc,
	  yc,
	  width,
	  height,
	  dist 
	 }).



-define(blinkGreenRed, 0).
-define(blinkGreen, 1).
-define(blinkRed, 2).
-define(blinkOrange, 3).
-define(snakeGreenRed, 4).
-define(fire, 5).
-define(standard, 6).
-define(red, 7).
-define(green, 8).
-define(redSnake, 9).
-define(blank, 10).
-define(rightMissile, 11).
-define(leftMissile, 12).
-define(doubleMissile, 13).
-define(frontLeftGreenOthersRed, 14).
-define(frontRightGreenOthersRed, 15).
-define(rearRightGreenOthersRed, 16).
-define(rearLeftGreenOthersRed, 17).
-define(leftGreenRightRed, 18).
-define(leftRedRightGreen, 19).
-define(blinkStandard, 20).

-define(phiM30Deg, 0).
-define(phi30Deg, 1).
-define(thetaM30Deg, 2).
-define(theta30Deg, 3).
-define(theta20degYaw200deg, 4).
-define(theta20degYawM200deg, 5).
-define(turnaround, 6).
-define(turnaroundGodown, 7).
-define(yawShake, 8).
-define(yawDance, 9).
-define(phiDance, 10).
-define(thetaDance, 11).
-define(vzDance, 12).
-define(wave, 13).
-define(phiThetaMixed, 14).
-define(doublePhiThetaMixed, 15).
-define(flipAhead, 16).
-define(flipBehind, 17).
-define(flipLeft, 18).
-define(flipRight, 19).

-define(Q, $"). %% "

-define(REF_FLAG_EMERGENCY, (1 bsl 8)).
-define(REF_FLAG_TAKEOFF,   (1 bsl 9)).
-define(REF_FLAGS_DEFAULT,  
	((1 bsl 18) bor (1 bsl 20) bor (1 bsl 22) bor
	     (1 bsl 24) bor (1 bsl 28))).



-define(PCMD_FLAG_PROGRESSIVE, (1 bsl 0)).
-define(PCMD_FLAG_COMBINED_YAW, (1 bsl 1)).


%%%===================================================================
%%% API
%%%===================================================================

flattrim(Pid) when is_pid(Pid) ->
    gen_server:call(Pid, flatrim).

takeoff(Pid) ->
    ref(Pid, true, false).

land(Pid) ->
    ref(Pid, false, false).

emergency(Pid) ->
    ref(Pid, false, true).

ref(Pid,TakeOff,Emergency) when is_pid(Pid),
				is_boolean(TakeOff),
				is_boolean(Emergency) ->
    gen_server:call(Pid, {ref,TakeOff,Emergency}).

control(Pid, Options) when is_pid(Pid), is_list(Options) ->
    gen_server:call(Pid, {control,Options}).

calibrate(Pid, DeviceNum) when is_pid(Pid), is_list(DeviceNum) ->
    gen_server:call(Pid, {calibrate, DeviceNum}).

config(Pid, Name, Value) when is_pid(Pid), is_list(Name), is_list(Value) ->
    gen_server:call(Pid, {config,Name,Value}).
    
led_anim(Pid, Anim, Hz, Duration) when is_pid(Pid),
				       is_atom(Anim),
				       is_number(Hz),
				       is_integer(Duration) ->
    try led_animation_(Anim) of
	IAnim -> gen_server:call(Pid, {led_anim,IAnim,Hz,Duration})
    catch
	error:_ -> {error, enoent}
    end.

flight_anim(Pid, Anim, Duration) when is_pid(Pid),
				      is_atom(Anim),
				      is_integer(Duration) ->
    try flight_animation_(Anim) of
	IAnim -> gen_server:call(Pid, {flight_anim,IAnim,Duration})
    catch
	error:_ -> {error, enoent}
    end.

nav_data(Pid,Multicast,On) when is_pid(Pid), is_boolean(Multicast),
				is_boolean(On) ->
    gen_server:call(Pid, {nav_data,Multicast,On}).

get_control_data() ->
    get_control_data(?DRONE_IP).

get_control_data(IP) ->
    case gen_tcp:connect(IP, ?TCP_CONFIG_PORT, [{active,once},binary]) of
	{ok,S} -> 
	    io:format("control_data: connected\n", []),
	    load_control_data_(S, []);
	Error ->
	    Error
    end.

load_control_data_(S, Acc) ->
    receive
	{tcp, S, Data} ->
	    io:format("got data: ~p\n", [Data]),
	    inet:setopts(S, [{active,once}]),
	    load_control_data_(S, [Data|Acc]);
	{tcp_closed, S} ->
	    list_to_binary(lists:reverse(Acc));
	{tcp_error, S, Error} ->
	    {error, Error}
    end.

%%--------------------------------------------------------------------
%% @doc
%% Starts the server
%%
%% @spec start_link() -> {ok, Pid} | ignore | {error, Error}
%% @end
%%--------------------------------------------------------------------
start_link() ->
    gen_server:start_link({local, ?SERVER}, ?MODULE, [], []).

%%%===================================================================
%%% gen_server callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Initializes the server
%%
%% @spec init(Args) -> {ok, State} |
%%                     {ok, State, Timeout} |
%%                     ignore |
%%                     {stop, Reason}
%% @end
%%--------------------------------------------------------------------
init([]) ->
    case gen_udp:open(?UDP_NAVDATA_PORT, [binary,{active,true}]) of
	{ok,U} ->
	    %% option enable_nav_data here?
	    Wdt = erlang:start_timer(?WDT_TIME,self(),wdt),
	    {ok,#state { socket=U, wdt = Wdt }};
	Error ->
	    {stop, Error}
    end.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling call messages
%%
%% @spec handle_call(Request, From, State) ->
%%                                   {reply, Reply, State} |
%%                                   {reply, Reply, State, Timeout} |
%%                                   {noreply, State} |
%%                                   {noreply, State, Timeout} |
%%                                   {stop, Reason, Reply, State} |
%%                                   {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------
handle_call(Cmd=flattrim, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={ref,_TakeOff,_Emergency}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={control,_Options}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={calibrate,_DeviceNum}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={config,_Name,_Value}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={led_anim,_Anim,_Hz,_Duration}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call(Cmd={flight_anim,_Anim,_Duration}, _From, State) ->
    {reply, ok, transmit(enqueue(Cmd, State))};
handle_call({nav_data,Multicast,On}, _From, State) ->
    Data = 
	case {Multicast,On} of
	    {false,false} -> <<0,0:13/unit:8>>; %% ?
	    {false,true}  -> <<1,0:13/unit:8>>;
	    {true,false}  -> <<0:32/little>>;   %% guess
	    {true,true}   -> <<1:32/little>>
	end,
    R = gen_udp:send(State#state.socket,State#state.ip,
		     ?UDP_NAVDATA_PORT, Data),
    {reply, R, State};
handle_call(_Request, _From, State) ->
    {reply, {error, bad_call}, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling cast messages
%%
%% @spec handle_cast(Msg, State) -> {noreply, State} |
%%                                  {noreply, State, Timeout} |
%%                                  {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------
handle_cast(_Msg, State) ->
    {noreply, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling all non call/cast messages
%%
%% @spec handle_info(Info, State) -> {noreply, State} |
%%                                   {noreply, State, Timeout} |
%%                                   {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------
handle_info({timeout,Ref,inhibit}, State) when State#state.inhibit =:= Ref ->
    %% inhibit time terminated, transmit more
    State1 = transmit(State#state { inhibit = undefined }),
    {noreply,State1};

handle_info({timeout,Ref,wdt}, State) when State#state.wdt =:= Ref ->
    Wdt = erlang:start_timer(?WDT_TIME,self(),wdt),
    Seq = State#state.seq,
    Data = comwdg_(Seq),
    gen_udp:send(State#state.socket,
		 State#state.ip,State#state.port,Data),
    Seq1 = next_seq(Seq),
    State1 = State#state { wdt = Wdt, seq = Seq1 },
    {noreply, State1};

handle_info({udp,S,IP,_Port,Data}, State) 
  when State#state.socket =:= S, IP =:= State#state.ip ->
    case Data of
	<<?NAVDATA_MAGIC:32/little, NavBits:32/bitstring,
	  Sequence:32/little,_Unknown:32/little,Options/binary>> ->
	    %% Brutal C hack in the middle of the day.
	    NavBools  = [ (Bi =:= 1) || <<Bi:1>> <= NavBits ],
	    NavState = list_to_tuple([nav_state | NavBools]),
	    Opts = decode_opts(Options),
	    io:format("navdata: seq=~w, nav_state=~p, opts=~w\n",
		      [Sequence,nav_true_list(record_info(fields,nav_state),
					      NavBools), Opts]),
	    {noreply, State#state { nav_state = NavState }};
	_ ->
	    io:format("Got reply udp  = ~p\n", [Data]),
	    {noreply, State}
    end;

handle_info(_Info, State) ->
    io:format("Got info = ~p\n", [_Info]),
    {noreply, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called by a gen_server when it is about to
%% terminate. It should be the opposite of Module:init/1 and do any
%% necessary cleaning up. When it returns, the gen_server terminates
%% with Reason. The return value is ignored.
%%
%% @spec terminate(Reason, State) -> void()
%% @end
%%--------------------------------------------------------------------
terminate(_Reason, _State) ->
    ok.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Convert process state when code is changed
%%
%% @spec code_change(OldVsn, State, Extra) -> {ok, NewState}
%% @end
%%--------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

%%%===================================================================
%%% Internal functions
%%%===================================================================

nav_true_list([F|Fs], [true|Bs]) -> 
    [F | nav_true_list(Fs,Bs)];
nav_true_list([_F|Fs], [false|Bs]) -> 
    nav_true_list(Fs,Bs);
nav_true_list([], []) -> 
    [].

decode_opts(<<Tag:16/little, Len:16/little, Rest/binary>>) ->
    VLen = Len - 4,
    <<Val:VLen/binary, Rest1/binary>> = Rest,
    Decode = decode_option(Tag, Val),
    [Decode|decode_opts(Rest1)];
decode_opts(<<>>) ->
    [].

decode_option(?NAVDATA_DEMO_TAG, Value) ->
    V = decode_demo(Value),
    {demo, V};
decode_option(?NAVDATA_CKS_TAG, Value) ->
    {cks, Value};
decode_option(?NAVDATA_VISION_TAG, Value) ->
    V = decode_vision(Value),
    {vision, V };
decode_option(Tag, Value) ->
    {Tag,Value}.

decode_vision(<<_N:32/signed-little,Tail/binary>>) ->
    [#vision {type=Type,xc=Xc,yc=Yc,width=Width,height=Height,dist=Dist} ||
	<<Type:32/signed-little,
	  Xc:32/signed-little,
	  Yc:32/signed-little,
	  Width:32/signed-little,
	  Height:32/signed-little,
	  Dist:32/signed-little>> <= Tail ].

decode_demo(<<Ctrl:32/signed-little,
	      Battery:32/signed-little,
	      Pitch:32/float-little,
	      Roll:32/float-little,
	      Yaw:32/float-little,
	      Altitude:32/signed-little,
	      Vx:32/float-little,
	      Vy:32/float-little,
	      Vz:32/float-little>>) ->
    #ctrl { state=Ctrl, 
	    battery=Battery, 
	    pitch = Pitch / 1000,
	    roll  = Roll / 1000,
	    yaw   = Yaw / 1000,
	    altitude = Altitude / 1000, %% altitude is int -> float
	    vx = Vx / 1000,
	    vy = Vy / 1000,
	    vz = Vz / 1000 }.

next_seq(Seq) ->
    case (Seq+1) band 16#ffffffff of
	0 -> 1;
	Seq1 -> Seq1
    end.

enqueue(Cmd, State) ->
    Q = queue:in(Cmd, State#state.queue),
    State#state { queue = Q }.

transmit(State) ->
    if is_reference(State#state.inhibit) ->
	    State;
       true ->
	    case queue:out(State#state.queue) of
		{empty, _Q} ->
		    State;
		{{value,Cmd},Q} ->
		    Seq = State#state.seq,
		    Data = case Cmd of
			       flattrim ->
				   flattrim_(Seq);
			       {ref,TakeOff,Emergency} ->
				   ref_(Seq,TakeOff,Emergency);
			       {control,Options} ->
				   control_(Seq, Options);
			       {calibrate,DeviceNum} ->
				   calibrate_(Seq, DeviceNum);
			       {config,Name,Value} ->
				   config_(Seq,Name,Value);
			       {led_anim,Anim,Hz,Duration} ->
				   led_anim_(Seq,Anim,Hz,Duration);
			       {flight_anim,Anim,Duration} ->
				   flight_anim_(Seq,Anim,Duration)
			   end,
		    gen_udp:send(State#state.socket,
				 State#state.ip,State#state.port,Data),
		    Inhibit = erlang:start_timer(?INHIBIT_TIME,self(),inhibit),
		    Seq1 = next_seq(Seq),
		    State#state { seq = Seq1, queue = Q, inhibit = Inhibit }
	    end
    end.
		    



float_string(V) when is_float(V) ->
    <<F:32/signed>> = <<(float(V)):32/float>>,
    integer_to_list(F).

command_(Type,Seq,Args) ->
    Args1 = convert_args(Args),
    [<<"AT*">>,Type,$=,join([integer_to_list(Seq)|Args1], $,), $\r].

%% convert args to iolist
convert_args(Args) ->
    [ if is_integer(A) -> integer_to_list(A);
	 is_binary(A) -> A;
	 is_float(A) -> float_string(A);
	 is_list(A) -> [?Q,A,?Q]
      end || A <- Args].

join([A],_Sep) -> [A];
join([A|As],Sep) -> [A,Sep | join(As,Sep)];
join([],_Sep) -> [].

flattrim_(Seq) ->
    command_(<<"FTRIM">>,Seq,[]).

comwdg_(Seq) ->
    command_(<<"COMWDG">>, Seq, []).

ref_(Seq,TakeOff,Emergency) ->
    Flags = 
	(if TakeOff -> ?REF_FLAG_TAKEOFF; true -> 0 end) bor
	(if Emergency ->  ?REF_FLAG_EMERGENCY; true -> 0 end) bor
	?REF_FLAGS_DEFAULT,
    command_(<<"REF">>,Seq,[Flags]).

%% Roll left to right tilt  (-1 .. 1 )
%% Pitch front-back tilt (-1 .. 1)
%% Gaz Vertical speed (-1 .. 1)
%% Yaw Angular speed  (-1 .. 1)
%%
control_(Seq, Options) ->
    Args = control_args_(Options, 0, 0.0, 0.0, 0.0, 0.0),
    command_(<<"PCMD">>, Seq, Args).

control_args_([progressive|Args],Flags,Roll,Pitch,Gaz,Yaw) ->
    control_args_(Args,Flags bor ?PCMD_FLAG_PROGRESSIVE,Roll,Pitch,Gaz,Yaw);
control_args_([combined_yaw|Args],Flags,Roll,Pitch,Gaz,Yaw) ->
    control_args_(Args,Flags bor ?PCMD_FLAG_COMBINED_YAW,Roll,Pitch,Gaz,Yaw);
control_args_([{Key,Value}|Args],Flags,Roll,Pitch,Gaz,Yaw) 
  when is_number(Value), Value >= -1, Value =< 1 ->
    FValue = float(Value),
    case Key of
	roll -> control_args_(Args,Flags,FValue,Pitch,Gaz,Yaw);
	pitch -> control_args_(Args,Flags,Roll,FValue,Gaz,Yaw);
	gaz   -> control_args_(Args,Flags,Roll,Pitch,FValue,Yaw);
	yaw   -> control_args_(Args,Flags,Roll,Pitch,Gaz,FValue)
    end;
control_args_([],Flags,Roll,Pitch,Gaz,Yaw) ->
    [Flags,Roll,Pitch,Gaz,Yaw].
	    
calibrate_(Seq,DeviceNum) when is_integer(DeviceNum) ->
    command_(<<"CALIB">>, Seq, [integer_to_list(DeviceNum)]).

config_(Seq,Name,Value) when is_list(Name), is_list(Value) ->
    command_(<<"CONFIG">>, Seq, [ Name, Value ]).

led_anim_(Seq,Anim,Hz,Duration) when is_number(Hz), is_integer(Duration),
				    is_integer(Anim), Anim >= 0 ->
    Value = join(convert_args([Anim,float(Hz),Duration]), $,),
    %% the is also LEDS at command? maybe 1.0 only (try)
    config_(Seq, ?LEDS_ANIM, Value);
led_anim_(Seq,Name,Hz,Duration) when is_atom(Name), is_number(Hz) ->
    led_anim_(Seq, led_animation_(Name), Hz, Duration).


flight_anim_(Seq,Anim,Duration) when is_integer(Anim), is_integer(Duration),
				    Anim >= 0 ->
    Value = join(convert_args([Anim,Duration]), $,),
    config_(Seq, ?FLIGHT_ANIM, Value);
flight_anim_(Seq,Name,Duration) when is_atom(Name) ->
    flight_anim_(Seq, flight_animation_(Name), Duration).

led_animation_(Name) ->
    case Name of
     'blinkGreenRed' -> ?blinkGreenRed;
     'blinkGreen' -> ?blinkGreen;
     'blinkRed' -> ?blinkRed;
     'blinkOrange' -> ?blinkOrange;
     'snakeGreenRed' -> ?snakeGreenRed;
     'fire' -> ?fire;
     'standard' -> ?standard;
     'red' -> ?red;
     'green' -> ?green;
     'redSnake' -> ?redSnake;
     'blank' -> ?blank;
     'rightMissile' -> ?rightMissile;
     'leftMissile' -> ?leftMissile;
     'doubleMissile' -> ?doubleMissile;
     'frontLeftGreenOthersRed' -> ?frontLeftGreenOthersRed;
     'frontRightGreenOthersRed' -> ?frontRightGreenOthersRed;
     'rearRightGreenOthersRed' -> ?rearRightGreenOthersRed;
     'rearLeftGreenOthersRed' -> ?rearLeftGreenOthersRed;
     'leftGreenRightRed' -> ?leftGreenRightRed;
     'leftRedRightGreen' -> ?leftRedRightGreen;
     'blinkStandard' -> ?blinkStandard
    end.

led_animations() -> 
    [
     'blinkGreenRed',
     'blinkGreen',
     'blinkRed',
     'blinkOrange',
     'snakeGreenRed',
     'fire',
     'standard',
     'red',
     'green',
     'redSnake',
     'blank',
     'rightMissile',
     'leftMissile',
     'doubleMissile',
     'frontLeftGreenOthersRed',
     'frontRightGreenOthersRed',
     'rearRightGreenOthersRed',
     'rearLeftGreenOthersRed',
     'leftGreenRightRed',
     'leftRedRightGreen',
     'blinkStandard'
    ].

flight_animation_(Name) when is_atom(Name) ->
    case Name of
     'phiM30Deg' -> ?phiM30Deg;
     'phi30Deg' -> ?phi30Deg;
     'thetaM30Deg' -> ?thetaM30Deg;
     'theta30Deg' -> ?theta30Deg;
     'theta20degYaw200deg' -> ?theta20degYaw200deg;
     'theta20degYawM200deg' -> ?theta20degYawM200deg;
     'turnaround' -> ?turnaround;
     'turnaroundGodown' -> ?turnaroundGodown;
     'yawShake' -> ?yawShake;
     'yawDance' -> ?yawDance;
     'phiDance' -> ?phiDance;
     'thetaDance' -> ?thetaDance;
     'vzDance' -> ?vzDance;
     'wave' -> ?wave;
     'phiThetaMixed' -> ?phiThetaMixed;
     'doublePhiThetaMixed' -> ?doublePhiThetaMixed;
     'flipAhead' -> ?flipAhead;
     'flipBehind' -> ?flipBehind;
     'flipLeft' -> ?flipLeft;
     'flipRight' -> ?flipRight
    end.

flight_animations() -> 
    [
     'phiM30Deg',
     'phi30Deg',
     'thetaM30Deg',
     'theta30Deg',
     'theta20degYaw200deg',
     'theta20degYawM200deg',
     'turnaround',
     'turnaroundGodown',
     'yawShake',
     'yawDance',
     'phiDance',
     'thetaDance',
     'vzDance',
     'wave',
     'phiThetaMixed',
     'doublePhiThetaMixed',
     'flipAhead',
     'flipBehind',
     'flipLeft',
     'flipRight'
    ].
