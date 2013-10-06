%%%-------------------------------------------------------------------
%%% @author magnus <magnus@t520>
%%% @copyright (C) 2013, magnus
%%% @doc
%%%
%%% @end
%%% Created : 24 Sep 2013 by magnus <magnus@t520>
%%%-------------------------------------------------------------------
-module(edrone_control).

-behaviour(gen_server).

%% API
-export([start_link/0]).
-include("nav.hrl").

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
	 terminate/2, code_change/3]).

-export([flat_trim/0, enable/0, disable/0]).

-define(SERVER, ?MODULE). 


%% 1.0    0.25   0.3  WORKS


-define(PID_P, 0.75). 
-define(PID_I, 0.25).
-define(PID_D, 0.2). 

-define(PITCH_RAMP, 0.8). %% Rampup/rampdown per second
-define(YAW_RAMP,   100.0). %% Rampup/rampdown per second
-define(ROLL_RAMP,  0.8). %% Rampup/rampdown per second

-define(MAX_ALT, 1000.0). %% Max altitude, in cm.

-define(ALT_RAMP,   6.0). %% Rampup/rampdown - cm / per second

-define(ACC_GYRO_SYNC_INTVL, 3000000). %% Sync gyro dead reconning to accelerometers every 3 seconds.

-define(UDP_PORT, 4509).

-define(PITCH_CMD,          16#0001).
-define(ROLL_CMD,           16#0002).
-define(YAW_CMD,            16#0003).
-define(THROTTLE_CMD,       16#0004).
-define(PITCH_TRIM_CMD,     16#0005).
-define(ROLL_TRIM_CMD,      16#0006).
-define(YAW_TRIM_CMD,       16#0007).
-define(UPGRADE_CMD,        16#0008).
-define(ALT_LOCK_CMD,       16#0009).
-define(MAX_CMD_VAL,        16#3FF). 


-define(HAVE_GLITCH,        -1). %% 0 = glitchy. -1 = not glitchy

-record(st, { 
	  udp_sock = undefined,
	  pitch_input = 0.0,
	  roll_input = 0.0,
	  yaw_input = 0.0,
	  throttle_input = 0.0,
	  pitch_trim_input = 0.0,
	  roll_trim_input = 0.0,
	  yaw_trim_input = 0.0,
	  enabled = false,
	  yaw_enabled = false, %% Don't try to do yaw before we are off the ground.
	  alt_lock_pid = undefined,  %% Only in use when we have altitude lock.
	  uart = undefined,
	  pwm_pid = undefined,
	  nav_recv_ref = undefined,
	  flat_trim = #flat_trim{ 
	    %% Flat trim for Magnus' drone
	    ax_offset=2058.5,
	    ay_offset=2034.34,
	    az_offset=2438.22,
	    gx_offset=-15.15,
	    gy_offset=2.16,
	    gz_offset=-14.62
	   },    %% Flat trim calibration data
	  nav_ts = 0,                  %% Timestamp of last reveived navboard data.
	  acc_gyro_sync_ts = 0,        %% Timestamp of last sync between gyro and acceleromoeter
	  ramp_pos = #position{},      %% Our position as is being ramped toward target_pos
	  target_pos = #position{},    %% Our desired target position
	  current_pos = #position{},    %% Our desired target position
	  motor = {0.0, 0.0, 0.0, 0.0}, %% Motor throttling (0.0 - 1.0)
	  pidctl = { undefined, undefined, undefined, undefined },
	  acc_lowpass = { undefined, undefined, undefined },
	  alt_lowpass = undefined,
	  gyro_lowpass = { undefined, undefined, undefined },
	  glitch_count = ?HAVE_GLITCH
	 }).



%%%===================================================================
%%% API
%%%===================================================================

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
%% @spec init(Args) -> {ok, St} |
%%                     {ok, St, Timeout} |
%%                     ignore |
%%                     {stop, Reason}
%% @end
%%--------------------------------------------------------------------
init([]) ->
    %% Setup navboard communication. 
    Pid = edrone_motor:start(),
    random:seed(),
    i2c:start(),
    edrone_bat:init(),
    
    io:format("~n~nBat: ~f~n~n", [edrone_bat:read()]),

    %% Use our target port as the source as well
    {ok, UDPSocket } = gen_udp:open(?UDP_PORT, [ binary ]), 

    {ok, Uart} = edrone_navboard:init(),
    {ok, #st { 
       uart = Uart, 
       pwm_pid = Pid,
       udp_sock = UDPSocket,
       pidctl = { 
	 edrone_pid:new(?PID_P, ?PID_I, ?PID_D, -1.0, 1.0), 
	 edrone_pid:new(?PID_P, ?PID_I, ?PID_D, -1.0, 1.0), 
	 edrone_pid:new(?PID_P, ?PID_I, ?PID_D, -1.0, 1.0), 
	 edrone_pid:new(?PID_P, ?PID_I, ?PID_D, -1.0, 1.0)
	} 
      }
    }.


flat_trim() ->
    gen_server:call(?MODULE, flat_trim).

enable() ->
    gen_server:call(?MODULE, enable).

disable() ->
    gen_server:call(?MODULE, flat_trim).

    
%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling call messages
%%
%% @spec handle_call(Request, From, St) ->
%%                                   {reply, Reply, St} |
%%                                   {reply, Reply, St, Timeout} |
%%                                   {noreply, St} |
%%                                   {noreply, St, Timeout} |
%%                                   {stop, Reason, Reply, St} |
%%                                   {stop, Reason, St}
%% @end
%--------------------------------------------------------------------

%% flat_trim must be called before we call enable
handle_call(flat_trim, _From, St) ->
    case edrone_navboard:flat_trim(St#st.uart) of
	{ ok, FlatTrim } -> { reply, ok, St#st { flat_trim = FlatTrim }};
	{ error, Reason }-> { reply, {error, Reason }, St };
	Unknown -> { reply, { error, Unknown }, St }
    end;
			     
%% We need a flat trim before we can enable system
handle_call(enable, _From, St) when St#st.flat_trim =:= undefined ->
    { reply, { error, no_flat_trim }, St };

%% Enable so that we get a message when a packet is read by uart.
handle_call(enable, _From, St) ->
    {ok, RecvRef } = edrone_navboard:enable_frame_report(St#st.uart),
    { reply, ok, St#st { enabled = true, 
			 nav_recv_ref = RecvRef,
			 nav_ts = edrone_lib:timestamp() } };

%% Disable the frame stream from the nav board
handle_call(disable, _From, St) when St#st.enabled =:= true->
    edrone:set_pwm_norm(St#st.pwm_pid, 0.0, 0.0, 0.0, 0.0),
    { reply, ok, St#st { enabled = false } };


handle_call(_Request, _From, St) ->
    Reply = ok,
    {reply, Reply, St}.
    


%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling cast messages
%%
%% @spec handle_cast(Msg, St) -> {noreply, St} |
%%                                  {noreply, St, Timeout} |
%%                                  {stop, Reason, St}
%% @end
%%--------------------------------------------------------------------
handle_cast(_Msg, St) ->
    {noreply, St}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling all non call/cast messages
%%
%% @spec handle_info(Info, St) -> {noreply, St} |
%%                                   {noreply, St, Timeout} |
%%                                   {stop, Reason, St}
%% @end
%%--------------------------------------------------------------------
handle_info({uart_async, Uart, RecvRef, Data} , St) 
  when St#st.uart =:= Uart, St#st.nav_recv_ref =:= RecvRef ->

    NSt = case edrone_navboard:decode_nav_data(Data) of 
	      { ok, #nav_frame{} = NavFrame } ->  
		  %% Process the nav frame to a nav state
		  NavState = edrone_navboard:process_nav_frame(NavFrame, St#st.flat_trim),

		  %% FIXME: INSERT SIGNAL FILTER CALL HERE

		  %% Process the position and return its new state.
		  process_nav_state(St, NavState);

	      %% Checksum error. Resync stream
	      { error, checksum } ->
		  case edrone_navboard:sync_stream(St#st.uart, 100) of
		      ok -> St;
		      { error, Err } -> 
			  io:format("FAILED TO SYNC NAVBOARD STREAM: ~p~n", [ Err] ),
			  St#st { enabled = false }
		  end
	  end,
    
    %% If we are still enabled, re-arm the uart to deliver the next frame.
    Ref = if NSt#st.enabled =:= true -> 
		  {ok, R } = edrone_navboard:enable_frame_report(NSt#st.uart),
		  R;
	     true -> undefined

	  end,
    { noreply, NSt#st { nav_recv_ref = Ref }};


	    
handle_info({uart_error, Uart, Reason} , St) when Uart =:= St#st.uart ->
    io:format("UART ERROR: ~p~n", [ Reason ]),
    { noreply, St#st {enabled = false} };

handle_info({udp, Sock, _, _, <<Cmd:6/unsigned, Val:10/unsigned>>} , St) 
  when Sock =:= St#st.udp_sock ->
    { noreply, decode_input(Cmd, Val, St) };

handle_info(Info, St) ->
    io:format("handle_info()??: ~p~n", [ Info ]),
    {noreply, St}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called by a gen_server when it is about to
%% terminate. It should be the opposite of Module:init/1 and do any
%% necessary cleaning up. When it returns, the gen_server terminates
%% with Reason. The return value is ignored.
%%
%% @spec terminate(Reason, St) -> void()
%% @end
%%--------------------------------------------------------------------
terminate(_Reason, _St) ->
    ok.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Convert process st when code is changed
%%
%% @spec code_change(OldVsn, St, Extra) -> {ok, NewSt}
%% @end
%%--------------------------------------------------------------------
%% Enable so that we get a message when a packet is read by uart.
code_change(_OldVsn, St, _Extra) when St#st.enabled =:= true->
    io:format("Code upgrade.~n"),
    { ok, St#st {glitch_count = ?HAVE_GLITCH }}.
     


%%%===================================================================
%%% Internal functions
%%%===================================================================

%% 
process_nav_state(#st{ nav_ts = PrevTS, 
		       current_pos = CurPos,
		       target_pos = TgtPos, 
		       acc_gyro_sync_ts = AccGyroSyncTS,
		       ramp_pos = RPos } = St, 
		  #nav_state{ } = NavState) ->
    
    %%
    %% Timestamp delta (sec) between last nav state update, and this nav state
    %% 
    TSDelta = (NavState#nav_state.timestamp - PrevTS) / 1000000.0,

    { AccPitchLP, AccRollLP, AccYawLP } = St#st.acc_lowpass,
    %% Accelerometer data
    { _AccPitch, NAccPitchLP } = lowpass_filter(AccPitchLP, NavState#nav_state.ax),
    { _AccRoll, NAccRollLP}   = lowpass_filter(AccRollLP, NavState#nav_state.ay),
    { _AccYaw, NAccYawLP}     = lowpass_filter(AccYawLP, NavState#nav_state.az),
    
    {Alt, NAltLP}     = lowpass_filter(St#st.alt_lowpass, NavState#nav_state.alt),

    %% Gyro data.
    { GyroPitchLP, GyroRollLP, GyroYawLP } = St#st.gyro_lowpass,
    %% { GyroPitch, NGyroPitchLP } = lowpass_filter(GyroPitchLP, NavState#nav_state.gx),
    %% { GyroRoll, NGyroRollLP }   = lowpass_filter(GyroRollLP, NavState#nav_state.gy),
    %% { GyroYaw, NGyroYawLP }     = lowpass_filter(GyroYawLP, -NavState#nav_state.gz),
    { GyroPitch, NGyroPitchLP } = { -NavState#nav_state.gy, GyroPitchLP },
    { GyroRoll, NGyroRollLP }   = { NavState#nav_state.gx, GyroRollLP },
    { GyroYaw, NGyroYawLP }     = { -NavState#nav_state.gz, GyroYawLP},

    %% Update our current spatial position
    NCurPos =#position {
      alt = Alt,
      pitch = CurPos#position.pitch + GyroPitch * TSDelta,
      roll = CurPos#position.roll + GyroRoll * TSDelta,
      yaw = CurPos#position.yaw + GyroYaw * TSDelta
     },

    %% Enable yaw only if altitude is greater than 30cm.
    %% Enable yaw if we see a "jump" in z accelerometer as we leave the ground.
    YawEnabled = 
	if St#st.yaw_enabled =:= true -> true;
	   St#st.throttle_input < 0.02 -> io:format("Yaw disabled~n"), false;
	   St#st.throttle_input > 0.4 -> io:format("Yaw enabled~n"), true;
	   true -> false
	end,

    NTgtPos = 
	if YawEnabled =:= true ->
		TgtPos#position { yaw = TgtPos#position.yaw + St#st.yaw_input * TSDelta };
	   true -> TgtPos#position { yaw = NCurPos#position.yaw } %% Let target follow current
	end,

    %%
    %% Calculate new ramp position, which wanders at a set max speed
    %% toward the target position.
    %%
    NRampPos = ramp_position(RPos, NTgtPos, TSDelta),

    { NAltLockPid, M1, NPids } = calculate_motors(St#st.pidctl, NCurPos, NRampPos,
						  St#st.throttle_input, St#st.alt_lock_pid),
    
    %%
    %% Cut off the motors if we are under 0.02 throttle
    %%
    M2 = if St#st.throttle_input < 0.02 -> { 0.0, 0.0, 0.0, 0.0 };
	    true -> M1
	 end,
    %% Artificially introduce a glitch
    {GlitchCount, M3} = 
	case glitch(St#st.glitch_count) of
	    -1 ->  %% Glitch deactivated.
		{ -1, edrone_motor:set_pwm_norm(St#st.pwm_pid, M2) };

	    0 -> %% Currently no glitch
		{ 0, edrone_motor:set_pwm_norm(St#st.pwm_pid, M2) };

      	    Cnt -> %% Glitch in progress
      		{ Cnt, edrone_motor:set_pwm_norm(St#st.pwm_pid, {0.01, 0.01, 0.01, 0.01}) }
      	end,
    

    St#st { motor = M3,
	    pidctl = NPids,
	    alt_lock_pid = NAltLockPid,
	    yaw_enabled = YawEnabled,
	    ramp_pos = NRampPos,
	    current_pos = NCurPos,
	    target_pos = NTgtPos,
	    gyro_lowpass = { NGyroPitchLP, NGyroRollLP, NGyroYawLP },
	    acc_lowpass = { NAccPitchLP, NAccRollLP, NAccYawLP },
	    alt_lowpass = NAltLP, %% WILL NOT WORK. Sonar works at 26+CM only.
	    nav_ts = NavState#nav_state.timestamp, 
	    acc_gyro_sync_ts = AccGyroSyncTS, %% Not used yet. 
	    glitch_count = GlitchCount}.


glitch(-1) ->
    -1;

glitch(0) ->
    case random:uniform(100) of
	1 -> io:format("Glitch!~n"), 10;
	_ -> 0
    end;

glitch(Count) ->
    Count - 1.

calculate_motors({P0, P1, P2, P3}, CurPos, TgtPos, Throttle, AltLockPid) ->

    YawDelta = TgtPos#position.yaw - CurPos#position.yaw, 
    %% Calculate the current position of each corner of the drone.
    CM0 = cap(CurPos#position.pitch + CurPos#position.roll, -1.0, 1.0),
    CM1 = cap(CurPos#position.pitch - CurPos#position.roll, -1.0, 1.0),
    CM2 = cap(-CurPos#position.pitch - CurPos#position.roll, -1.0, 1.0),
    CM3 = cap(-CurPos#position.pitch + CurPos#position.roll, -1.0, 1.0),

    %% Calculate the desired position of each corner of the drone.
    TM0 = cap(TgtPos#position.pitch + TgtPos#position.roll - YawDelta, -1.0, 1.0),
    TM1 = cap(TgtPos#position.pitch - TgtPos#position.roll + YawDelta, -1.0, 1.0),
    TM2 = cap(-TgtPos#position.pitch - TgtPos#position.roll - YawDelta, -1.0, 1.0),
    TM3 = cap(-TgtPos#position.pitch + TgtPos#position.roll + YawDelta, -1.0, 1.0),

      %% io:format("P(~-7.4f|~-7.4f) R(~-7.4f|~-7.4f) Yaw(~-7.4f|~-7.4f) CM(~-7.4f|~-7.4f|~-7.4f|~-7.4f) TM(~-7.4f|~-7.4f|~-7.4f|~-7.4f)~n",
      %% 	       [  CurPos#position.pitch, TgtPos#position.pitch,
      %% 		  CurPos#position.roll,TgtPos#position.roll,
      %% 		  TgtPos#position.yaw, CurPos#position.yaw, 
      %% 		  CM0, CM1, CM2, CM3,
      %% 		  TM0-CM0, TM1-CM1, TM2-CM2, TM3-CM3]),

    

    %% Set the new target point for each motor, and then calculate
    %% a motor output to add to the baseline.
    PidTS = edrone_pid:timestamp(),
%%    io:format("P("),
    { NM0, NP0 } = edrone_pid:update(edrone_pid:set_point(P0, TM0), CM0, PidTS), 
    { NM1, NP1 } = edrone_pid:update(edrone_pid:set_point(P1, TM1), CM1, PidTS), 
    { NM2, NP2 } = edrone_pid:update(edrone_pid:set_point(P2, TM2), CM2, PidTS), 
    { NM3, NP3 } = edrone_pid:update(edrone_pid:set_point(P3, TM3), CM3, PidTS), 
%%    io:format(") "),
%%     io:format("~n"),

    %% If altitude lock is off, we just use throttle (0.0 - 1.0)
    %% If altitude lock is set, we use a proportional adjuster, with caps
    { AltAdd, NAltLockPid } =
	case AltLockPid of
	    undefined -> { 0.0 , undefined };
	    _ -> edrone_pid:update(AltLockPid, CurPos#position.alt / ?MAX_ALT, PidTS)
	end,
		
    { NAltLockPid, 
      { NM0 + AltAdd + Throttle,
	NM1 + AltAdd + Throttle, 
	NM2 + AltAdd + Throttle, 
	NM3 + AltAdd + Throttle},
      { NP0, NP1, NP2, NP3 }
    }.


lowpass_filter({V0, V1, V2}, Val) ->
    NV = 
	V0 * 0.25 +
	V1 * 0.25 +
	V2 * 0.25 +
	%% V3 * 0.16666666 +
	%% V4 * 0.16666666 +
	Val * 0.25,

    {NV, { V1, V2, Val } };

lowpass_filter(_, Val) ->
    {Val, { Val, Val, Val } }.
    

ramp_position(RPos, TPos, TSDelta) ->
    RPos#position {
      pitch = ramp_position_(RPos#position.pitch, TPos#position.pitch, ?PITCH_RAMP, TSDelta),
      roll = ramp_position_(RPos#position.roll, TPos#position.roll, ?ROLL_RAMP, TSDelta),
      yaw = ramp_position_(RPos#position.yaw, TPos#position.yaw, ?YAW_RAMP, TSDelta)
      %% Altitude is not part of ramp position.
     }.

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp > Target + Max * TSDelta->    
%%    io:format("-T(~f) R(~f) TSDelta(~f) Max(~f)~n", [Target, Ramp, TSDelta, Max]),
    Ramp - Max * TSDelta;

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp < Target - Max * TSDelta -> 
%%    io:format("+T(~f) R(~f) TSDelta(~f) Max(~f)~n", [Target, Ramp, TSDelta, Max]),
    Ramp + Max * TSDelta;

ramp_position_(_Ramp, Target, _Max, _TSDelta)  ->    
%%    io:format("-=T(~f) R(~f) TSDelta(~f) Max(~f) ~f~n", [Target, Ramp, TSDelta, Max, TSDelta * Max]),
    Target.

cap(Val, Min, Max) ->
    min(max(Val, Min), Max).

decode_input(?PITCH_CMD, Val, #st {target_pos = TgtPos} = St) -> 
    Inp = convert_input_value(Val),
     %% io:format("Pitch: ~-7.4f~n", [Inp]),
    St#st { pitch_input = Inp, target_pos = TgtPos#position {pitch = Inp / 10.0}};
    
decode_input(?ROLL_CMD, Val, #st {target_pos = TgtPos} = St) -> 
    Inp = convert_input_value(Val),
     %% io:format("               Roll: ~-7.4f~n", [Inp]),
    St#st { roll_input = Inp, target_pos = TgtPos#position {roll = Inp / 10.0}};
    

decode_input(?YAW_CMD, Val, St) -> 
    Inp = convert_input_value(Val),
%%    io:format("                             Yaw: ~p/~-7.4f~n", [Val,Inp]),
    St#st { yaw_input = Inp / 3};

decode_input(?THROTTLE_CMD, Val, St) when St#st.alt_lock_pid =:= undefined-> 
    Inp = Val / ?MAX_CMD_VAL,
    %% io:format("                                            Throttle: ~-7.4f~n", [Inp]),
    St#st { throttle_input = Inp };


%% Don't react to throttle when in alt lock mode
decode_input(?THROTTLE_CMD, _Val, St) -> 
    St;

decode_input(?PITCH_TRIM_CMD, Val, St) -> 
    Inp = convert_input_value(Val),
    io:format("---- PitchTrim: ~-7.4f~n", [Inp]),
    St#st { pitch_trim_input = Inp };

decode_input(?ROLL_TRIM_CMD, Val, St) ->
    Inp = convert_input_value(Val),
    io:format("---- RollTrim: ~-7.4f~n", [Inp]),
    St#st { roll_trim_input = Inp };

decode_input(?YAW_TRIM_CMD, Val, St) ->
    Inp = convert_input_value(Val),
    io:format("---- YawTrim: ~-7.4f~n", [Inp]),
    St#st { roll_trim_input = Inp };

decode_input(?UPGRADE_CMD, _Val, St) ->
    io:format("UPGRADE TIME!~n"),
    gen_server:cast(edrone_upgrade, { upgrade, ?MODULE }),
    St;

decode_input(?ALT_LOCK_CMD, 1, St) ->
    io:format("Altitude Lock: Engaged!~n"),
    AltPid = edrone_pid:set_point(edrone_pid:new(?PID_P, ?PID_I, ?PID_D, -1.0, 1.0),
				 (St#st.current_pos)#position.alt / ?MAX_ALT), %% Alttitude lock    
    St#st { alt_lock_pid = AltPid };



decode_input(?ALT_LOCK_CMD, 0, St) ->
    io:format("Altitude Disengaged!~n"),
    St#st { alt_lock_pid = undefined };

decode_input(ErrKey, ErrVal, St)-> 
    io:format("---- Unknown key(~p) val(~p)~n", [ErrKey, ErrVal]),
    St.
    
convert_input_value(Val) ->
    (Val / ?MAX_CMD_VAL) * 2 - 1.


