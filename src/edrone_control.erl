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

-export([flat_trim/0, enable/0, disable/0, 
	 set_position/1, set_altitude/1,
	 set_pitch/1]).

-define(SERVER, ?MODULE). 

%% -define(PID_P, 0.8).
%% -define(PID_I, 1.0).
%% -define(PID_D, 0.1). 
-define(PID_P, 0.8).  %% 9 4
-define(PID_I, 1.0).
-define(PID_D, 0.1). 

-define(PITCH_RAMP, 1.0). %% Rampup/rampdown per second
-define(YAW_RAMP,   1.0). %% Rampup/rampdown per second
-define(ROLL_RAMP,  1.0). %% Rampup/rampdown per second
-define(ALT_RAMP,   4.0). %% Rampup/rampdown - cm / per second

-define(MOTOR_BASELINE, 0.4). %% Never let motors go under this level.

%% multiplier to apply to all motors to reach the given altitude.
-define(ALTITUDE_K, (1.0 - ?MOTOR_BASELINE) / 50.0). 

-record(st, { 
	  enabled = false,
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
	  ramp_pos = #position{},      %% Our position as is being ramped toward target_pos
	  target_pos = #position{},    %% Our desired target position
	  movement = #movement{},      %% Our current movement.
	  motor = {0.0, 0.0, 0.0, 0.0}, %% Motor throttling (0.0 - 1.0)
	  pidctl = { undefined, undefined, undefined, undefined },
	  pitch_lowpass = undefined,
	  roll_lowpass = undefined,
	  yaw_lowpass = undefined,
	  alt_lowpass = undefined,
	  glitch_count = -1 %% 0 == glitch every now and then.
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
%%    {ok, Uart} = edrone_navboard:init(),
    {ok, Uart} = edrone_navboard:init(),
    {ok, #st { 
       uart = Uart, 
       pwm_pid = Pid,
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

%% Specifiy all positions.
set_position(#position {} = Pos) ->
    gen_server:call(?MODULE, {set, Pos}).

%% Altitude. 
set_altitude(CM) when is_number(CM) ->
    gen_server:call(?MODULE, {set_altitude, CM}).

%% Altitude. 
set_pitch(A) when is_float(A), A =< 1.0, A >= 0.0->
    gen_server:call(?MODULE, {set_pitch, A}).
    
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

%% Setup a new target position/orientation that we should strive
%% to acheive.
%% Once we have arrived at the new position, an "arrived" 
%% message will be sent to the provided pid with the given arg
%%
handle_call({ set, #position { } = TargetPos}, _From, St) ->
    { reply, ok, St#st { target_pos = TargetPos } };

handle_call({ set_altitude, CM }, _From, St) ->
    { reply, ok, St#st { target_pos = (St#st.target_pos)#position { alt = CM } } };

handle_call({ set_pitch, A }, _From, St) ->
    { reply, ok, St#st { target_pos = (St#st.target_pos)#position { pitch = A } } };

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

handle_info(Info, St) ->
    io:format("handle_info(~p)??: ~p~n", [ St, Info ]),
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
    { ok, St }.
     


%%%===================================================================
%%% Internal functions
%%%===================================================================

%% 
process_nav_state(#st{ nav_ts = PrevTS, 
		       target_pos = TgtPos, 
		       ramp_pos = RPos } = St, 
		  #nav_state{ } = NavState) ->
    
    %%
    %% Timestamp delta (sec) between last nav state update, and this nav state
    %% 
    TSDelta = (NavState#nav_state.timestamp - PrevTS) / 1000000.0,

     {FilteredPitch, PitchLowPass} = lowpass_filter(St#st.pitch_lowpass, NavState#nav_state.ax),
     {FilteredRoll, RollLowPass}   = lowpass_filter(St#st.roll_lowpass, NavState#nav_state.ay),
     {FilteredYaw, YawLowPass}     = lowpass_filter(St#st.yaw_lowpass, NavState#nav_state.az),
     {FilteredAlt, AltLowPass}     = lowpass_filter(St#st.alt_lowpass, NavState#nav_state.az),

    %% {PitchLowPass, FilteredPitch} = {St#st.pitch_lowpass, NavState#nav_state.ax},
    %% {RollLowPass, FilteredRoll}   = {St#st.roll_lowpass, NavState#nav_state.ay},
    %% {YawLowPass, FilteredYaw}     = {St#st.yaw_lowpass, NavState#nav_state.az},
    %% {AltLowPass, FilteredAlt}     = {St#st.alt_lowpass, NavState#nav_state.az},




    %% Update our spatial position
    CurPos = #position {
      alt = FilteredAlt,
      pitch = FilteredPitch,
      roll = FilteredRoll,
      yaw = FilteredYaw
     },

    %%
    %% Calculate new ramp position, which wanders at a set max speed
    %% toward the target position.
    %%
    NRampPos = ramp_position(RPos, TgtPos, TSDelta),
    { M1, NPids } = calculate_motors(St#st.pidctl, CurPos, NRampPos),

    %% Set the motor speeds

    %% Artificially introduce a glitch
    {GlitchCount, M2} = 
	case glitch(St#st.glitch_count) of
	    -1 ->  %% Glitch deactivated.
		{ -1, edrone_motor:set_pwm_norm(St#st.pwm_pid, M1) };

	    0 -> %% Currently no glitch
		{ 0, edrone_motor:set_pwm_norm(St#st.pwm_pid, M1) };

      	    Cnt -> %% Glitch in progress
      		{ Cnt, edrone_motor:set_pwm_norm(St#st.pwm_pid, {0.1, 0.1, 0.1, 0.1}) }
      	end,
    

    St#st { motor = M2,
	    pidctl = NPids,
	    ramp_pos = NRampPos,
	    pitch_lowpass = PitchLowPass,
	    roll_lowpass = RollLowPass,
	    yaw_lowpass = YawLowPass,
	    alt_lowpass = AltLowPass, %% WILL NOT WORK. Sonar works at 26+CM only.
	    nav_ts = NavState#nav_state.timestamp, 
	    glitch_count = GlitchCount}.


glitch(-1) ->
    -1;

glitch(0) ->
    case random:uniform(100) of
	1 -> io:format("Glitch!~n"), 7;
	_ -> 0
    end;

glitch(Count) ->
    Count - 1.

calculate_motors({P0, P1, P2, P3}, CurPos, TgtPos) ->

    %% Calculate the current position of each corner of the drone.
    CM0 = cap(CurPos#position.pitch - CurPos#position.roll, -1.0, 1.0),
    CM1 = cap(CurPos#position.pitch + CurPos#position.roll, -1.0, 1.0),
    CM2 = cap(-CurPos#position.pitch - CurPos#position.roll, -1.0, 1.0),
    CM3 = cap(-CurPos#position.pitch + CurPos#position.roll, -1.0, 1.0),

    %% Calculate the desired position of each corner of the drone.
    TM0 = cap(TgtPos#position.pitch - TgtPos#position.roll, -1.0, 1.0),
    TM1 = cap(TgtPos#position.pitch + TgtPos#position.roll, -1.0, 1.0),
    TM2 = cap(-TgtPos#position.pitch - TgtPos#position.roll, -1.0, 1.0),
    TM3 = cap(-TgtPos#position.pitch + TgtPos#position.roll, -1.0, 1.0),

    io:format("P(~-7.4f) R(~-7.4f) CM(~-7.4f|~-7.4f|~-7.4f|~-7.4f) TM(~-7.4f|~-7.4f|~-7.4f|~-7.4f)",
	      [CurPos#position.pitch, CurPos#position.roll,
	       CM0, CM1, CM2, CM3,
	       TM0-CM0, TM1-CM1, TM2-CM2, TM3-CM3]),

    
    %% Set the new target point for each motor, and then calculate
    %% a motor output to add to the baseline.
    PidTS = edrone_pid:timestamp(),
    io:format("P("),
    { NM0, NP0 } = edrone_pid:update(edrone_pid:set_point(P0, TM0), CM0, PidTS), 
    { NM1, NP1 } = edrone_pid:update(edrone_pid:set_point(P1, TM1), CM1, PidTS), 
    { NM2, NP2 } = edrone_pid:update(edrone_pid:set_point(P2, TM2), CM2, PidTS), 
    { NM3, NP3 } = edrone_pid:update(edrone_pid:set_point(P3, TM3), CM3, PidTS), 
    io:format(") "),
    
    AltAdd = ?MOTOR_BASELINE + TgtPos#position.alt * ?ALTITUDE_K,
     io:format("| CA(~-7.4f), TA(~-7.4f)*K(~-7.4f)=~-7.4f",
     	      [ CurPos#position.alt,TgtPos#position.alt, ?ALTITUDE_K,
     		 TgtPos#position.alt * ?ALTITUDE_K]),

    io:format("~n"),
    {{ max(NM0 + AltAdd, ?MOTOR_BASELINE), 
       max(NM1 + AltAdd, ?MOTOR_BASELINE), 
       max(NM2 + AltAdd, ?MOTOR_BASELINE), 
       max(NM3 + AltAdd, ?MOTOR_BASELINE) },
     { NP0, NP1, NP2, NP3 }}.



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
       yaw = ramp_position_(RPos#position.yaw, TPos#position.yaw, ?YAW_RAMP, TSDelta),
       alt = ramp_position_(RPos#position.alt, TPos#position.alt, ?ALT_RAMP, TSDelta)
      }.

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp > Target + 0.0001->    
    Ramp - min(Max * TSDelta, (Ramp - Target) * TSDelta);

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp < Target - 0.0001 -> 
    Ramp + min(Max * TSDelta , (Target - Ramp) * TSDelta);

ramp_position_(Ramp, _Target, _Max, _TSDelta)  ->    
    Ramp.

cap(Val, Min, Max) ->
    min(max(Val, Min), Max).
