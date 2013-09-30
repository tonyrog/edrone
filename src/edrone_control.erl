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

-define(PITCH_PID_P, 1.0).
-define(PITCH_PID_I, 0.0).
-define(PITCH_PID_D, 0.2). 

-define(ROLL_PID_P, 0.9).
-define(ROLL_PID_I, 0.04).
-define(ROLL_PID_D, 0.0).

-define(YAW_PID_P, 0.9).
-define(YAW_PID_I, 0.04).
-define(YAW_PID_D, 0.0).

-define(ALT_PID_P, 0.9).
-define(ALT_PID_I, 0.04).
-define(ALT_PID_D, 0.0).

-define(MOTOR_BASE, 0.5).
-define(PITCH_RAMP, 1.0). %% Rampup/rampdown per second
-define(YAW_RAMP,   0.01). %% Rampup/rampdown per second
-define(ROLL_RAMP,  0.01). %% Rampup/rampdown per second
-define(ALT_RAMP,   0.01). %% Rampup/rampdown per second

-record(st, { 
	  enabled = false,
	  uart = undefined,
	  pwm_pid = undefined,
	  nav_recv_ref = undefined,
	  flat_trim = #flat_trim{},    %% Flat trim calibration data
	  pos = #position {},          %% Our current, measured position.
	  ramp_pos = #position{},      %% Our position as is being ramped toward target_pos
	  target_pos = #position{},    %% Our desired target position
	  movement = #movement{},      %% Our current movement.
	  motor = {0.0, 0.0, 0.0, 0.0}, %% Motor throttling (0.0 - 1.0)
	  pitch_pidctl = undefined,
	  roll_pidctl = undefined,
	  yaw_pidctl = undefined,
	  alt_pidctl = undefined,
	  pitch_lowpass = undefined,
	  roll_lowpass = undefined,
	  yaw_lowpass = undefined,
	  alt_lowpass = undefined
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
    
    io:format("Bat: ~f", [edrone_bat:read()]),
    {ok, Uart} = edrone_navboard:init(),
    {ok, #st { 
       uart = Uart, 
       pwm_pid = Pid,
       pitch_pidctl = edrone_pid:new(?PITCH_PID_P, ?PITCH_PID_I, ?PITCH_PID_D, -1.0, 1.0),
       roll_pidctl = edrone_pid:new(?ROLL_PID_P, ?ROLL_PID_I, ?ROLL_PID_D, -1.0, 1.0),
       yaw_pidctl = edrone_pid:new(?YAW_PID_P, ?YAW_PID_I, ?YAW_PID_D, -1.0, 1.0),
       alt_pidctl = edrone_pid:new(?ALT_PID_P, ?ALT_PID_I, ?ALT_PID_D, -1.0, 1.0)
      } }.


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
%%--------------------------------------------------------------------

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
			 pos = #position { timestamp = edrone_lib:timestamp() } }};


%% Disable the frame stream from the nav board
handle_call(disable, _From, St) when St#st.enabled =:= true->
    edrone:set_pwm(St#st.pwm_pid, 0.0, 0.0, 0.0, 0.0),
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
handle_info({uart_async, Uart, RecvRef, Data} , St) when St#st.uart =:= Uart, St#st.nav_recv_ref =:= RecvRef ->
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
    Ref = if 
	NSt#st.enabled =:= true -> 
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
    {ok, RecvRef } = edrone_navboard:enable_frame_report(St#st.uart),
    { ok, St#st { nav_recv_ref = RecvRef } }.


%%%===================================================================
%%% Internal functions
%%%===================================================================

%% 
process_nav_state(#st{ pos = CurPos, 
		       target_pos = TgtPos, 
		       ramp_pos = RPos } = St, 
		  #nav_state{ } = NavState) ->
    
    %%
    %% Timestamp delta (sec) between last nav state update, and this nav state
    %% 
    TSDelta = (NavState#nav_state.timestamp - CurPos#position.timestamp) / 1000000.0,

    %% Shift the ramp position toward target position.


    %% We are not using movement yet, so leave it out.
    {FilteredPitch, PitchLowPass}  = lowpass_filter(St#st.pitch_lowpass, NavState#nav_state.ax),
    %% io:format("Pitch(~-7.4f / ~-7.4f) ", [ NavState#nav_state.ax, FilteredPitch ]),
    {_FilteredRoll, RollLowPass}   = lowpass_filter(St#st.roll_lowpass, NavState#nav_state.ay),
    {_FilteredYaw, YawLowPass}     = lowpass_filter(St#st.yaw_lowpass, NavState#nav_state.az),
    {_FilteredAlt, AltLowPass}     = lowpass_filter(St#st.alt_lowpass, NavState#nav_state.az),

    %% Update our spatial position
    NCurPos = #position {
      alt = NavState#nav_state.alt, 
      pitch = FilteredPitch,
      roll = NavState#nav_state.ay,
      yaw = NavState#nav_state.az,
      timestamp = NavState#nav_state.timestamp
     },

    %%
    %% Calculate new ramp position, and set its points in the pid controllers.
    %%
    NRampPos   = ramp_position(RPos, TgtPos, TSDelta),

    NPitchPid1 = edrone_pid:set_point(St#st.pitch_pidctl, NRampPos#position.pitch),
    NRollPid1  = edrone_pid:set_point(St#st.roll_pidctl, NRampPos#position.roll),
    NYawPid1   = edrone_pid:set_point(St#st.yaw_pidctl, NRampPos#position.yaw),
    NAltPid1   = edrone_pid:set_point(St#st.alt_pidctl, NRampPos#position.alt),

    PidTS = edrone_pid:timestamp(),

    %% Mix in motor adjustments for pitch, using the appropriate pid controller.
    { M1, NPitchPid2 } = mixin(St#st.motor, 
			       NPitchPid1,
			       NCurPos#position.pitch, 
			       NRampPos#position.pitch, PidTS),

    %% io:format("~n"),
    {DBG, _, _, _ } = M2 = edrone_motor:set_pwm(St#st.pwm_pid, M1),

    %% { M1_0, M1_1, M1_2, M1_3 } = M1,
    %% M2 = edrone_motor:clip_pwm(M1_0, M1_1, M1_2, M1_3), %% Don't actually run motor
    
    io:format("CP(~-7.4f) TP(~-7.4f) RP(~-7.4f) M(~-7.4f)~n", 
	      [FilteredPitch, 
	       TgtPos#position.pitch,
	       NRampPos#position.pitch,
	       DBG]),

    St#st { motor = M2,
	    ramp_pos = NRampPos,
	    pitch_pidctl = NPitchPid2,
	    roll_pidctl = NRollPid1,
	    yaw_pidctl = NYawPid1,
	    alt_pidctl = NAltPid1, %% WILL NOT WORK. Sonar works at 26+CM only.
	    pitch_lowpass = PitchLowPass,
	    roll_lowpass = RollLowPass,
	    yaw_lowpass = YawLowPass,
	    alt_lowpass = AltLowPass, %% WILL NOT WORK. Sonar works at 26+CM only.
	    pos = NCurPos}.



mixin({M0, M1, M2, M3}, PidCtl, CurPos, TgtPos, _PidTS) 
  when CurPos =:= TgtPos->
    { {M0, M1, M2, M3}, PidCtl }; %% We have the position


%%
%% Mix  adjustments for motors.
%%
mixin({_M0_, _M1, M2, M3}, PidCtl, CurPos, _TgtPos, PidTS) ->
    %% FIXME: Harmonize timestamp usage across entire system 
    %%        edrone_pid:timestamp() vs. edrone_lib:timestamp().
    { NVal, NPidCtl } = edrone_pid:update(PidCtl, CurPos, PidTS), 

     %% io:format(" Cur(~-6.4f) Tgt(~-6.4f) -> NVal(~-6.4f) ",
     %%  	      [ CurPos, TgtPos, NVal]),
    %% Throttle back rear motors, throttle up front motors with the given adjustments
    { {?MOTOR_BASE + NVal, ?MOTOR_BASE + NVal, M2, M3}, NPidCtl }.


dbg_nav(NS) ->
    io:format("nav:  Acc(g)=   {x(~-6.4f) y(~-6.4f) z(~-6.4f)}         Gyro(deg)={x(~-6.4f) y(~-6.4f) z(~-6.4f)} Alt(cm)=~6.4f Comp(deg)={x(~-6.4f) y(~-6.4f) z(~-6.4f)}", 
	      [ NS#nav_state.ax, NS#nav_state.ay, NS#nav_state.az, 
		NS#nav_state.gx, NS#nav_state.gy, NS#nav_state.gz, 
		NS#nav_state.alt,
		NS#nav_state.mx, NS#nav_state.my, NS#nav_state.mz ]).

dbg_movement(M) ->
    io:format("move: X(cm/sec)={x(~-6.4f) y(~-6.4f) alt(~-6.4f)} orient(deg/sec)={roll(~-6.4f) pitch(~-6.4f) yaw(~-6.4f)}", 
	      [ M#movement.x_spd, M#movement.y_spd, M#movement.alt_spd,
		M#movement.roll_spd, M#movement.pitch_spd, M#movement.yaw_spd ]).


dbg_position(P) ->
    io:format("pos:  X(cm)=    {x(~-6.4f) y(~-6.4f) alt(~-6.4f)}     orient(deg)={roll(~-6.4f) pitch(~-6.4f) yaw(~-6.4f)}", 
	      [ P#position.x, P#position.y, P#position.alt,
		P#position.roll, P#position.pitch, P#position.yaw ]).

lowpass_filter({V0, V1, V2, V3, V4}, Val) ->
    NV = 
	V0 * 0.16666666 +
	V1 * 0.16666666 +
	V2 * 0.16666666 +
	V3 * 0.16666666 +
	V4 * 0.16666666 +
	Val * 0.16666666,

    {NV, { V1, V2, V3, V4, Val } };

lowpass_filter(_, Val) ->
    {Val, { Val, Val, Val, Val, Val } }.
    

ramp_position(RPos, TPos, TSDelta) ->
    RPos#position {
       pitch = ramp_position_(RPos#position.pitch, TPos#position.pitch, ?PITCH_RAMP, TSDelta),
       roll = ramp_position_(RPos#position.roll, TPos#position.roll, ?ROLL_RAMP, TSDelta),
       yaw = ramp_position_(RPos#position.yaw, TPos#position.yaw, ?YAW_RAMP, TSDelta),
       alt = ramp_position_(RPos#position.alt, TPos#position.alt, ?ALT_RAMP, TSDelta)
      }.

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp > Target + 0.0001->    

    %% io:format("| -(R(~p) - T(~p)) * TSDelta(~-6.4f) = ~-6.4f / ~-6.4f ", 
    %% 	      [ Ramp, Target, TSDelta, 
    %% 		Ramp - min(Max * TSDelta , (Ramp - Target) * TSDelta), 
    %% 		Max * TSDelta]),

    Ramp - min(Max * TSDelta, (Ramp - Target) * TSDelta);

ramp_position_(Ramp, Target, Max, TSDelta) when Ramp < Target - 0.0001 -> 
    %% io:format("| +(T(~-6.4f) - R(~-6.4f)) * TSDelta(~-6.4f) = ~-6.4f / ~-6.4f ", 
    %% 	      [ Target, Ramp, TSDelta, 
    %% 		Ramp + min(Max * TSDelta , (Target - Ramp) * TSDelta), Max * TSDelta]),
    Ramp + min(Max * TSDelta , (Target - Ramp) * TSDelta);

ramp_position_(Ramp, _Target, _Max, _TSDelta)  ->    
    Ramp.
