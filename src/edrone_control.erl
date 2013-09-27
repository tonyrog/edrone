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

-export([flat_trim/0, enable/0, disable/0, move_to/1, move_to_altitude/1]).

-define(SERVER, ?MODULE). 

-record(st, { 
	  enabled = false,
	  uart = undefined,
	  pwm_pid = undefined,
	  nav_recv_ref = undefined,
	  flat_trim = #flat_trim{},    %% Flat trim calibration data
	  pos = #position {},          %% Our current position
	  target_pos = #position{},    %% Our desired target position
	  movement = #movement{},      %% Our current movement.
	  motor = {0.0, 0.0, 0.0, 0.0} %% Motor throttling (0.0 - 1.0)
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
    {ok, #st { uart = Uart, pwm_pid = Pid } }.


flat_trim() ->
    gen_server:call(?MODULE, flat_trim).

enable() ->
    gen_server:call(?MODULE, enable).

disable() ->
    gen_server:call(?MODULE, flat_trim).

%% Specifiy all positions.
move_to(#position {} = Pos) ->
    gen_server:call(?MODULE, {move_to, Pos}).

%% Altitude. 
move_to_altitude(CM) ->
    gen_server:call(?MODULE, {move_to_altitude, CM}).
    
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
    { reply, ok, St#st { enabled = true, nav_recv_ref = RecvRef } };


%% Disable the frame stream from the nav board
handle_call(disable, _From, St) when St#st.enabled =:= true->
    edrone:set_pwm(St#st.pwm_pid, 0.0, 0.0, 0.0, 0.0),
    { reply, ok, St#st { enabled = false } };

%% Setup a new target position/orientation that we should strive
%% to acheive.
%% Once we have arrived at the new position, an "arrived" 
%% message will be sent to the provided pid with the given arg
%%
handle_call({ move_to, #position { } = TargetPos}, _From, St) ->
    { reply, ok, St#st { target_pos = TargetPos } };

handle_call({ move_to_altitude, CM }, _From, St) ->
    { reply, ok, St#st { target_pos = (St#st.target_pos)#position { alt = CM } } };

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
code_change(_OldVsn, St, _Extra) ->
    {ok, St}.

%%%===================================================================
%%% Internal functions
%%%===================================================================

%% 
process_nav_state(#st{ pos = CurPos, target_pos = TgtPos, movement = CurMove } = St, 
		  #nav_state{ } = NavState) ->
    
    %% Timestamp delta (usec) between last nav state update, and this nav state
    %% 
    TSDelta = (NavState#nav_state.timestamp - CurPos#position.timestamp) / 1000000.0,
    


    %% Update our movement record.
    NewMove = #movement { 
      %% Set altitude speed (up = positive, down = negative) in cm/sec.
      alt_spd = (NavState#nav_state.alt - CurPos#position.alt) / TSDelta,
      x_spd = CurMove#movement.x_spd + NavState#nav_state.ax,
      y_spd = CurMove#movement.y_spd + NavState#nav_state.ay,

      roll_spd = NavState#nav_state.gx,
      pitch_spd = NavState#nav_state.gy,
      yaw_spd = NavState#nav_state.gz
     },

    %% Update our spatial position
    NewPos = #position {
      alt = NavState#nav_state.alt, 
      pitch = NavState#nav_state.ax,
      roll = NavState#nav_state.ay,
      yaw = NavState#nav_state.mx,
      timestamp = NavState#nav_state.timestamp
     },


    %% TmpT1 = mixin_altitude(St#st.motor, NavState, NewPos, TgtPos), %% Nil op for now
    %% TmpT2 = mixin_roll(TmpT1, NavState, NewPos, TgtPos),
    %% TmpT3 = mixin_yaw(TmpT2, NavState, NewPos, TgtPos),
    
    { M0, M1, M2, M3 } = mixin_pitch(St#st.motor, NavState, NewPos, TgtPos),

    { T0, T1, T2, T3 } = edrone_motor:set_pwm(St#st.pwm_pid, M0, M1, M2, M3),
%%    ClippedThrottle = edrone_motor:clip_pwm(M0, M1, M2, M3),
    
    %% ugly slowdown of output
    %% Rand = random:uniform(),

    %% if Rand < 0.1 ->
    %% 	    io:format("\e[H\e[2JTSDelta: ~p~n", 
    %% 		      [TSDelta * 1000.0]),

%%	    dbg_nav(NavState),
    %% dbg_position(NewPos), 
%%    dbg_movement(NewMove), io:format("~n"),
%%	    dbg_position(NewPos),
%%	    io:format("Throttle: ~p ~n", [ Throttle ]),
%%	    true;
%%       true -> true
%%    end,
    St#st { motor = { T0, T1, T2, T3 },
	    target_pos = TgtPos#position { pitch = 0.03},
	    pos = NewPos,
	    movement = NewMove }.



mixin_pitch(M, _NavState, Pos, TPos) when Pos#position.pitch =:= TPos#position.pitch->
    M;  %% We have the desired pitch

%%
%% Mix in pitch adjustments for motors.
%%
%% We do this by calculating a desired value for our pitch roll, which is based
%% on how far away our current pitch angle is from the target pitch angle.
%% 
%% If our current roll rate (as reported by the x-axis gyro) is too low, we 
%% add throttle with an increment that approaches zero as we get closer to
%% our desired roll rate.
%%
mixin_pitch({M0, M1, M2, M3}, NavState, CPos, TPos) ->
    
    %% How far away are we from our target pitch angle? (-1.0..1.0) 
    TargetRollRate = distance(CPos#position.pitch, TPos#position.pitch),

    %% How far away is our current roll rate from the target roll rate.
    %%RollRateDistance = distance(NavState#nav_state.gy, TargetRollRate),

    %% Calculate the roll rate adjustments to be made to the motors
    %% FIXME: Configurable roll rate adjustment divider
    
    Adj = cap(TargetRollRate / 30.0, -0.0003 , 0.0003),
    io:format("CPitch(~-10.4f) TPitch(~-10.4f) -> TRollRate(~-10.4f) -> MotorAdj(~-10.4f) acc_x(~10.4f) roll_rate(~10.f4)~n",
     	      [ CPos#position.pitch, TPos#position.pitch,
     		TargetRollRate,  
     		Adj,NavState#nav_state.ax, NavState#nav_state.gx]),
    %% Throttle back rear motors, throttle up front motors with the given adjustments
%%    { M0 + Adj, M1 + Adj, M2 - Adj, M3 - Adj }.
    { M0 + Adj, M1 + Adj, M2, M3 }.


%% Nil op for now
mixin_roll(M, _NavState, _CPos, _TPos) -> M.

%% Nil op for now
mixin_yaw(M, _NavState, _CPos, _TPos) -> M. 

mixin_altitude(M, _NavState, _CPos, _TPos) -> M.

%% FIXME: 1/X Formula?
distance(Current, Target)  ->
    Target - Current.


dbg_nav(NS) ->
    io:format("nav:  Acc(g)=   {x(~-10.4f) y(~-10.4f) z(~-10.4f)}         Gyro(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)} Alt(cm)=~10.4f Comp(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)}", 
	      [ NS#nav_state.ax, NS#nav_state.ay, NS#nav_state.az, 
		NS#nav_state.gx, NS#nav_state.gy, NS#nav_state.gz, 
		NS#nav_state.alt,
		NS#nav_state.mx, NS#nav_state.my, NS#nav_state.mz ]).

dbg_movement(M) ->
    io:format("move: X(cm/sec)={x(~-10.4f) y(~-10.4f) alt(~-10.4f)} orient(deg/sec)={roll(~-10.4f) pitch(~-10.4f) yaw(~-10.4f)}", 
	      [ M#movement.x_spd, M#movement.y_spd, M#movement.alt_spd,
		M#movement.roll_spd, M#movement.pitch_spd, M#movement.yaw_spd ]).


dbg_position(M) ->
    io:format("pos:  X(cm)=    {x(~-10.4f) y(~-10.4f) alt(~-10.4f)}     orient(deg)={roll(~-10.4f) pitch(~-10.4f) yaw(~-10.4f)}", 
	      [ M#position.x, M#position.y, M#position.alt,
		M#position.roll, M#position.pitch, M#position.yaw ]).
cap(Val, Min, Max) ->
    max(min(Val, Max), Min).
