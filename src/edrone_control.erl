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

-export([flat_trim/0, enable/0, disable/0, move_to/1, move_to_height/1]).

-define(SERVER, ?MODULE). 

-record(st, { 
	  uart = undefined,
	  pwm_pid = undefined,
	  nav_recv_ref = undefined,
	  flat_trim = #flat_trim{},
	  current_pos = #position {},
	  target_pos = #position{},
	  enabled = false,
	  motor = {0,0,0,0}
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

%% Height. 
move_to_height(CM) ->
    gen_server:call(?MODULE, {move_to_height, CM}).
    
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

handle_call({ move_to_height, CM }, _From, St) ->
    { reply, ok, St#st { target_pos = (St#st.target_pos)#position { height = CM } } };

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
		  %% FIXME: INSERT SIGNAL FILTER CALL HERE

		  %% Process the nav frame to a nav state
		  NavState = edrone_navboard:process_nav_frame(NavFrame, St#st.flat_trim),
		  io:format("nav: Acc(g)={x(~-10.4f) y(~-10.4f) z(~-10.4f)} Gyro(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)} Height(cm)=~10.4f Compass(deg)={x(~-10.4f) y(~-10.4f) z(~-10.4f)}~n", 
			    [ NavState#nav_state.ax, NavState#nav_state.ay, NavState#nav_state.az, 
			      NavState#nav_state.gx, NavState#nav_state.gy, NavState#nav_state.gz, 
			      NavState#nav_state.height,
			      NavState#nav_state.mx, NavState#nav_state.my, NavState#nav_state.mz ]),

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
process_nav_state(#st{ current_pos = CP, target_pos = TP } = St, 
		  #nav_state{ } = NavState) ->


    #position {
      x = TgtX, y = TgtY, height = TgtHeight,
      yaw = TgtYaw, pitch = TgtPitch, roll = TgtRoll,
      heading = TgtHeading, direction = TgtDirection,
      speed = TgtSpeed, climb = TgtClimb 
     } = TP,

    #position {
      x = CurX, y = CurY, height = CurHeight,
      yaw = CurYaw, pitch = CurPitch, roll = CurRoll,
      heading = CurHeading, direction = CurDirection,
      speed = CurSpeed, climb = CurClimb 
     } = CP,

    %% Lose the compile warnings without having to add an underscore.
    TgtHeight, TgtX, TgtY, TgtPitch, TgtRoll, TgtYaw, TgtDirection,
    TgtHeading, TgtSpeed, CurX, CurY, CurPitch, TgtClimb, CurHeight,
    CurRoll, CurYaw, CurDirection, CurHeading, CurClimb, CurSpeed, 


    %% Just operate on climb rate right now. (cm/sec)
    ClippedHeight = min(NavState#nav_state.height, TgtHeight),
    Throttle = case TgtHeight of 
	0 -> 0.0;
	0.0 -> 0.0;
	_ -> 1 - (ClippedHeight / TgtHeight)
    end,

%%    ClippedThrottle = edrone_motor:clip_pwm(Throttle, Throttle, Throttle, Throttle),
    io:format("Throttle: ~p ~n", [ Throttle ]),
    ClippedThrottle = edrone_motor:set_pwm(St#st.pwm_pid, Throttle, Throttle, Throttle, Throttle),
    St#st { motor = ClippedThrottle, 
	    current_pos = CP#position { height = NavState#nav_state.height} }.



