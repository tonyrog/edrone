%%% @author tony <tony@rogvall.se>
%%% @copyright (C) 2013, tony
%%% @doc
%%%    Motorboard control
%%% @end
%%% Created :  6 Sep 2013 by tony <tony@rogvall.se>

-module(edrone_motor).

-export([start/0]).
-export([set_pwm/5, set_leds/5]).
-export([get_pwm/1, get_pwm_async/1]).

%% internal test
-export([open/0, init/1]).
-export([command_/3]).
-export([write_pwm/5, write_leds/5, 
	 run_pwm/5, cvt_pwm/4, clip_pwm/4]).


-define(MOTOR_UART, "/dev/ttyO0").
-define(GPIO_M1, 78).
-define(GPIO_M2, 79).
-define(GPIO_M3, 80).
-define(GPIO_M4, 81).

-define(GPIO_ERROR_READ,  175).
-define(GPIO_ERROR_RESET, 176).

-define(PWM_MIN, 16#000).
-define(PWM_MAX, 16#1ff).

-define(MOT_LEDOFF, 0).
-define(MOT_LEDRED, 1).
-define(MOT_LEDGREEN, 2).
-define(MOT_LEDORANGE, 3).

-define(TRANSMIT_IVAL, 5).

start() ->
    spawn_link(
      fun() ->
	      {ok,U} = open(),
	      ok = init(U),
	      write_leds(U,?MOT_LEDGREEN,?MOT_LEDGREEN,
			 ?MOT_LEDGREEN,?MOT_LEDGREEN),
	      %% start a timer used to syncronize the 5ms transmit tmo
	      %% the time is selected so that it should not timeout during
	      %%% flight.
	      T = erlang:start_timer(60*60*1000,self(),noway),
	      T0 = erlang:read_timer(T),
	      loop(U, T, T0, ?TRANSMIT_IVAL, 0, 0, 0, 0, 0, 0)
      end).

set_pwm(Pid, M0, M1, M2, M3) ->
    { P0, P1, P2, P3 } = cvt_pwm(M0,M1, M2, M3),
    cast(Pid, {set_pwm,P0,P1,P2,P3}),
 
    { P0, P1, P2, P3 }. %% Will be clipped to 0.0-1.0 interval
    

set_leds(Pid, L0, L1, L2, L3) ->
    cast(Pid, {set_leds,L0,L1,L2,L3}).

get_pwm(Pid) ->
    call(Pid, get_pwm).

get_pwm_async(Pid) ->
    async_call(Pid, get_pwm).

%% call/cast
call(Pid, Call) ->
    call(Pid, Call, infinity).

call(Pid, Call, Tmo) ->
    Ref = async_call(Pid, Call),
    wait_reply(Ref, Tmo).

async_call(Pid, Call) ->
    Ref = make_ref(),
    Pid ! {'$call', [self()|Ref], Call},
    Ref.

cast(Pid, Cast) ->
    Pid ! {'$cast', [self()|undefined], Cast},
    true.

wait_reply(Ref,Tmo) ->
    receive
	{Ref, Result} ->
	    Result
    after Tmo ->
	    {error,timeout}
    end.

send_reply([_Pid|undefined], _Result) -> ok;
send_reply([Pid|Ref], Result) -> Pid ! {Ref, Result}.


open() ->
    uart:open(?MOTOR_UART, [{baud,115200},{mode,binary}]).

init(U) ->
    MotorPins = [?GPIO_M1,?GPIO_M2,?GPIO_M3,?GPIO_M4],
    gpio:init(?GPIO_ERROR_READ),
    gpio:init(?GPIO_ERROR_RESET),
    [gpio:init(M) || M <- MotorPins],

    gpio:set_direction(?GPIO_ERROR_READ, in),
    gpio:set_direction(?GPIO_ERROR_RESET, low),
    gpio:set(?GPIO_ERROR_RESET),

    [ gpio:set_direction(M, high) || M <- MotorPins],

    [ begin
	  gpio:set_direction(M, in),
	  case command_(U,16#e0,2) of
	      {ok,<<16#E0,16#00>>} -> ok;
	      {ok,Ret} ->
		  io:format("motor%d cmd=~w reply=~w\n", [Mi,Ret])
	  end,
	  command_(U, Mi, 1),
	  gpio:set_direction(M, high)
      end || {Mi,M} <- [{1,?GPIO_M1},{2,?GPIO_M2},{3,?GPIO_M3},{4,?GPIO_M4}]],

    [ gpio:set_direction(M, in) || M <- MotorPins],

    [ command_(U,16#a0, 1) || _ <- lists:seq(1,5)],

    %% not needed ! gpio:set_direction(?GPIO_ERROR_READ, in),
    %% gpio:set_interrupt(?GPIO_ERROR_READ, rising),
    %% receive {gpio_interrupt, _Port, ?GPIO_ERROR_READ, 1} -> error!
    %% 
    gpio:clr(?GPIO_ERROR_RESET),
    gpio:set(?GPIO_ERROR_RESET),
    ok.

loop(U, T, T0, TransmitTmo, I, Mi, P0, P1, P2, P3) when I >= 25 ->
    %% check if motor Mi = 0,1,2,3 is runnig
    loop_(U, T, T0, TransmitTmo, 0, (Mi+1) band 3, P0, P1, P2, P3);
loop(U, T, T0, TransmitTmo, I, Mi, P0, P1, P2, P3) ->
    receive
	{gpio_interrupt, _Port, ?GPIO_ERROR_READ, 1} ->
	    io:format("interrupt: read_error reported\n"),
	    write_pwm(U,0,0,0,0),
	    error;
	{'$cast', _From, {set_pwm, NP0, NP1, NP2, NP3}} ->
	    loop_(U,T,T0,TransmitTmo,I,Mi,NP0,NP1,NP2,NP3);
	{'$cast', _From, {set_leds, L0, L1, L2, L3}} ->
	    write_leds(U,L0,L1,L2,L3),
	    loop_(U,T,T0,TransmitTmo,I,Mi,P0,P1,P2,P3);
	{'$call', From, get_pwm} ->
	    send_reply(From, {P0,P1,P2,P3}),
	    loop_(U,T,T0,TransmitTmo,I,Mi,P0,P1,P2,P3);
	Other ->
	    io:format("edrone_motor: loop got ~p\n", [Other]),
	    loop_(U,T,T0,TransmitTmo,I,Mi,P0,P1,P2,P3)
    after
	TransmitTmo ->
	    write_pwm(U,P0,P1,P2,P3),
	    T1 = erlang:read_timer(T),
	    loop(U,T,T1,?TRANSMIT_IVAL,I+1,Mi,P0,P1,P2,P3)
    end.

loop_(U,T,T0,TransmitTmo,I,Mi,P0,P1,P2,P3) ->
    T1 = erlang:read_timer(T),
    Tmo = TransmitTmo - (T0-T1),
    if Tmo =< 0 ->
	    write_pwm(U, P0, P1, P2, P3),
	    loop(U,T,T1,?TRANSMIT_IVAL,I+1,Mi,P0,P1,P2,P3);
       true ->
	    loop(U,T,T1,Tmo,I,Mi,P0,P1,P2,P3)
    end.
	    
command_(U, Cmd, ReplyLen) ->
    uart:send(U, <<Cmd>>),
    uart:recv(U, ReplyLen).

write_pwm(U, Pwm0, Pwm1, Pwm2, Pwm3) ->
    uart:send(U, <<2#001:3, Pwm0:9, Pwm1:9, Pwm2:9, Pwm3:9, 0:1>>).

write_leds(U, Led0, Led1, Led2, Led3) ->
    uart:send(U, <<2#011:3, 
		   Led0:1, Led1:1, Led2:1, Led3:1, 0:1,
		   2#000:3,
		   (Led0 bsr 1):1, (Led1 bsr 1):1, 
		   (Led2 bsr 1):1, (Led3 bsr 1):1, 0:1 >>).

clip_pwm(M0, M1, M2, M3) ->
    { min(max(0.0, M0), 1.0),
      min(max(0.0, M1), 1.0),
      min(max(0.0, M2), 1.0),
      min(max(0.0, M3), 1.0) }.

cvt_pwm(M0, M1, M2, M3) ->
    { V0, V1, V2, V3 } = clip_pwm(M0, M1, M2, M3),

    { trunc(?PWM_MIN + V0*(?PWM_MAX - ?PWM_MIN)),
      trunc(?PWM_MIN + V1*(?PWM_MAX - ?PWM_MIN)),
      trunc(?PWM_MIN + V2*(?PWM_MAX - ?PWM_MIN)),
      trunc(?PWM_MIN + V3*(?PWM_MAX - ?PWM_MIN)) }.

run_pwm(U, M0, M1, M2, M3) ->
    {P0, P1, P2, P3} = cvt_pwm(M0, M1, M2, M3),

    write_pwm(U, P0, P1, P2, P3),
    clip_pwm(M0, M1, M2, M3).
