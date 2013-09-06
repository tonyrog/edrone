%%% @author tony <tony@rogvall.se>
%%% @copyright (C) 2013, tony
%%% @doc
%%%    Motorboard control
%%% @end
%%% Created :  6 Sep 2013 by tony <tony@rogvall.se>

-module(edrone_motor).

-export([init/0, command/3, 
	 set_pwm/5, set_leds/5, run/5]).

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


init() ->
    {ok,U} = uart:open(?MOTOR_UART, [{baud,115200},{mode,binary}]),
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
	  case command(U,16#e0,2) of
	      {ok,<<16#E0,16#00>>} -> ok;
	      {ok,Ret} ->
		  io:format("motor%d cmd=~w reply=~w\n", [Mi,Ret])
	  end,
	  command(U, Mi, 1),
	  gpio:set_direction(M, high)
      end || {Mi,M} <- [{1,?GPIO_M1},{2,?GPIO_M2},{3,?GPIO_M3},{4,?GPIO_M4}]],

    [ gpio:set_direction(M, in) || M <- MotorPins],

    [ command(U,16#a0, 1) || _ <- lists:seq(1,5)],

    gpio:set_direction(?GPIO_ERROR_READ, in),
    gpio:clr(?GPIO_ERROR_RESET),
    gpio:set(?GPIO_ERROR_RESET),

    set_leds(U, ?MOT_LEDGREEN,?MOT_LEDGREEN,?MOT_LEDGREEN,?MOT_LEDGREEN),
    {ok, U}.

command(U, Cmd, ReplyLen) ->
    uart:send(U, <<Cmd>>),
    uart:recv(U, ReplyLen).

set_pwm(U, Pwm0, Pwm1, Pwm2, Pwm3) ->
    uart:send(U, <<2#001:3, Pwm0:9, Pwm1:9, Pwm2:9, Pwm3:9, 0:1>>).

set_leds(U, Led0, Led1, Led2, Led3) ->
    uart:send(U, <<2#011:3, 
		   Led0:1, Led1:1, Led2:1, Led3:1, 0:1,
		   2#000:3,
		   (Led0 bsr 1):1, (Led1 bsr 1):1, 
		   (Led2 bsr 1):1, (Led3 bsr 1):1, 0:1 >>).

run(U, M0, M1, M2, M3) ->
    V0 = min(max(0.0, M0), 1.0),
    V1 = min(max(0.0, M1), 1.0),
    V2 = min(max(0.0, M2), 1.0),
    V3 = min(max(0.0, M3), 1.0),
    
    PWM0 = trunc(?PWM_MIN + V0*(?PWM_MAX - ?PWM_MIN)),
    PWM1 = trunc(?PWM_MIN + V1*(?PWM_MAX - ?PWM_MIN)),
    PWM2 = trunc(?PWM_MIN + V2*(?PWM_MAX - ?PWM_MIN)),
    PWM3 = trunc(?PWM_MIN + V3*(?PWM_MAX - ?PWM_MIN)),
    set_pwm(U, PWM0, PWM1, PWM2, PWM3),
    {V0,V1,V2,V3}.
