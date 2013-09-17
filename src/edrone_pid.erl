%%% @author Tony Rogvall <tony@rogvall.se>
%%% @copyright (C) 2013, Tony Rogvall
%%% @doc
%%%    PID controler
%%% @end
%%% Created : 17 Sep 2013 by Tony Rogvall <tony@rogvall.se>

-module(edrone_pid).

-export([new/3, new/5]).
-export([set_point/2]).
-export([update/2, update/3]).
-export([timestamp_to_us/1]).
-export([timestamp/0]).
-export([timestamp_diff/2]).

-record(pid,
	{
	  integral = 0,    %% Integrator state
	  prev_error = 0,  %% last error value
	  timestamp,       %% previous timestamp
	  set_point,       %% requested value
	  %% Maximum and minimum allowable inegrator state
	  i_min, i_max,
	  %% Proportional/Integral/Derevative gain
	  kp, ki, kd
	}).


new(Kp,Ki,Kd)
  when is_number(Kp), is_number(Ki), is_number(Kd) ->
    #pid { kp = Kp, ki = Ki, kd = Kd, 
	   set_point = 0,
	   timestamp = timestamp() }.

new(Kp,Ki,Kd, IMin, IMax) 
  when is_number(Kp), is_number(Ki), is_number(Kd),
       is_number(IMin), is_number(IMax), IMin =< IMax ->
    #pid { kp = Kp, ki = Ki, kd = Kd,
	   set_point = 0,
	   timestamp = timestamp(),
	   i_min=IMin, i_max=IMax }.    

set_point(PID, Value) when is_number(Value) ->
    PID#pid { set_point = Value }.

%%
%% 
%% read the current value
%%    Input = read_input(),
%%    [Time  = os:timestamp()],
%%    {Output,PID1} = update(PID0, Requested, Input [,Time]),
%%    write_output(Output)
%%
update(PID, Feed) when is_number(Feed) ->
    update_e(PID, PID#pid.set_point - Feed, 1.0, undefined).

update(PID, Feed, Ts) when is_number(Feed) ->
    Dt = timestamp_diff(Ts, PID#pid.timestamp) / 1000000.0,
    update_e(PID, PID#pid.set_point - Feed, Dt, Ts).
    
%% Error is the delta between
%% requested Value and current Value
update_e(PID, Error, Dt, Ts) ->
    I1 = PID#pid.integral + Error*Dt,
    I2 = if is_number(PID#pid.i_max), I1 > PID#pid.i_max ->
		 PID#pid.i_max;
	    is_number(PID#pid.i_min), I1 < PID#pid.i_min ->
		 PID#pid.i_min;
	    true -> I1
	 end,
    D1 = (Error - PID#pid.prev_error) / Dt,
    Output = (PID#pid.kp*Error)+(PID#pid.ki*I2)+(PID#pid.kd*D1),
    {Output, PID#pid { integral=I2, prev_error=Error, timestamp=Ts } }.

timestamp_diff({M,S,U1},{M,S,U0}) ->
    (U1 - U0);
timestamp_diff({M,S1,U1},{M,S0,U0}) ->
    (S1-S0)*1000000 + (U1-U0);
timestamp_diff({M1,S1,U1},{M0,S0,U0}) ->
    (((M1-M0)*1000000)+(S1-S0))*1000000 + (U1-U0).

timestamp() ->
    os:timestamp().

timestamp_to_us({M,S,U}) ->
    ((M*1000000+S)*1000000)+U.

