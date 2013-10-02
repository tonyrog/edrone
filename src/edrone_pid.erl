%%% @author Tony Rogvall <tony@rogvall.se>
%%% @copyright (C) 2013, Tony Rogvall
%%% @doc
%%%    PID controler
%%% @end
%%% Created : 17 Sep 2013 by Tony Rogvall <tony@rogvall.se>

-module(edrone_pid).

-export([new/3, new/5, new/6, new/7]).
-export([set_point/2, set_point/3]).
-export([set_param/4, set_integral/2, set_integral/4]).
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
	  target_set_point,%% current set_point
	  set_point_time,  %% setpoint time ramp
	  set_point_dt_sum,%% sum of td
	  %% Maximum and minimum allowable integrator state
	  i_min, i_max,
	  %% Proportional/Integral/Derevative gain
	  kp, ki, kd
	}).


new(Kp,Ki,Kd)
  when is_number(Kp), is_number(Ki), is_number(Kd) ->
    new_(Kp,Ki,Kd,undefined,undefined,0.0,timestamp()).

new(Kp,Ki,Kd,I_min,I_max) 
  when is_number(Kp), is_number(Ki), is_number(Kd),
       is_number(I_min), is_number(I_max), I_min =< I_max ->
    new_(Kp,Ki,Kd,I_min,I_max,0.0,timestamp()).

new(Kp,Ki,Kd,I_min,I_max,I)
  when is_number(Kp), is_number(Ki), is_number(Kd),
       is_number(I_min), is_number(I_max), is_number(I), I_min =< I_max ->
    new_(Kp,Ki,Kd,I_min,I_max,I,timestamp()).

new(Kp,Ki,Kd,I_min,I_max,I,Ts)
  when is_number(Kp), is_number(Ki), is_number(Kd),
       is_number(I_min),is_number(I_max),is_number(I), I_min =< I_max ->
    new_(Kp,Ki,Kd,I_min,I_max,I,Ts).

new_(Kp,Ki,Kd,I_min,I_max,I,Ts) ->
    #pid { kp = Kp, ki = Ki, kd = Kd,
	   set_point = 0,
	   target_set_point = 0,
	   set_point_time = 0,
	   set_point_dt_sum = 0,
	   timestamp = Ts,
	   integral = clip_(I,I_min,I_max),
	   i_min=I_min, i_max=I_max }.

set_param(PID,Kp,Ki,Kd)
  when is_record(PID,pid),is_number(Kp), is_number(Ki), is_number(Kd) ->
    PID#pid { kp = Kp, ki = Ki, kd = Kd }.

set_integral(PID, I) when is_record(PID,pid), is_number(I) ->
    PID#pid { integral = clip_integral(PID, I) }.

set_integral(PID, I_min, I_max, I) when 
      is_record(PID,pid),
      is_number(I_min), is_number(I_max), is_number(I), I_min =< I_max ->
    PID#pid { i_min = I_min, i_max = I_max, 
	      integral = clip_(I, I_min, I_max) }.

%% Set the new target value directly 
set_point(PID, Value) when is_record(PID,pid), is_number(Value) ->
    PID#pid { set_point = Value,
	      target_set_point = Value,
	      set_point_time = 0,
	      set_point_dt_sum = 0 }.

%% Set the new target value using a time ramp of Time seconds
set_point(PID, Value, Time)
  when is_record(PID,pid), is_number(Value), is_number(Time), Time > 0 ->
    PID#pid { target_set_point = Value,
	      set_point_time = Time,
	      set_point_dt_sum = 0 }.

update(PID, Feed) when is_number(Feed) ->
    update(PID, Feed, os:timestamp()).

update(PID, Feed, Ts) when is_number(Feed) ->
    Dt = timestamp_diff(Ts, PID#pid.timestamp) / 1000000.0,
    update_e(PID, Feed, Dt, Ts).
    
%% Error is the delta between
%% requested Value and current Value
%% 
update_e(PID, Feed, Dt, Ts) when 
      PID#pid.set_point_time < PID#pid.set_point_dt_sum ->
    Current = PID#pid.set_point,
    Target = PID#pid.target_set_point,
    DtSum = PID#pid.set_point_dt_sum + Dt,
    Sd = (Target - Current)/Dt,
    SetPoint = max(Current + Sd, Target),
    update_ef(PID#pid { set_point_dt_sum = DtSum }, SetPoint, Feed, Dt, Ts);
update_e(PID, Feed, Dt, Ts) ->
    update_ef(PID, PID#pid.target_set_point, Feed, Dt, Ts).

update_ef(PID, SetPoint, Feed, Dt, Ts) ->
    Error = SetPoint - Feed,
    I0 = PID#pid.integral,
    I1 = I0 + Error*Dt,
    I2 = clip_integral(PID, I1),
    D1 = (Error - PID#pid.prev_error) / Dt,
    Output = (PID#pid.kp*Error)+(PID#pid.ki*I2)+(PID#pid.kd*D1),
    {Output, PID#pid { integral=I2, prev_error=Error, timestamp=Ts } }.


clip_integral(PID, I) -> clip_(I, PID#pid.i_min, PID#pid.i_max).

clip_(I, undefined, _) -> I;
clip_(I, _, undefined) -> I;
clip_(I, I_min, I_max) -> max(min(I,I_max),I_min).
    
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

