%% 
%% Reading battery voltage levels
%%
-module(edrone_bat).

-export([init/0]).
-export([read/0]).

%% debug
-export([read_adc/1, get_value/1]).

%% open i2c bus 0 (/dev/i2c-0)
-define(BUS, 0).
-define(VBAT_ADDRESS, 16#49).

init() ->
    i2c:open(?BUS),
    i2c:set_slave(?BUS, ?VBAT_ADDRESS),
    i2c:smbus_write_byte_data(?BUS, 16#30, 16#C7), %% ADC_CTRL
    i2c:smbus_write_byte_data(?BUS, 16#31, 16#5f), %% ADC_MUX_1
    i2c:smbus_write_byte_data(?BUS, 16#32, 16#0f), %% ADC_MUX_2
    %% Setpoint
    {ok,V0} = i2c:smbus_read_byte_data(?BUS, 16#06),
    {ok,V1} = i2c:smbus_read_byte_data(?BUS, 16#07),
    {ok,V2} = i2c:smbus_read_byte_data(?BUS, 16#08),
    {ok,V3} = i2c:smbus_read_byte_data(?BUS, 16#09),
    {ok,V4} = i2c:smbus_read_byte_data(?BUS, 16#0A),
    Vdd0 = if V0 band 16#80 =:= 0 -> 0;
	      true -> (V0 band 16#3f) * 0.05 + 0.80
	   end,
    Vdd1 = if V1 band 16#80 =:= 0 -> 0;
	      true -> (V1 band 16#3f) * 0.05 + 0.80
	   end,
    Vdd2 = if V2 band 16#80 =:= 0 -> 0;
	      true -> (V2 band 16#3f) * 0.05 + 0.80
	   end,
    Vdd3 = if V3 band 16#80 =:= 0 -> 0;
	      true -> (V3 band 16#3f) * 0.05 + 2.70
	   end,
    Vdd4 = if V4 band 16#80 =:= 0 -> 0;
	      true -> (V4 band 16#3f) * 0.05 + 2.70
	   end,
    {Vdd0,Vdd1,Vdd2,Vdd3,Vdd4}.
	    
read() ->    
    { get_value(0),
      get_value(4),
      get_value(5),
      get_value(6),
      get_value(7),
      get_value(8) }.

read_adc(Channel) when Channel =< 9 ->
    Chan = 2*Channel,
    L = i2c:smbus_read_byte_data(?BUS, 16#34 + Chan),
    U = i2c:smbus_read_byte_data(?BUS, 16#33 + Chan),
    (U bsl 2) bor L.  %% is this really correct? check!

get_value(Channel) ->
    ADC = read_adc(Channel),
    ADC01 = ADC/1023,
    if Channel < 4 -> ADC01*(7.92825/0.25); %% ?? 0.031*1023
       Channel < 9 -> ADC01*(1.8/0.4);
       true -> ADC01*(1.8/0.25)
    end.


    

		
	    
