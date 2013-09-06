%% 
%% Reading battery voltage levels AR drone version 2
%%
-module(edrone_bat).

-export([init/0]).
-export([read/0]).

%% debug
-export([read_adc/1, get_value/1]).

%% AR drone 2
-define(BUS, 1).  %% (/dev/i2c-0)
-define(VBAT_ADDRESS, 16#4a).

init() ->
    i2c:open(?BUS),
    i2c:set_slave(?BUS, ?VBAT_ADDRESS),
    i2c:smbus_write_byte_data(?BUS, 16#00, 16#01), %% SET_POWER_ON
    i2c:smbus_write_byte_data(?BUS, 16#06, 16#01), %% SELECT_CHANNEL
    i2c:smbus_write_byte_data(?BUS, 16#08, 16#01), %% ENABLE AVERAGING
    i2c:smbus_write_byte_data(?BUS, 16#62, 16#0F), %% DISABLE (?) INTERUPT
    i2c:smbus_write_byte_data(?BUS, 16#12, 16#20), %% START CONVERSION
    ok.

read() ->
    get_value(0).

%% only one channel support ?
read_adc(Channel) when Channel =< 9 ->
    %% Chan = 2*Channel,
    i2c:smbus_write_byte_data(?BUS, 16#12, 16#20), %% START CONVERSION
    {ok,L} = i2c:smbus_read_byte_data(?BUS, 16#37),
    {ok,U} = i2c:smbus_read_byte_data(?BUS, 16#38),
    ((U bsl 8) bor L) bsr 6.

get_value(Channel) ->
    ADC = read_adc(Channel),
    ADC01 = ADC/1023,
    ADC01*13.781.
