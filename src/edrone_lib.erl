%%% @author Magnus Feuer <magnus.feuer@feuerlabs.se>
%%% @copyright (C) 2013, Feuerlabs, Inc
%%% @doc
%%%    Motorboard control
%%% @end
%%% Created :  17 Sep 2013 by Magnus Feuer <magnus.feuer@feuerlabs.com>

-module(edrone_lib).

-export([timestamp/0]).

timestamp() ->
    %% Invented epoc that was about when this code was written.
    %% Microsecond resolution
    {MS,S,US} = os:timestamp(),
    MS*1000000000000 + S*1000000 + US - 946689817000000.
