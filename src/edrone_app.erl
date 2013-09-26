-module(edrone_app).

-behaviour(application).

%% Application callbacks
-export([start/2,
	 start_phase/3,
	 stop/1]).

%% ===================================================================
%% Application callbacks
%% ===================================================================

start(_StartType, _StartArgs) ->
    io:format("start~n"),
    edrone_sup:start_link().

start_phase(Ph, _, _) ->
    io:format("start_phase(~p)~n", [ Ph ]),
    ok.

stop(_State) ->
    ok.
