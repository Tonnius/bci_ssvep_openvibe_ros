
vrpn_host = nil
vrpn_port = nil
screen_refresh_rate = nil

function initialize(box)
	dofile(box:get_config("${Path_Data}") .. "/plugins/stimulation/lua-stimulator-stim-codes.lua")
	screen_refresh_rate = box:get_setting(2)
end

function uninitialize(box)
end

function process(box)


	box:send_stimulation(1, OVTK_StimulationId_TrainCompleted, box:get_current_time() + 0.2, 0)
end
