
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

	box:log("Info", box:get_config("Generating '${CustomConfigurationPrefix${OperatingSystem}}-ssvep-demo${CustomConfigurationSuffix${OperatingSystem}}'"))

	cfg_file = assert(io.open(box:get_config("${CustomConfigurationPrefix${OperatingSystem}}-ssvep-demo${CustomConfigurationSuffix${OperatingSystem}}"), "w"))

	success = true
	success = success and cfg_file:write("# This file was automatically generated!\n\n")
	success = success and cfg_file:write("# If you want to change the SSVEP configuration\n")
	success = success and cfg_file:write("# please use the provided ssvep-configuration scenario.\n")
	success = success and cfg_file:write("SSVEP_ScreenRefreshRate = ", screen_refresh_rate, "\n")

	if (success == false) then
		box:log("Error", box:get_config("Write error"))
	end
	
	cfg_file:close()

	box:send_stimulation(1, OVTK_StimulationId_TrainCompleted, box:get_current_time() + 0.2, 0)
end
