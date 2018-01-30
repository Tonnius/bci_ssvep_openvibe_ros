
stimulation_frequencies = {}
frequency_count = 0

target_light_color = {}
target_dark_color = {}
training_target_size = {}
training_targets_positions = {}

processing_epoch_duration = nil
processing_epoch_interval = nil
processing_frequency_tolerance = nil

function initialize(box)

	dofile(box:get_config("${Path_Data}") .. "/plugins/stimulation/lua-stimulator-stim-codes.lua")

	for value in box:get_setting(2):gmatch("%d+") do
		table.insert(target_light_color, value)
	end

	for value in box:get_setting(3):gmatch("%d+") do
		table.insert(target_dark_color, value)
	end

	for value in box:get_setting(4):gmatch("%d+[.]?%d*") do
		table.insert(stimulation_frequencies, value)
		frequency_count = frequency_count + 1
	end

	processing_epoch_duration = box:get_setting(5)
	processing_epoch_interval = box:get_setting(6)
	processing_frequency_tolerance = box:get_setting(7)

end

function uninitialize(box)
end

function process(box)

	while box:keep_processing() and box:get_stimulation_count(1) == 0 do
		box:sleep()
	end

	box:log("Info", box:get_config("Writing additional configuration to '${CustomConfigurationPrefix${OperatingSystem}}-ssvep-demo${CustomConfigurationSuffix${OperatingSystem}}'"))

	cfg_file = assert(io.open(box:get_config("${CustomConfigurationPrefix${OperatingSystem}}-ssvep-demo${CustomConfigurationSuffix${OperatingSystem}}"), "a"))

	success = true 
	success = success and cfg_file:write("SSVEP_TargetLightColourRed = ", target_light_color[1] / 100, "\n")
	success = success and cfg_file:write("SSVEP_TargetLightColourGreen = ", target_light_color[2] / 100, "\n")
	success = success and cfg_file:write("SSVEP_TargetLightColourBlue = ", target_light_color[3] / 100, "\n")
	success = success and cfg_file:write("SSVEP_TargetDarkColourRed = ", target_dark_color[1] / 100, "\n")
	success = success and cfg_file:write("SSVEP_TargetDarkColourGreen = ", target_dark_color[2] / 100, "\n")
	success = success and cfg_file:write("SSVEP_TargetDarkColourBlue = ", target_dark_color[3] / 100, "\n")

	for i=1,frequency_count do
		success = success and cfg_file:write("SSVEP_Frequency_", i, " = ", string.format("%g", stimulation_frequencies[i]), "\n")
	end
	
	cfg_file:close()

	if (success == false) then
		box:log("Error", box:get_config("Write error"))
		return false
	end
	
	-- create configuration files for temporal filters

	scenario_path = box:get_config("${Player_ScenarioDirectory}")
		
	for i=1,frequency_count do
		cfg_file_name = scenario_path .. string.format("/configuration/temporal-filter-freq-%d.cfg", i)
		box:log("Info", "Writing file '" .. cfg_file_name .. "'")

		cfg_file = io.open(cfg_file_name, "w")
		if cfg_file == nil then
			box:log("Error", "Cannot write to [" .. cfg_file_name .. "]")
			box:log("Error", "Please copy the scenario folder to a directory with write access and use from there.")		
			return false
		end
		
		success = true
		success = success and cfg_file:write("<OpenViBE-SettingsOverride>\n")
		success = success and cfg_file:write("<SettingValue>Butterworth</SettingValue>\n")
		success = success and cfg_file:write("<SettingValue>Band pass</SettingValue>\n")
		success = success and cfg_file:write("<SettingValue>4</SettingValue>\n")
		success = success and cfg_file:write(string.format("<SettingValue>%g</SettingValue>\n", stimulation_frequencies[i] - processing_frequency_tolerance))
		success = success and cfg_file:write(string.format("<SettingValue>%g</SettingValue>\n", stimulation_frequencies[i] + processing_frequency_tolerance))
		success = success and cfg_file:write("<SettingValue>0.500000</SettingValue>\n")
		success = success and cfg_file:write("</OpenViBE-SettingsOverride>\n")
		
		cfg_file:close()
		
		if (success == false) then
			box:log("Error", box:get_config("Write error"))
			return false
		end
	
	end

	-- create configuration file for time based epoching
	cfg_file_name = scenario_path .. "/configuration/time-based-epoching.cfg";
	
	box:log("Info", "Writing file '" .. cfg_file_name .. "'")

	cfg_file = io.open(cfg_file_name, "w")
	if cfg_file == nil then
		box:log("Error", "Cannot write to [" .. cfg_file_name .. "]")
		box:log("Error", "Please copy the scenario folder to a directory with write access and use from there.")
		return false
	end
		
	success = true
	success = success and cfg_file:write("<OpenViBE-SettingsOverride>\n")
	success = success and cfg_file:write(string.format("<SettingValue>%g</SettingValue>\n", processing_epoch_duration))
	success = success and cfg_file:write(string.format("<SettingValue>%g</SettingValue>\n", processing_epoch_interval))
	success = success and cfg_file:write("</OpenViBE-SettingsOverride>\n")
		
	cfg_file:close()

	if (success == false) then
		box:log("Error", box:get_config("Write error"))
		return false
	end
	
	-- notify the scenario that the configuration process is complete
	
	box:send_stimulation(1, OVTK_StimulationId_TrainCompleted, box:get_current_time() + 0.2, 0)

end
