classes = nil

current_target = nil
start_time = 999;
stop_time = -1;

do_debug = false;

score = {}
mean_detect_time = 0;
nr_of_stim = 0;
function initialize(box)
	dofile(box:get_config("${Path_Data}") .. "/plugins/stimulation/lua-stimulator-stim-codes.lua")
	classes = box:get_setting(2)

	for j = 1, classes do
		score[j] = {}
		for i = 1, classes do
			score[j][i] = 0
		end
	end
end

function uninitialize(box)

	correct = 0
	incorrect = 0

	for j = 1, classes do
		
		output = string.format("Target %d : ", j - 1)
		inClass = 0
		for i = 1, classes do
			inClass = inClass + score[j][i]
		end

		for i = 1, classes do
			output = output .. string.format("%d : %5.1f%%, ", i - 1, 100*score[j][i]/inClass)

			if i == j then
				
				correct = correct + score[j][i]
				
			else
				incorrect = incorrect + score[j][i]
			end
		end

		box:log("Info", string.format("%s total %d", output, inClass))
	end
	box:log("Info", string.format("Correct   %4d -> %5.1f%%", correct, 100*correct/(correct+incorrect)))
	box:log("Info", string.format("Incorrect %4d -> %5.1f%%", incorrect, 100*incorrect/(correct+incorrect)))
	box:log("Info", string.format("Mean time between stimulation start and prediction was %s , nr of stims was %s", mean_detect_time / nr_of_stim, nr_of_stim))
end

function process(box)

	finished = false
	pred_done = false
	current_target_time = box:get_current_time()
	while not finished do

		-- time = box:get_current_time()

		while box:get_stimulation_count(1) > 0 do

			s_code, s_date, s_duration = box:get_stimulation(1, 1)
			box:remove_stimulation(1, 1)
			
			if s_code >= OVTK_StimulationId_Label_01 and s_code <= OVTK_StimulationId_Label_1F then
				if do_debug then box:log("Info", string.format("Received target %d at ", s_code) .. s_date) end
				current_target = s_code - OVTK_StimulationId_Label_01
				current_target_time = s_date
				pred_done = false
				nr_of_stim = nr_of_stim + 1
				box:send_stimulation(3, OVTK_StimulationId_Label_1E, box:get_current_time() + 1, 0)

			elseif s_code == OVTK_StimulationId_VisualStimulationStart then
				if do_debug then box:log("Info", "Trial started at " .. s_date) end
				start_time = s_date
				-- We don't know the stop time yet, so for now accept anything in range [start_time, infty]
				stop_time = -1
				
			elseif s_code == OVTK_StimulationId_VisualStimulationStop then
				if do_debug then box:log("Info", "Trial ended at " .. s_date) end
				stop_time = s_date

			elseif s_code == OVTK_StimulationId_ExperimentStop then
				finished = true
			end
		end

		while box:get_stimulation_count(2) > 0 do
			if current_target ~= nil then
				s_code, s_date, s_duration = box:get_stimulation(2, 1)
				box:remove_stimulation(2, 1)

				if do_debug then box:log("Info", string.format("Received prediction %d at ", s_code) .. s_date) end
				
				if (s_date >= start_time) and (stop_time < 0 or s_date <= stop_time) and (s_code >= OVTK_StimulationId_Label_01 and s_code <= OVTK_StimulationId_Label_1F) then

					if do_debug then box:log("Info", string.format("Accepted prediction %d at ", s_code) .. s_date) end

					real_target = current_target + 1
					prediction = s_code - OVTK_StimulationId_Label_01 + 1
					score[real_target][prediction] = score[real_target][prediction] + 1
					--if prediction == 4 and real_target == 2 then box:log("Info", string.format("pred 4, real target 2")) end
					box:send_stimulation(1, OVTK_StimulationId_Label_01 + real_target, box:get_current_time() + 0.001, 0)
					box:send_stimulation(2, OVTK_StimulationId_Label_01 + prediction, box:get_current_time() + 0.001, 0)
					if pred_done == false then --and real_target == prediction then
						box:send_stimulation(3, OVTK_StimulationId_Label_1F, box:get_current_time() + 0.001, 0)

						pred_done = true
						--box:log("Info", string.format("Time between stimulation start and prediction was %s ", s_date - start_time))
						mean_detect_time = mean_detect_time + (s_date - start_time)
					end
				end
			else
				box:remove_stimulation(2, 1)
			end
			
		end

		box:sleep()

	end

end
