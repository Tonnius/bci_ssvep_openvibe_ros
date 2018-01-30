-- this function is called when the box is initialized
function initialize(box)
        box:log("Trace", "initialize has been called")
 
        dofile(box:get_config("${Path_Data}") .. "/plugins/stimulation/lua-stimulator-stim-codes.lua")
        -- inspects the box topology
        box:log("Info", string.format("box has %i input(s)", box:get_input_count()))
        box:log("Info", string.format("box has %i output(s)", box:get_output_count()))
        box:log("Info", string.format("box has %i setting(s)", box:get_setting_count()))
        for i = 1, box:get_setting_count() do
                box:log("Info", string.format(" - setting %i has value [%s]", i, box:get_setting(i)))
        end
end
-- this function is called when the box is uninitialized
function uninitialize(box)
        box:log("Trace", "uninitialize has been called")
end
-- this function is called once by the box
function process(box)
        box:log("Trace", "process has been called")
        -- enters infinite loop
        -- cpu will be released with a call to sleep
        -- at the end of the loop
        while box:keep_processing() do
                -- gets current simulated time
                --t = box:get_current_time()
                -- loops on all inputs of the box
                for input = 1, box:get_input_count() do
                        -- loops on every received stimulation for a given input
                        for stimulation = 1, box:get_stimulation_count(input) do
                                -- gets the received stimulation
                                identifier = box:get_stimulation(input, 1)
                                -- logs the received stimulation
                                --box:log("Trace", string.format("At time %f on input %i got stimulation id:%s date:%s duration:%s", t, input, identifier, date, duration))
                                -- discards it
                                box:remove_stimulation(input, 1)
                               	if identifier >= OVTK_StimulationId_Label_00 and identifier <= OVTK_StimulationId_Label_1F then
                                	box:send_stimulation(1, identifier - 1, box:get_current_time() + 0.001, 0)
                                else
                                	box:send_stimulation(1, identifier, box:get_current_time() + 0.001, 0)
                                end
                        end
                end
                -- releases cpu
                box:sleep()
        end
end