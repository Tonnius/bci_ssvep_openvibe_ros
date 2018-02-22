-- this function is called when the box is initialized
function initialize(box)
        box:log("Trace", "initialize has been called")
        box:log("Info", string.format("Time is %f ", box:get_current_time()))

end
-- this function is called when the box is uninitialized
function uninitialize(box)
        box:log("Trace", "uninitialize has been called")
        box:log("Info", string.format("Time is %f ", box:get_current_time()))
end
-- this function is called once by the box
function process(box)
        box:log("Info", string.format("Time is %f ", box:get_current_time()))

       
end