local M = {}

ROS_CONFIG = {
    catkin_ws_path = "~/catkin_ws",
    terminal_height = 8,
    catkin_program = "catkin_make"
}

function M.setup(config)
    for key, value in pairs(config) do
        if value ~= nil then
            ROS_CONFIG[key] = value
        end
    end
end

return M
