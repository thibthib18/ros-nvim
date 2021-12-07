local M = {}

ROS_CONFIG = {
    catkin_ws_path = "~/catkin_ws",
    terminal_height = 8,
    -- program to use for make/build commands, use e.g. "catkin build"
    catkin_program = "catkin_make",
    -- optional redirection in case you work in a directory that catkin_ws/src is a symlink of
    -- in package search, this avoids opening files from a different path
    catkin_ws_link_from = nil,
    catkin_ws_link_to = nil
}

function M.setup(config)
    for key, value in pairs(config) do
        if value ~= nil then
            ROS_CONFIG[key] = value
        end
    end
end

return M
