local vim_utils = require "ros-nvim.vim-utils"
local action_state = require "telescope.actions.state"

local M = {}

ROS_CONFIG = {
    catkin_ws_path = "~/catkin_ws",
    catkin_program = "catkin_make",
    -- vim_utils.open_new_buffer or custom function
    open_terminal_method = function()
        vim_utils.open_split()
    end,
    terminal_height = 8, -- only for split terminal
    -- Picker mappings
    node_picker_mappings = function(map)
        map("n", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("ros2 node kill %s"))
        map("i", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("ros2 node kill %s"))
    end,
    topic_picker_mappings = function(map)
        local cycle_previewers = function(prompt_bufnr)
            local picker = action_state.get_current_picker(prompt_bufnr)
            picker:cycle_previewers(1)
        end
        map("n", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("ros2 topic pub %s"))
        map("i", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("ros2 topic pub %s"))
        map("n", "<c-e>", cycle_previewers)
        map("i", "<c-e>", cycle_previewers)
    end,
    service_picker_mappings = function(map)
        map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("ros2 service call %s"))
        map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("ros2 service call %s"))
    end,
    param_picker_mappings = function(map)
        map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("ros2 param set %s"))
        map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("ros2 param set %s"))
    end
}

function M.setup(config)
    for key, value in pairs(config) do
        if value ~= nil then
            ROS_CONFIG[key] = value
        end
    end
end

return M
