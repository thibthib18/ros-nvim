local finders = require "telescope.finders"
local pickers = require "telescope.pickers"
local _, Job = pcall(require, "plenary.job")
local conf = require "telescope.config".values
local action_set = require "telescope.actions.set"
local ros_previewers = require "ros-nvim.telescope.previewer"
local utils = require "ros-nvim.telescope.utils"
local action_state = require "telescope.actions.state"
local vim_utils = require "ros-nvim.vim-utils"

local M = {}

local function info_picker(opts)
    local job =
        Job:new(
        {
            enable_recording = true,
            command = opts.command,
            args = {"list"},
            on_exit = vim.schedule_wrap(
                function(j_self, _, _)
                    local results = j_self:result()
                    local picker_opts = {}
                    picker_opts.preview_title = opts.preview_title or "Info"
                    picker_opts.prompt_title = opts.prompt_title or ""
                    picker_opts.results_title = opts.results_title or "List"
                    pickers.new(
                        picker_opts,
                        {
                            finder = finders.new_table {
                                results = utils.gen_entries(results),
                                entry_maker = utils.gen_from_name()
                            },
                            sorter = conf.generic_sorter(picker_opts),
                            previewer = opts.previewers or ros_previewers.info_preview(opts.command, opts.preview_arg),
                            attach_mappings = function(_, map)
                                action_set.select:replace(
                                    function(prompt_bufnr, type)
                                        print("ok")
                                    end
                                )
                                if opts.mappings ~= nil then
                                    opts.mappings(map)
                                end
                                return true
                            end
                        }
                    ):find()
                end
            )
        }
    )
    job:start()
end

local function open_terminal_with_cmd(partial_command)
    return function(prompt_bufnr)
        local entry_name = action_state.get_selected_entry().name
        local command = partial_command .. " " .. entry_name
        vim_utils.open_split()
        vim_utils.send_command_to_current_term(command, true, nil, false)
    end
end

function M.node_picker()
    local opts = {
        command = "rosnode",
        preview_arg = "info",
        preview_title = "Node Info",
        prompt_title = "ROS Nodes",
        results_title = "Nodes List",
        mappings = function(map)
            map("n", "<c-k>", open_terminal_with_cmd("rosnode kill"))
            map("i", "<c-k>", open_terminal_with_cmd("rosnode kill"))
        end
    }
    info_picker(opts)
end

function M.topic_picker()
    local opts = {
        command = "rostopic",
        preview_arg = "info",
        preview_title = "Topic Info",
        prompt_title = "ROS Topics",
        results_title = "Topics List",
        mappings = function(map)
            local cycle_previewers = function(prompt_bufnr)
                local picker = action_state.get_current_picker(prompt_bufnr)
                picker:cycle_previewers(1)
            end
            map("n", "<c-u>", open_terminal_with_cmd("rostopic pub"))
            map("i", "<c-u>", open_terminal_with_cmd("rostopic pub"))
            map("n", "<c-e>", cycle_previewers)
            map("i", "<c-e>", cycle_previewers)
        end
    }
    opts.previewers = {
        ros_previewers.info_preview(opts.command, opts.preview_arg),
        ros_previewers.topic_echo_preview()
    }
    info_picker(opts)
end

function M.service_picker()
    local opts = {
        command = "rosservice",
        preview_arg = "info",
        preview_title = "Service Info",
        prompt_title = "ROS Services",
        results_title = "Services List",
        mappings = function(map)
            map("n", "<c-e>", open_terminal_with_cmd("rosservice call"))
            map("i", "<c-e>", open_terminal_with_cmd("rosservice call"))
        end
    }
    info_picker(opts)
end

function M.srv_picker()
    local opts = {
        command = "rossrv",
        preview_arg = "info",
        preview_title = "Service Info",
        prompt_title = "ROS Services",
        results_title = "Services List"
    }
    info_picker(opts)
end

function M.msg_picker()
    local opts = {
        command = "rosmsg",
        preview_arg = "info",
        preview_title = "Message Info",
        prompt_title = "ROS Messages",
        results_title = "Messages List"
    }
    info_picker(opts)
end

function M.param_picker()
    local opts = {
        command = "rosparam",
        preview_arg = "get",
        preview_title = "Param value",
        prompt_title = "ROS Params",
        results_title = "Parameters List",
        mappings = function(map)
            map("n", "<c-e>", open_terminal_with_cmd("rosparam set"))
            map("i", "<c-e>", open_terminal_with_cmd("rosparam set"))
        end
    }
    info_picker(opts)
end

return M
