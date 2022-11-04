local finders = require "telescope.finders"
local pickers = require "telescope.pickers"
local _, Job = pcall(require, "plenary.job")
local conf = require "telescope.config".values
local action_set = require "telescope.actions.set"
local ros_previewers = require "ros-nvim.telescope.previewer"
local utils = require "ros-nvim.telescope.utils"

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

function M.node_picker()
    local opts = {
        command = "ros2 node",
        preview_arg = "info",
        preview_title = "Node Info",
        prompt_title = "ROS Nodes",
        results_title = "Nodes List",
        mappings = ROS_CONFIG.node_picker_mappings
    }
    info_picker(opts)
end

function M.topic_picker()
    local opts = {
        command = "ros2 topic",
        preview_arg = "info",
        preview_title = "Topic Info",
        prompt_title = "ROS Topics",
        results_title = "Topics List",
        mappings = ROS_CONFIG.topic_picker_mappings
    }
    opts.previewers = {
        ros_previewers.info_preview(opts.command, opts.preview_arg),
        ros_previewers.topic_echo_preview()
    }
    info_picker(opts)
end

function M.service_picker()
    local opts = {
        command = "ros2 service",
        preview_arg = "info",
        preview_title = "Service Info",
        prompt_title = "ROS Services",
        results_title = "Services List",
        mappings = ROS_CONFIG.service_picker_mappings
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
        command = "ros2 param",
        preview_arg = "get",
        preview_title = "Param value",
        prompt_title = "ROS Params",
        results_title = "Parameters List",
        mappings = ROS_CONFIG.param_picker_mappings
    }
    info_picker(opts)
end

return M
