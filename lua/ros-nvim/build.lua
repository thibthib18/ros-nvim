local utils = require("ros-nvim.vim-utils")
local package = require("ros-nvim.package")

local M = {}

function M.catkin_make(suffix, flags)
    flags = flags or ""
    local make_command = ROS_CONFIG.catkin_program .. (suffix or "") .. " " .. flags
    local current_bufnr = vim.fn.bufnr()
    local bufnr = vim.fn.bufnr("catkin_make")
    if bufnr ~= -1 then
        utils.go_to_buffer_id(bufnr)
    else
        utils.open_terminal()
    end
    local catkin_ws_path = ROS_CONFIG.catkin_ws_path
    utils.send_command_to_current_term("cd " .. catkin_ws_path, false)
    utils.send_command_to_current_term(make_command)
    utils.go_to_buffer_id(current_bufnr)
end

function M.catkin_make_pkg()
    local pkg_name = package.get_current_package_name()
    if pkg_name == nil then
        local path = vim.fn.expand("%:p")
        vim.notify(path .. " is not part of a ROS package", "error", {title = "catkin_make"})
        return
    end
    local suffix = " --pkg " .. pkg_name
    M.catkin_make(suffix)
end

return M
