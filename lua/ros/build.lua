local utils = require("ros.vim-utils")
local package = require("ros.vim-utils")

local M = {}

function M.catkin_make(suffix, flags)
    flags = flags or ""
    local make_command = "catkin_make" .. (suffix or "") .. " " .. flags
    local current_bufnr = vim.fn.bufnr()
    local bufnr = vim.fn.bufnr("catkin_make")
    if bufnr ~= -1 then
        utils.go_to_buffer_id(bufnr)
    else
        utils.open_split()
    end
    local catkin_ws_path = "~/catkin_ws"
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
    catkin_make(suffix)
end

return M
