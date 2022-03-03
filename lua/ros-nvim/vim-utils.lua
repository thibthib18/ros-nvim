local action_state = require "telescope.actions.state"
local actions = require "telescope.actions"
local M = {}

function M.go_to_buffer_id(bufnr)
    local winnr = vim.fn.bufwinnr(bufnr)
    local winid = vim.fn.win_getid(winnr)
    vim.fn.win_gotoid(winid)
end

function M.resize_split(height)
    vim.cmd("resize " .. (height or ROS_CONFIG.terminal_height))
end

function M.open_new_buffer()
    vim.cmd("enew")
end

function M.open_split()
    vim.cmd("split")
    M.resize_split()
end

function M.open_terminal()
    ROS_CONFIG.open_terminal_method()
    vim.cmd("terminal")
end

function M.open_terminal_with_format_cmd_entry(unformatted_command)
    return function(prompt_bufnr)
        actions.close(prompt_bufnr)
        local entry_name = action_state.get_selected_entry().name
        local command = string.format(unformatted_command, entry_name)
        M.open_terminal()
        M.send_command_to_current_term(command, true, nil, false)
    end
end

function M.send_command_to_current_term(command, autoscroll, name, enter)
    -- local opts = {autoscroll = true, name = "catkin_make", enter = true}
    local append_enter = enter == nil and "\\n" or ""
    local send_to_term = ':call jobsend(b:terminal_job_id, "' .. command .. append_enter .. '")'
    name = name or command
    vim.cmd(":file " .. name)
    vim.cmd(send_to_term)
    if autoscroll ~= false then
        vim.cmd(":normal! G")
    end
end

function M.execute_current_file()
    local path = vim.fn.expand("%:p")
    local filename = vim.fn.expand("%:t")
    M.open_terminal()
    M.send_command_to_current_term(path, true, "exec " .. filename)
end

return M
