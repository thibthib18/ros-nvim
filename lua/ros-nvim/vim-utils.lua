local M = {}

function M.go_to_buffer_id(bufnr)
    local winnr = vim.fn.bufwinnr(bufnr)
    local winid = vim.fn.win_getid(winnr)
    vim.fn.win_gotoid(winid)
end

function M.resize_split(height)
    vim.cmd("resize " .. (height or ROS_CONFIG.terminal_height))
end

function M.open_terminal()
    vim.cmd("terminal")
end

function M.open_split(with_terminal)
    with_terminal = with_terminal == nil and true or with_terminal
    vim.cmd("split")
    if with_terminal then
        M.open_terminal()
    end
    M.resize_split()
end

function M.send_command_to_current_term(command, autoscroll, name, enter)
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
    M.open_split()
    M.send_command_to_current_term(path, true, "exec " .. filename)
end

return M
