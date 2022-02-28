local _, Job = pcall(require, "plenary.job")
local previewers = require "telescope.previewers"

local M = {}

function M.topic_echo_preview()
    return previewers.new_buffer_previewer {
        title = "Echo",
        get_buffer_by_name = function(_, entry)
            return entry.name
        end,
        define_preview = function(self, entry)
            local bufnr = self.state.bufnr
            if self.state.bufname ~= entry.name or vim.api.nvim_buf_line_count(bufnr) == 1 then
                local job =
                    Job:new(
                    {
                        enable_recording = true,
                        command = "rostopic",
                        args = {"echo", entry.name},
                        on_stdout = vim.schedule_wrap(
                            function(error, line, j_self)
                                if vim.api.nvim_buf_is_valid(bufnr) then
                                    vim.api.nvim_buf_set_lines(bufnr, -1, -1, false, {line})
                                    -- continuously place cursor on last line to keep scrolling
                                    local linesCount = vim.api.nvim_buf_line_count(bufnr)
                                    local winnr = vim.fn.bufwinnr(bufnr)
                                    local winid = vim.fn.win_getid(winnr)
                                    if winid ~= 0 then
                                        vim.api.nvim_win_set_cursor(winid, {linesCount, 0})
                                    end
                                else
                                    j_self:_stop()
                                end
                            end
                        ),
                        on_exit = vim.schedule_wrap(
                            function(j_self, _, _)
                                local result = j_self:result()
                                vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, result)
                            end
                        )
                    }
                )
                --job:sync()
                job:start()
            end
        end
    }
end

function M.info_preview(command, arg)
    return previewers.new_buffer_previewer {
        title = "Info",
        get_buffer_by_name = function(_, entry)
            return entry.name
        end,
        define_preview = function(self, entry)
            local bufnr = self.state.bufnr
            if self.state.bufname ~= entry.name or vim.api.nvim_buf_line_count(bufnr) == 1 then
                local job =
                    Job:new(
                    {
                        enable_recording = true,
                        command = command,
                        args = {arg, entry.name},
                        on_exit = vim.schedule_wrap(
                            function(j_self, _, _)
                                local result = j_self:result()
                                vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, result)
                            end
                        )
                    }
                )
                job:start()
            end
        end
    }
end

return M
