local package = require("ros-nvim.package")
local utils = require("ros-nvim.vim-utils")

local M = {}

function M.rostest()
    local test_name = vim.fn.expand("%:t:r") .. ".test"
    -- naming convention isnt always very clear
    test_name, _ = string.gsub(test_name, "_test", "")
    local pkg_name = package.get_current_package_name()
    local test_command = "rostest " .. pkg_name .. " " .. test_name
    utils.open_terminal()
    utils.send_command_to_current_term(test_command)
end

return M
