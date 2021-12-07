local package = require("ros-nvim.package")
local M = {}

M.search_package = function()
    local package_path = package.get_current_package_path()
    local package_name = package.get_current_package_name()
    if package_path == nil then
        vim.notify("Not in a ROS package", "error", {title = "Search ROS package"})
        return
    end
    if ROS_CONFIG.catkin_ws_link_from ~= nil then
        package_path, _ = string.gsub(package_path, ROS_CONFIG.catkin_ws_link_from, ROS_CONFIG.catkin_ws_link_to)
    end
    require("telescope.builtin").find_files(
        {
            prompt_title = "< " .. package_name .. " >",
            cwd = package_path
        }
    )
end

M.grep_package = function()
    local package_path = package.get_current_package_path()
    if package_path == nil then
        vim.notify("Not in a ROS package", "error", {title = "Grep ROS package"})
        return
    end
    if ROS_CONFIG.catkin_ws_link_from ~= nil then
        package_path, _ = string.gsub(package_path, ROS_CONFIG.catkin_ws_link_from, ROS_CONFIG.catkin_ws_link_to)
    end
    require("telescope.builtin").live_grep(
        {
            search_dirs = {package_path}
        }
    )
end

return M
