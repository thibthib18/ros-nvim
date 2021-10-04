local M = {}

function M.get_current_package_name(path)
    path = path or vim.fn.expand("%:p")
    local pkg_name = vim.fn.system('python3 -c "import rospkg;print(rospkg.get_package_name(\'' .. path .. '\'))"')
    -- clean up output
    pkg_name, _ = string.gsub(pkg_name, "\r", "")
    pkg_name, _ = string.gsub(pkg_name, "\n", "")
    if pkg_name == "None" then
        return
    end
    return pkg_name
end

function M.get_current_package_path()
    local pkg_name = M.get_current_package_name()
    if pkg_name == nil then
        return
    end
    local pkg_path = vim.fn.system("rospack find " .. pkg_name)
    -- clean up output
    pkg_path, _ = string.gsub(pkg_path, "\r", "")
    pkg_path, _ = string.gsub(pkg_path, "\n", "")
    return pkg_path
end

return M
