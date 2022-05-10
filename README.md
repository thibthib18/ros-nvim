# 🐢 ros-nvim 🐢
[ROS](https://www.ros.org) in Neovim.

**List ROS nodes with live info 🔦**
![nodelist_compressed](https://user-images.githubusercontent.com/37300147/135919833-d8988f88-7bf6-4e62-928a-5470ff18a1b5.gif)
**List ROS msgs with definitions ✉️**
![msg_compressed](https://user-images.githubusercontent.com/37300147/135919840-d5978470-f50e-4d66-9c02-dc6120189da4.gif)
**Search current ROS package 🔎**
![search_package_compressed](https://user-images.githubusercontent.com/37300147/135919843-af069238-8483-43c0-85ee-35ac1e08b3f0.gif)

Wraps ROS CLI utils (`rosnode`, `rostopic`, `rosmsg`, etc) with Vim and Telescope to bring it right to your favourite editor! 😁

## 🤩 Features

### 📦 ROS package wise build and search

- Search files with Telescope in the current ROS package
- Live grep with Telescope in the current ROS package
- Build current ROS package in terminal
- Execute current ROS test in terminal

### 🕵️  Several Telescope extensions for ROS introspection

- Nodes list & info
- Topics list & info & echo
- Services list & info
- Msgs list & info
- Srvs list & info
- Params list & values

## 🦒 Requirements

- Nvim >= 0.5
- (recommended) [telescope.nvim](https://github.com/nvim-telescope/telescope.nvim)

## 🏠 Installation

With your plugin manager of choice:

vim-plug:
```vim
Plug 'thibthib18/ros-nvim'
```
Packer:
```lua
use { 'thibthib18/ros-nvim', config=function()
    require 'ros-nvim'.setup({})
  end
}
```

## ⚙️  Setup

In Lua:
```lua
require 'ros-nvim'.setup {
  -- path to your catkin workspace
  catkin_ws_path = "~/catkin_ws",

  -- make program (e.g. "catkin_make" or "catkin build" )
  catkin_program = "catkin_make"

  --method for opening terminal for e.g. catkin_make: utils.open_new_buffer or custom function
  open_terminal_method = function()
      require "ros-nvim.vim-utils".open_split()
  end,

  -- terminal height for build / test, only valid with `open_terminal_method=open_split()`
  terminal_height = 8

  -- Picker mappings
  node_picker_mappings = function(map)
      map("n", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("rosnode kill %s"))
      map("i", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("rosnode kill %s"))
  end,
  topic_picker_mappings = function(map)
      local cycle_previewers = function(prompt_bufnr)
          local picker = action_state.get_current_picker(prompt_bufnr)
          picker:cycle_previewers(1)
      end
      map("n", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("rostopic pub %s"))
      map("i", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("rostopic pub %s"))
      -- While browsing topics, press <c-e> to switch between `rostopic info` and `rostopic echo`
      map("n", "<c-e>", cycle_previewers)
      map("i", "<c-e>", cycle_previewers)
  end,
  service_picker_mappings = function(map)
      map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosservice call %s"))
      map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosservice call %s"))
  end,
  param_picker_mappings = function(map)
      map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosparam set %s"))
      map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosparam set %s"))
  end
}
```

In Vim, simply enclose it in a lua block:
```vim
lua << EOF
require 'ros-nvim'.setup {
  -- path to your catkin workspace
  catkin_ws_path = "~/catkin_ws",

  -- make program (e.g. "catkin_make" or "catkin build" )
  catkin_program = "catkin_make"

  --method for opening terminal for e.g. catkin_make: utils.open_new_buffer or custom function
  open_terminal_method = function()
      require "ros-nvim.vim-utils".open_split()
  end,

  -- terminal height for build / test, only valid with `open_terminal_method=open_split()`
  terminal_height = 8

  -- Picker mappings
  node_picker_mappings = function(map)
      map("n", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("rosnode kill %s"))
      map("i", "<c-k>", vim_utils.open_terminal_with_format_cmd_entry("rosnode kill %s"))
  end,
  topic_picker_mappings = function(map)
      local cycle_previewers = function(prompt_bufnr)
          local picker = action_state.get_current_picker(prompt_bufnr)
          picker:cycle_previewers(1)
      end
      map("n", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("rostopic pub %s"))
      map("i", "<c-b>", vim_utils.open_terminal_with_format_cmd_entry("rostopic pub %s"))
      map("n", "<c-e>", cycle_previewers)
      map("i", "<c-e>", cycle_previewers)
  end,
  service_picker_mappings = function(map)
      map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosservice call %s"))
      map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosservice call %s"))
  end,
  param_picker_mappings = function(map)
      map("n", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosparam set %s"))
      map("i", "<c-e>", vim_utils.open_terminal_with_format_cmd_entry("rosparam set %s"))
  end
}
EOF
```

## ⚙️  Config

```vim

" Search files in current package
nnoremap <leader>fp <cmd>lua require('ros-nvim.telescope.package').search_package()<cr>
" Live grep in current package
nnoremap <leader>fgp <cmd>lua require('ros-nvim.telescope.package').grep_package()<cr>

" #### ROS Introspection ####
" Topics list & info
nnoremap <leader>rtl <cmd>lua require('ros-nvim.telescope.pickers').topic_picker()<cr>
" Nodes list & info
nnoremap <leader>rnl <cmd>lua require('ros-nvim.telescope.pickers').node_picker()<cr>
" Services list & info
nnoremap <leader>rsl <cmd>lua require('ros-nvim.telescope.pickers').service_picker()<cr>
" Service definitions list & info
nnoremap <leader>rds <cmd>lua require('ros-nvim.telescope.pickers').srv_picker()<cr>
" Message definitions list & info
nnoremap <leader>rdm <cmd>lua require('ros-nvim.telescope.pickers').msg_picker()<cr>
" Params list & values
nnoremap <leader>rpl <cmd>lua require('ros-nvim.telescope.pickers').param_picker()<cr>

" Build entire workspace
nnoremap <leader>bb <cmd>lua require('ros-nvim.build').catkin_make()<cr>
" Build current package
nnoremap <leader>bp <cmd>lua require('ros-nvim.build').catkin_make_pkg()<cr>

" Execute current rostest
nnoremap <leader>rt <cmd>lua require('ros-nvim.test').rostest()<cr>

```
