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
- Build current ROS package in split terminal
- Execute current ROS test in split terminal

### 🕵️  Several Telescope extensions for ROS introspection

- Nodes list & info
- Topics list & info
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
}
```

## ⚙️  Setup

In Lua:
```lua
require 'ros-nvim'.setup {
  -- path to your catkin workspace
  catkin_ws_path = "~/catkin_ws",
  -- terminal height for build / test
  terminal_height = 8
}
```

In Vim, simply enclose it in a lua block:
```vim
lua << EOF
require 'ros-nvim'.setup {
  -- path to your catkin workspace
  catkin_ws_path = "~/catkin_ws",
  -- terminal height for build / test
  terminal_height = 8
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
