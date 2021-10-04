# 🐢 ros-nvim 🐢
ROS in Neovim.

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

" ROS Introspection
nnoremap <leader>rtl <cmd>lua require('ros-nvim.telescope.pickers').topic_picker()<cr>
nnoremap <leader>rnl <cmd>lua require('ros-nvim.telescope.pickers').node_picker()<cr>
nnoremap <leader>rsl <cmd>lua require('ros-nvim.telescope.pickers').service_picker()<cr>
nnoremap <leader>rds <cmd>lua require('ros-nvim.telescope.pickers').srv_picker()<cr>
nnoremap <leader>rdm <cmd>lua require('ros-nvim.telescope.pickers').msg_picker()<cr>

" Build entire workspace
nnoremap <leader>bb <cmd>lua require('ros-nvim.build').catkin_make()<cr>
" Build current package
nnoremap <leader>bp <cmd>lua require('ros-nvim.build').catkin_make_pkg()<cr>

" Execute current rostest
nnoremap <leader>rt <cmd>lua require('ros-nvim.test').rostest()<cr>

```
