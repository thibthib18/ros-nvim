# ros-nvim
ROS Integration in Neovim

Features

Search files with Telescope in the current ROS package
Live grep with Telescope in the current ROS package
Build current ROS package in split terminalpackage
Execute current ROS test in split terminal

Telescope extensions for ROS introspection

Nodes list & info
Topics list & info
Services list & info
Msgs list & info
Srvs list & info



Requirements

Nvim >= 0.5
nvim-telescope

Installation

With your plugin manager of choice:

vim-plug:
```vim
Plug 'thibthib18/ros-nvim'
```

Setup

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

Example config

```vim
" Build
nnoremap <leader>bb <cmd>lua require('ros-nvim').catkin_make()<cr>
nnoremap <leader>bp <cmd>lua require('ros-nvim').catkin_make_pkg()<cr>

" Test
nnoremap <leader>rt <cmd>lua require('ros-nvim').rostest()<cr>


" Node/topic/service info
nnoremap <leader>rtl <cmd>lua require('ros-nvim').topic_picker()<cr>
nnoremap <leader>rnl <cmd>lua require('ros-nvim').node_picker()<cr>
nnoremap <leader>rsl <cmd>lua require('ros-nvim').service_picker()<cr>

" Msg/srv
nnoremap <leader>rds <cmd>lua require('ros-nvim').srv_picker()<cr>
nnoremap <leader>rdm <cmd>lua require('ros-nvim').msg_picker()<cr>

" Search in current ROS package
nnoremap <leader>fp <cmd>lua require('ros-nvim').search_package()<cr>
nnoremap <leader>fgp <cmd>lua require('ros-nvim').grep_package()<cr>
```
