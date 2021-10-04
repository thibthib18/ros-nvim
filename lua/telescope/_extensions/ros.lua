local pickers = require "ros.telescope.pickers"

return require "telescope".register_extension {
    exports = {
        node_picker = pickers.node_picker,
        topic_picker = pickers.topic_picker,
        service_picker = pickers.service_picker,
        msg_picker = pickers.msg_picker,
        srv_picker = pickers.srv_picker
    }
}
