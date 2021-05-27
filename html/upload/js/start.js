'use strict';

//const vConsole = new VConsole();
//window.datgui = new dat.GUI();

const websocket_url = "";
var socket = null;

var vue_options = {
    el: "#top",
    mixins: [mixins_bootstrap],
    data: {
        ws_url: websocket_url,
        jscode: "console.log('Hello Websocket');",
        status: "closed",
    },
    computed: {
    },
    methods: {
        m5core2_connect: function () {
            if (socket)
                socket.close();
            socket = new WebSocket(this.ws_url);
            socket.binaryType = 'arraybuffer';
            socket.onopen = (event) => {
                this.status = "opened";
                console.log("websocket opened");
            };
            socket.onclose = (event) => {
                this.status = "closed";
                console.log("websocket closed");
            };
            socket.onerror = (event) => {
                console.error("websocket error");
            };
        },
        js_send: function () {
            try{
                socket.send(this.jscode);
            }catch(error){
                alert(error);
                console.error(error);
            }
        }
    },
    created: function(){
    },
    mounted: function(){
        proc_load();
    }
};
vue_add_data(vue_options, { progress_title: '' }); // for progress-dialog
vue_add_global_components(components_bootstrap);

/* add additional components */
  
window.vue = new Vue( vue_options );
