/**
 * Handles keydown and keyup events
 * https://github.com/trolllabs/eduROV
 */

var last_key;

document.onkeydown = function(evt) {
    evt = evt || window.event;
    console.log(evt.keyCode)
    console.log(" ")
    console.log(last_key)
    if (evt.keyCode != last_key){
        console.log("inside")
        last_key = evt.keyCode;
        if (!handle_in_browser(evt.keyCode)){
            send_keydown(evt.keyCode);
        }
    }
}

document.onkeyup = function(evt) {
    last_key = 0;
    send_keyup(evt.keyCode);
}

function send_keydown(keycode){
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "/keydown="+keycode, true);
    xhttp.setRequestHeader("Content-Type", "text/html");
    xhttp.send(null);
}

function send_keyup(keycode){
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "/keyup="+keycode, true);
    xhttp.setRequestHeader("Content-Type", "text/html");
    xhttp.send(null);
}
