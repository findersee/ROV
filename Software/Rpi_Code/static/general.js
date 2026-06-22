/**
 * Handles sensor values and some simple GUI updates
 * https://github.com/trolllabs/eduROV
 */

var keycodes = {l:76, c:86, esc:27, enter:13, u:85};
var MOTOR_KEYS = [87, 83, 65, 68, 88, 90];
var stat = {light:false, armed:false, roll_ui:true, cinema:false,
            video_rotation:0};
var sensors = {time:0, temp:0, pressure:0, humidity:0, pitch:0, roll:0, yaw:0,
            tempWater:0, pressureWater:0, batteryVoltage:0, free_space:0,
            cpu_temp:0,leak_front:1,leak_rear:1,leak_high:1,leak_low:1};
var critical = {voltage:13.0, disk_space:300.0, cpu_temp:80.0};
var warning =  {voltage:15.0, cpu_temp:70.0, pressureWater:100.0};

var sensor_interval = 500;
var interval;
var padTimer = null;
var padTimestamp = 0;
var padTimeCount = 0;
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function send_keydown(keycode){
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "/keydown="+keycode, true);
    xhttp.setRequestHeader("Content-Type", "text/html");
    xhttp.send(null);
}

function handle_in_browser(keycode){
    if (MOTOR_KEYS.indexOf(keycode) > -1 && !stat.armed){
        if (confirm("The ROV is not armed, do you want to arm it?")) {
            toggle_armed();
        }
        return true;
    } else if (keycode == keycodes.enter){
        toggle_armed();
        return true;
    } else if (keycode == keycodes.esc && stat.cinema){
        toggle_cinema();
        return true;
    } else if (keycode == keycodes.c){
        toggle_cinema();
        return true;
    }
}

function toggle_cinema(){
    stat.cinema = !stat.cinema;
    set_cinema(stat.cinema);
}

function sendTrimValue(Type,Value){
	var xhttp = new XMLHttpRequest();
	xhttp.open("GET", "/"+Type+"_Trim="+Value, true);
    xhttp.setRequestHeader("Content-Type", "text/html");
    xhttp.send(null);
}

function getTrimValue(){
	
}

function updateNeutral(val){
	
	sendTrimValue("Neutral",val);
	document.getElementById('NeutralValue').value=val;
}

function settings(val){

    var xhttp = new XMLHttpRequest();
    xhttp.open("POST", "/settings", true);
    xhttp.setRequestHeader("Content-Type", "application/json");
    xhttp.send(JSON.stringify({ "trim":1.3,"neutral":33.1}));
}



function toggle_light(){
    var btn = document.getElementById("lightBtn");
    if(stat.light){
        btn.className = btn.className.replace(" active", "");
    }else{
        btn.className += " active";
    }
    stat.light = !stat.light;
    send_keydown(keycodes.l);
}

function toggle_armed(){
    var btn = document.getElementById("armBtn");
    if(stat.armed){
        btn.className = btn.className.replace(" active", "");
        send_keydown(keycodes.u);
    }else{
        btn.className += " active";
    }
    stat.armed = !stat.armed;
    refresh_ui();
}

function set_update_frequency(){
    var interval = prompt("Set sensor update interval in ms",sensor_interval);
    if (interval){
        if (interval<30){
            alert('Sensor frequency can not be less than 30 ms');
            interval = 30;
        }
        sensor_interval = interval;
    }
}

function toggle_roll(){
    var btn = document.getElementById("rollBtn");
    if(stat.roll_ui){
        document.getElementById("rollOverlay").style.visibility = "hidden";
        btn.className = btn.className.replace(" active", "");
    }else{
        document.getElementById("rollOverlay").style.visibility = "visible";
        btn.className += " active";
    }
    stat.roll_ui = !stat.roll_ui;
}

function stop_rov(){
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "stop", true);
    xhttp.setRequestHeader("Content-Type", "application/text");
    xhttp.send();
}

function rotate_image(){
    stat.video_rotation += 180;
    rotation = stat.video_rotation;
    document.getElementById("image").style.transform =
        `rotate(${rotation}deg)`;
}

function get_sensor(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            var response = JSON.parse(this.responseText);
            for (var key in response) {
                sensors[key] = response[key];
            }
            refresh_ui();
        }
    };
    xhttp.open("GET", "sensor.json", true);
    xhttp.send();

    // Reset interval
    interval = setInterval(function () {
        clearInterval(interval);
        get_sensor();
    }, sensor_interval);
}

function refresh_ui(){
    var roll_val = sensors.roll
    document.getElementById("rollOverlay").style.transform =
        `rotate(${roll_val}deg)`;

    for (var key in sensors){
        var element = document.getElementById(key);
        if (element){
            var val = sensors[key];
            if (isNaN(val)){
                element.innerHTML = val;
            } else{
                element.innerHTML = val.toFixed(1);
            }
        }
    }

    // Check critical system values
    var voltElem = document.getElementById("voltageTr");
    var cpuElem = document.getElementById("cpuTr");
    var frontElem = document.getElementById("leakFTr");
    var rearElem = document.getElementById("leakRTr");
    var highElem = document.getElementById("leakHTr");
    var lowElem = document.getElementById("leakLTr");
    var presElem = document.getElementById("pressureWater");
    if (sensors.batteryVoltage < critical.voltage){
        voltElem.className = " table-danger";
    } else{
        voltElem.className = voltElem.className.replace(" table-danger", "");
    }
    if (sensors.cpu_temp > critical.cpu_temp){
        cpuElem.className = " table-danger";
	}else if(sensors.cpu_temp > warning.cpu_temp){
		cpuElem.className = " table-warning";
	}
	else{
		cpuElem.className = cpuElem.className.replace(" table-danger", "");
		cpuElem.className = cpuElem.className.replace(" table-warning", "");
	}
    if (sensors.leak_front < 1){
        frontElem.className = " table-danger";
    } else{
        frontElem.className = frontElem.className.replace(" table-danger", "");
    }
    if (sensors.leak_rear < 1){
        rearElem.className = " table-danger";
    } else{
        rearElem.className = rearElem.className.replace(" table-danger", "");
    }
    if (sensors.leak_high < 1){
        highElem.className = " table-danger";
    } else{
        highElem.className = highElem.className.replace(" table-danger", "");
    }
    if (sensors.leak_low < 1){
        lowElem.className = " table-danger";
    } else{
        lowElem.className = lowElem.className.replace(" table-danger", "");
    }
	if (sensors.pressureWater > critical.pressureWater){
		presElem.className = " table-danger";
	} 
	else if(sensors.pressureWater > warning.pressureWater){
		presElem.className = " table-warning";
	}
	else{
		presElem.className = presElem.className.replace(" table-danger", "");
		presElem.className = presElem.className.replace(" table-warning", "");
	}


}


function connect_pad(){

    if(navigator.getGamepads()[0] != null){
        readController();
        document.getElementById("ConnectBtn").disabled = true
        document.getElementById("DisconnectBtn").disabled = false
        send_gamepad_state(true)
    }
    else
        console.log(navigator.getGamepads())
}

function disconnect_pad(){
    
    if(padTimer != null)
        clearInterval(padTimer);
        
    document.getElementById("ConnectBtn").disabled = false
    document.getElementById("DisconnectBtn").disabled = true
    send_gamepad_state(false)

}

function refresh(){

    //console.log(navigator.getGamepads()[0].buttons);
    if(navigator.getGamepads()[0] != null){
    document.getElementById("ctrlBox").innerHTML = (navigator.getGamepads()[0].id.split(" "));
    document.getElementById("ConnectBtn").disabled = false
    }
}

function readController(){

    if(navigator.getGamepads()[0] != null){
        padTimer = setInterval(() => {
            const myGamepad = navigator.getGamepads()[0];
            var axes = myGamepad.axes//[myGamepad.axes[0],myGamepad.axes[1],myGamepad.axes[2],myGamepad.axes[3]];
            var buttons = []// myGamepad.buttons;
            //console.log(myGamepad.timestamp);
            
            for(let index = 0; index < myGamepad.buttons.length; ++index){
            buttons.push(myGamepad.buttons[index].value);
            
            }
            
            if(myGamepad.timestamp > padTimestamp){
                padTimestamp = myGamepad.timestamp;

                send_gamepad(axes,buttons);
            }
            else if(padTimeCount >= 15){
                send_gamepad(axes,buttons);
            }
            else
                padTimeCount++;
        }, 100); 
    }
    else{
        clearInterval(padTimer);
        console.log("No controller present!");
        }
} 

    

function send_gamepad(axes,buttons){
    padTimeCount = 0;
    var xhttp = new XMLHttpRequest();
    xhttp.open("POST", "/pad", true);
    xhttp.setRequestHeader("Content-Type", "application/json");
    xhttp.send(JSON.stringify({ "axes":axes,"buttons":buttons}));

}

function send_gamepad_state(state){
    var xhttp = new XMLHttpRequest();
    xhttp.open("POST", "/ConnectedPad", true);
    xhttp.setRequestHeader("Content-Type", "application/json");
    xhttp.send(JSON.stringify({ "state":state}));

}

function videoGrab(){
    var link = document.getElementById('downloadlink');
    link.download = new Date().toISOString().split('T')[0] +'_'+ new Date().toISOString().split('T')[1].split('.')[0] + '.jpeg'

}

get_sensor();
