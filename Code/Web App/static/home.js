var socket = io.connect('http://127.0.0.1:5000');


function send_event(event_name){
	console.log(event_name);
    socket.emit(event_name);
}

var delayInMilliseconds = 300;

// ====================== MANUAL CONTROL FUNCTIONS

function set_manual_control_modality(){
    send_event("set_manual_control_modality");
}

function rotate_ccw() {
    send_event("ccw_rotation");
}

function step_ccw() {
    send_event("ccw_step");
}

function ustep_ccw() {
    send_event("ccw_ustep");
}

function stop_ccw() {
    send_event("stop_ccw");
}

function set_zero() {
	send_event("set_zero");
}

function rotate_cw() {
    send_event("cw_rotation");
}

function step_cw() {
    send_event("cw_step");
}

function ustep_cw() {
    send_event("cw_ustep");
}

function stop_cw() {
    send_event("stop_cw");
}

// ====================== HARDWARE STATE INDICATORS

function request_relevant_states(){
    // Send event to start receiving angle data
    socket.emit("request_relevant_states");
}

request_relevant_states();

socket.on("relevant_states", function (relevant_states){
    // console.log("received relevant_states!", relevant_states);
    var angle = Math.round(relevant_states[0]);
    var ball_position = Math.round(relevant_states[1]);
    // Obtain the span element for the angle and ball position
    var angle_span_value = document.getElementById("angle_value");
    var ball_position_span_value = document.getElementById("ball_position_value");
    // Change the value of the span
    angle_span_value.innerText = angle;
    ball_position_span_value.innerText = ball_position;
});


// socket.on('string for event', function(parameter) {
// 	console.log("hola");
// 	console.log(parameter);
// });

// ====================== PID FUNCTIONS

function set_pid_control_modality(){
    send_event("set_pid_control_modality");
}

var target_slider = document.getElementById("target_slider");
target_slider.onmouseup = function(){
    var target_position = this.value;
    console.log("target_position: ", target_position)

    // Send command over socketio
    socket.emit("target_position", target_position);
}

target_slider.oninput = function(){
    // Update text by slider
    var target_slider_value = document.getElementById("target_slider_value");
    var target_position = this.value;
    target_slider_value.innerText = " " + target_position + " cm";
}

function reset_pid_gains(){
    send_event("reset_pid_gains");
}

function set_kp_value(){
    var kp_value = parseFloat(document.getElementById("kp_input").value);
    console.log("kp_value", kp_value);
    socket.emit("kp_value", kp_value);
}

function set_ki_value(){
    var ki_value = parseFloat(document.getElementById("ki_input").value);
    console.log("ki_value", ki_value);
    socket.emit("ki_value", ki_value);
}

function set_kd_value(){
    var kd_value = parseFloat(document.getElementById("kd_input").value);
    console.log("kd_value", kd_value);
    socket.emit("kd_value", kd_value);
}