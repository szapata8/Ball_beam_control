from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import logging
from see_saw_control import SeeSawControl


# seesaw = SeeSawControl(debug_mode=True)
seesaw = SeeSawControl()
seesaw.connect()
app = Flask(__name__)
socketio = SocketIO(app)

# This section gets rid of console messages from the server (that way we don't loose error messages from the system in development after a long run time)
app.logger.disabled = True
log = logging.getLogger('werkzeug')
log.disabled = True
logging.getLogger("socketio").setLevel(logging.ERROR)
logging.getLogger("engineio").setLevel(logging.ERROR)

rotation_step_period = 1
cw_roration_flag = [False]
ccw_roration_flag = [False]
already_sending_relevant_states = [False]

# ============== WEB APP ROUTES ==============
@app.route('/')
def home():
    return render_template("home.html")

# ============== SOCKETIO EVENTS ==============
@socketio.on("set_manual_control_modality")
def set_manual_control_modality():
    print("set_manual_control_modality called!")
    seesaw.change_to_manual_control_modality()

@socketio.on("ccw_rotation")
def ccw_rotation():
    ccw_roration_flag[0] = True
    while(ccw_roration_flag[0]):
        seesaw.step_ccw()
        time.sleep(rotation_step_period)

@socketio.on("stop_ccw")
def stop_ccw():
    print("stop_ccw called!")
    ccw_roration_flag[0] = False

@socketio.on("ccw_step")
def ccw_rotation():
    seesaw.step_ccw()

@socketio.on("ccw_ustep")
def ccw_rotation():
    seesaw.microstep_ccw()

@socketio.on("set_zero")
def set_zero():
    seesaw.zero_reference_angle()

@socketio.on("cw_rotation")
def cw_rotation():
    cw_roration_flag[0] = True
    while(cw_roration_flag[0]):
        seesaw.step_cw()
        time.sleep(rotation_step_period)

@socketio.on("stop_cw")
def stop_cw():
    print("stop_cw called!")
    cw_roration_flag[0] = False

@socketio.on("cw_step")
def ccw_rotation():
    seesaw.step_cw()

@socketio.on("cw_ustep")
def ccw_rotation():
    seesaw.microstep_cw()

@socketio.on("request_relevant_states")
def ccw_rotation():
    if(not already_sending_relevant_states[0]):
        already_sending_relevant_states[0] = True
        # counter = 0
        while True:
            angle = seesaw.get_angle()
            ball_position = seesaw.get_ball_position()
            relevant_states = [angle, ball_position]
            # dummy_data = [counter -1, counter +1]
            # counter += 1
            # socketio.emit("relevant_states", dummy_data)
            # print(f"relevant_states sent!: {dummy_data}")
            socketio.emit("relevant_states", relevant_states)
            # print(f"relevant_states sent!: {relevant_states}")
            time.sleep(0.1)

@socketio.on("set_pid_control_modality")
def set_pid_control_modality():
    print("set_pid_control_modality called!")
    seesaw.change_to_pid_control_modality()
    
@socketio.on("reset_pid_gains")
def reset_pid_gains():
    print("reset_pid_gains called!")
    seesaw.set_kp(0.1)
    seesaw.set_ki(0.5)
    seesaw.set_kd(0)

@socketio.on("kp_value")
def kp_input(kp_value):
    print(f"kp_input called! kp_value: {kp_value}")
    seesaw.set_kp(float(kp_value))
    
@socketio.on("ki_value")
def ki_input(ki_value):
    print(f"ki_input called! ki_value: {ki_value}")
    seesaw.set_ki(float(ki_value))
    
@socketio.on("kd_value")
def kd_input(kd_value):
    print(f"kd_input called! kd_value: {kd_value}")
    seesaw.set_kd(float(kd_value))


@socketio.on("target_position")
def target_angle(target_position):
    print(f"target_angle called! target_angle: {target_position}")
    seesaw.set_target_angle(float(target_position))

if __name__ == "__main__":
    socketio.run(app, debug=True, use_reloader=False) # use_reloader is to be able to open microcontroller COM port