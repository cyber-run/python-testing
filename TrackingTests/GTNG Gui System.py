# GTNG Gui System for the Areal Stimuli Provider created by Jacob Tomaszewski for his Master's Project: 15/09/23
# This is a gui system for use with the robot. The pop ups should allow ease of use
# Ensure you use the version of MoCapStream provided with this file
# This gui uses pysimplegui. IF you are not familiar with it, this file may be confusing. It would be worth quickly familiarising yourself with it first.
# Certain functionalities have not been implemented yet however the gui has been created with allowing this in mind
import PySimpleGUI as sg
import os.path
#import MoCapStream			#Un-comment this when using on lab computer
import serial
import time
from numpy import array
from numpy.linalg import norm
import numpy as np

# --------------FUNCTIONS----------------

# The calibration function. It is important to know where the centre of rotation for each servo is, so the command is
# sent to the arduino and the centre rotation is captured. The 6D rotation matrix is pulled by MoCapStream and 
# the associated rotational euler angles are extracted for display
# This is probably not needed however as calibration occurs during the movement
def calibration(arduino, rotation):
    try:
        arduino.write(bytes('c', 'utf-8'))   #Sends calibration command
        time.sleep(0.5)
        yaw, pitch = angles_from_rot(rotation[0][2], rotation[1][2], rotation[2][2], rotation[0][0], rotation[0][1])
        return True, yaw, pitch, rotation[:,0]
    except:
        sg.popup("Calibration Error: Try again")


# A function to extract yaw and pitch angles from the rotation matrix
def angles_from_rot(R31, R32, R33, R11, R21):
    yaw_centre = np.degrees(np.arctan2(R21, R11))
    pitch_centre = 90 - np.degrees(np.arctan2(-R31, np.sqrt(R32**2 + R33**2)))

    return yaw_centre, pitch_centre


# This sends a command to the arduino. Does not need to be an angle and can be any of the commands coded into the arduino
def send_angle(arduino, angle = 0, axis = 'n'):
    angles_to_send = axis + ":" + str(angle)
    arduino.write(bytes(angles_to_send, 'utf-8'))
    return 0


# A function copied from the matlab implementation of vrrotvec to extract the angle between two vectors
def magic_vrrotvec(a, b):
    # Step 1: The input vectors must be normalised
    anorm = a / np.linalg.norm(a)
    bnorm = b / np.linalg.norm(b)

    # Step 2: Calculate he axis of rotation using the cross product between the 2 vectors and normalising it
    cross = np.cross(anorm, bnorm)
    axis = cross / np.linalg.norm(cross)

    # Step 3: calculate the angle with the dot product Round down to reduce any rounding error
    dot_prod = np.min(np.dot(anorm, bnorm), 0)
    angle = np.degrees(np.arccos(dot_prod))

    return angle


# The aiming function that uses feedforward and feedback to aim at a target passed in to the function. Needs the marker labels of the device too.
# This can definitely be optimised. Possibly redesigning this to be a single pass feedback function with the overall loop in the main body of code that
# keeps calling this, and with this function moving the device then passing back the error. A possible method...
def aim_at(target, arduino, back_marker, centre_marker, front_marker, side_marker):

    # Checks to see if the target is legitimate
    if target[0] == np.nan:
        return 0, 0

    # Creates a connection to mocap stream to allow streaming of the position of the device.
    Servo = MoCapStream.MoCap(stream_type="3d_labelled")

    # The yaw aiming
    # moves the device to 0 degrees
    send_angle(arduino, 0, 'y')
    time.sleep(1)


    while True:
        # Finds the 0 degree vector
        min_vector = [Servo.marker[front_marker][0] - Servo.marker[back_marker][0], 
                      Servo.marker[front_marker][1] - Servo.marker[back_marker][1], 
                      Servo.marker[front_marker][2] - Servo.marker[back_marker][2]]
        
        # Normalise minimum vector
        norm_min = min_vector / np.linalg.norm(min_vector)

        min_yaw = np.degrees(np.arctan2(norm_min[1], norm_min[0]))
        # print(min_yaw)
        
        time.sleep(0.3)

        # Finds the direction vector
        direction = [target[0] - Servo.marker[centre_marker][0], 
                     target[1] - Servo.marker[centre_marker][1], 
                     target[2] - Servo.marker[centre_marker][2]]
        
        # Normalises direction vector
        d_norm = direction / np.linalg.norm(direction)

        # Finds the desired yaw angle via atan2 function
        yaw_angle = np.degrees(np.arctan2(d_norm[1], d_norm[0]))

        # Find the total yaw angle in terms of device movement
        total_yaw = yaw_angle - min_yaw

        # Stores this
        feedforward_yaw = total_yaw
        # total_yaw = total_yaw - ff_error

        print(total_yaw)
        
        # Checks if angle is too large else it sends to movement
        if yaw_angle > 120:
            print('Yaw too large')
        else:
            send_angle(arduino, total_yaw, 'y')
            time.sleep(0.3)
        print("Feedforward Yaw: " + str(total_yaw))
        time.sleep(1)

        # The feedback part
        yaw_error = 100
        gain = 0.8     #havent dialed it
        iteration = 1
        while yaw_error >= 0.3 and iteration < 40:
            # calculate the new direction vector
            # direction = np.subtract(target, Servo.marker[1])
            direction = [target[0] - Servo.marker[centre_marker][0], 
                         target[1] - Servo.marker[centre_marker][1], 
                         target[2] - Servo.marker[centre_marker][2]]
            d_norm = direction / np.linalg.norm(direction)

            # calculate the new barrel vector
            # barrel_vector = np.subtract(Servo.marker[2], Servo.marker[3])
            barrel_vector = [Servo.marker[front_marker][0] - Servo.marker[back_marker][0], 
                             Servo.marker[front_marker][1] - Servo.marker[back_marker][1], 
                             Servo.marker[front_marker][2] - Servo.marker[back_marker][2]]
            barrel_norm = barrel_vector / np.linalg.norm(barrel_vector)

            # calculate the dot product along the xy plane and the xy plane
            yaw_error = magic_vrrotvec([d_norm[0], d_norm[1]], [barrel_norm[0], barrel_norm[1]])
            print('Error: '+ str(yaw_error))

            # calculate if we need to add or subtract the feedback term
            # by finding the cross product of the direction vector to get a 90 degree perpendicular
            cross_dir = [-d_norm[1], d_norm[0]]

            # find the angle between this new vector and the barrel vector
            barrel_to_cross = magic_vrrotvec(cross_dir, [barrel_norm[0], barrel_norm[1]])

            # calculate if it is greater or less than 90
            # if it is greater than then it needs to be added
            if barrel_to_cross > 90:
                total_yaw = total_yaw + yaw_error * (gain * 0.2)
            elif barrel_to_cross < 90:
                total_yaw = total_yaw - yaw_error * gain
            else:
                total_yaw = total_yaw
         # indicating the angles are the same


            if total_yaw > 120 or total_yaw < 0:
                print('Yaw out of range')
                break
            else:
                total_yaw = np.around(total_yaw, 2)
                send_angle(arduino, total_yaw, 'y')
                #send_angle(arduino, 20, 'p')
                print("Sent Yaw: " + str(total_yaw))
                print("Feedback Term: " + str(yaw_error * gain))
                print(" ")
                time.sleep(0.4)

            if yaw_error > 30:
                print('!!!!!!!!!!Resent Feedforward!!!!!!!!!!!')
                send_angle(arduino, feedforward_yaw, 'y')
                discard = True
                time.sleep(0.3)
                print('Reset yaw to feedforward command')
            iteration = iteration + 1

        # the PITCH PART, essentially the same as above just on a different plane
        print("TUNING PITCH")
        send_angle(arduino, 0, 'p')
        time.sleep(1)

        min_vector = [Servo.marker[front_marker][0] - Servo.marker[back_marker][0], Servo.marker[front_marker][1] - Servo.marker[back_marker][1], Servo.marker[front_marker][2] - Servo.marker[back_marker][2]]
        norm_min = min_vector / np.linalg.norm(min_vector)
        min_pitch = np.degrees(np.arctan2(norm_min[2], norm_min[0]))
        print(min_pitch)
        time.sleep(0.3)

        direction = [target[0] - Servo.marker[centre_marker][0], target[1] - Servo.marker[centre_marker][1], target[2] - Servo.marker[centre_marker][2]]
        d_norm = direction / np.linalg.norm(direction)

        pitch_angle = np.degrees(np.arctan2(d_norm[2], d_norm[0]))

        total_pitch = pitch_angle - min_pitch
        feedforward_pitch = total_pitch
        # total_yaw = total_yaw - ff_error

        print(total_pitch)


        if pitch_angle > 120:
            print('Pitch too large')
        else:
            send_angle(arduino, total_pitch, 'p')
            time.sleep(0.3)
        print("Feedforward pitch: " + str(total_pitch))
        time.sleep(1)

        # The feedback part
        pitch_error = 100
        gain = 0.5     # Haven't dialed it

        iteration = 1
        while pitch_error >= 0.3 and iteration < 40:
            # calculate the new direction vector
            direction = [target[0] - Servo.marker[centre_marker][0], target[1] - Servo.marker[centre_marker][1], target[2] - Servo.marker[centre_marker][2]]
            d_norm = direction / np.linalg.norm(direction)

            # calculate the new barrel vector
            # barrel_vector = np.subtract(Servo.marker[2], Servo.marker[3])
            barrel_vector = [Servo.marker[front_marker][0] - Servo.marker[back_marker][0], Servo.marker[front_marker][1] - Servo.marker[back_marker][1], Servo.marker[front_marker][2] - Servo.marker[back_marker][2]]
            barrel_norm = barrel_vector / np.linalg.norm(barrel_vector)

            # calculate the dot product along the xz plane
            pitch_error = magic_vrrotvec([d_norm[0], d_norm[2]], [barrel_norm[0], barrel_norm[2]])
            print('Error: '+ str(pitch_error))

            # calculate if we need to add or subtract the feedback term
            # by finding the cross product of the direction vector to get a 90 degree perpendicular
            cross_dir = [-d_norm[2], d_norm[0]]

            # find the angle between this new vector and the barrel vector
            barrel_to_cross = magic_vrrotvec(cross_dir, [barrel_norm[0], barrel_norm[2]])

            # calculate if it is greater or less than 90
            if barrel_to_cross > 90:  # if it is greater than then it needs to be added
                total_pitch = total_pitch + pitch_error * (gain * 0.2)
            elif barrel_to_cross < 90:
                total_pitch = total_pitch - pitch_error * gain
            else:
                total_pitch = total_pitch
            # indicating the angles are the same

            if total_pitch > 120 or total_pitch < 0:
                print('Pitch out of range')
                break
            else:
                total_pitch = np.around(total_pitch, 2)
                send_angle(arduino, total_pitch, 'p')
                print("Sent Pitch: " + str(total_pitch))
                print("Feedback Term: " + str(pitch_error * gain))
                print(" ")
                time.sleep(0.4)

            if pitch_error > 50:
                print('!!!!!!!!!!Resent Feedforward!!!!!!!!!!!')
                send_angle(arduino, feedforward_pitch, 'p')
                discard = True
                time.sleep(0.3)
                print('Reset pitch to feedforward command')

            iteration = iteration + 1

        # once it gets to the end break the while loop as the servo streaming is only inside a loop
        break


    return total_yaw, total_pitch, yaw_error, pitch_error
    

# a rotation matrix generator generally used to test the calibration algorithm
def rotation_matrix(alpha, beta, gamma):
    # Convert angles to radians
    alpha_rad = np.radians(alpha)
    beta_rad = np.radians(beta)
    gamma_rad = np.radians(gamma)

    # Define the rotation matrices for each axis
    Rz = np.array([[np.cos(alpha_rad), -np.sin(alpha_rad), 0],
                   [np.sin(alpha_rad), np.cos(alpha_rad), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(beta_rad), 0, np.sin(beta_rad)],
                   [0, 1, 0],
                   [-np.sin(beta_rad), 0, np.cos(beta_rad)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(gamma_rad), -np.sin(gamma_rad)],
                   [0, np.sin(gamma_rad), np.cos(gamma_rad)]])

    # Combine the rotation matrices to get the final rotation matrix
    rotation_matrix = np.matmul(Rz, Ry)
    rotation_matrix = np.matmul(rotation_matrix, Rx)
    
    return rotation_matrix


# A popup for the gui to accept a target input manally
def make_target_popup():
    target_popup_layout = [[sg.Text('X: '),sg.Input(s = (10,20)), sg.Text('Y: '),sg.Input(s = (10,20)), sg.Text('Z: '),sg.Input(s = (10,20))]]
    choice, target = sg.Window('Enter Target Position', [[target_popup_layout],[sg.OK(s=10), sg.Cancel(s=10)]], disable_close=True, keep_on_top=True).read(close=True)
    if choice == "Cancel" or target[0] == '' or target[1] == '' or target[2] == '':
        return choice, [np.nan, np.nan, np.nan]
    else:
        return choice, [float(target[0]), float(target[1]), float(target[2])]

# A function to gather the via a marker in QTM
def gather_qns_target():
    try:
        qns_target = MoCapStream.MoCap(stream_type="3d_unlabelled")
        for i in range(1):
            try:
                time.sleep(0.009)
                temporaryTarget = [qns_target.marker[0][0], qns_target.marker[0][1], qns_target.marker[0][2]]
            except:
                sg.popup("Cannot See Target in Tent")
        qns_target.close()
        return temporaryTarget
    except:
        sg.popup("Cannot connect to QTM")
        return [np.nan, np.nan, np.nan]

#----------------Main Body-----------------
# Initial variable set up

default_qns_ip = "127.0.0.1"

x = 0
y = 1
z = 2
target = [np.nan, np.nan, np.nan]
delta_time = 0.1
arduino_connected = False
qns_connected = False
calibration_success = False
min_vector = []
front_marker = 2
centre_marker = 0
back_marker = 1
side_marker = 3
stimuli = 0
activate = False
marker_type = 0
currently_tracking = False
distance_to_fire = 100

# Create a sample rotation matrix
rotation = np.zeros([3,3])

# makes a theme to look nice
sg.theme('Default1') 

# Creating the frames to hold all the gui components
# set up the arduino frame to allow connection to any port as input by the user 
# without code needing to be changed
arduino_layout = []
arduino_layout.append(sg.Text('Arduino Port: '))
arduino_layout.append(sg.Input(key = '-PORT_ADD-'))
arduino_layout.append(sg.Button(button_text = 'Connect', enable_events=True, key = '-A_CONNECT-'))
arduino_layout = [arduino_layout,[sg.Text('Connected: '), sg.Text('False', key = '-A_CONNECTED_STATUS-')]]
arduino_frame = sg.Frame("ARDUINO", arduino_layout, s = (500, 90))

# set up the QNS to be connected with a button click
sg_qns_layout = []
sg_qns_layout.append(sg.Text("QTM Port: "))
sg_qns_layout.append(sg.Input(default_text = default_qns_ip, key = '-QNS_ADD-'))
sg_qns_layout.append(sg.Button(button_text = 'Connect', enable_events=True, key = '-QNS_CONNECT-'))
sg_qns_layout = [sg_qns_layout, [sg.Text('Connected: '), sg.Text('False', key = '-QNS_CONNECTED_STATUS-')]]
qns_frame = sg.Frame("QNS", sg_qns_layout, s = (500, 90))

marker_layout = []
marker_layout = [
                        [sg.Text("Back Marker:   "), sg.Input(key = '-BACK_MARKER-', default_text= back_marker + 1, s = (10, 2))],
                        [sg.Text("Centre Marker: "), sg.Input(key = '-CENTRE_MARKER-', default_text= centre_marker+ 1, s = (10, 2)), sg.Text("Side Marker: "), sg.Input(key = '-SIDE_MARKER-', default_text= side_marker+ 1, s = (10, 2))],
                        [sg.Text("Front Marker:   "), sg.Input(key = '-FRONT_MARKER-', default_text= front_marker+ 1, s = (10, 2))],
                        [sg.Button("Update", key = '-MARKER_UPDATE-')]
                    ]
marker_frame = sg.Frame("Marker Layout", marker_layout, s = (500, 125))
sg_slider_layout = [
                    [sg.Text('Yaw: ')],
                    [sg.Slider(range = (0, 120), default_value = 0, enable_events=True, key = '-YAW_SLIDER-', orientation = 'horizontal')],
                    [sg.Text('Pitch: ')],
                    [sg.Slider(range = (0, 120), default_value = 0, enable_events=True, key = '-PITCH_SLIDER-', orientation = 'horizontal')],
                    [sg.Button('Enable', key = '-SLIDER_BUTTON-', enable_events = True)]
                     ]

sg_slider_frame = sg.Frame("Manual Control: ", sg_slider_layout, key= '-SLIDER_FRAME-', s = (500, 200))

right_col = [[arduino_frame],[qns_frame],[sg_slider_frame], [marker_frame]]

# set up the rotation output to pull and display the current rotation
sg_rotation_layout = []
sg_rotation_layout = [
                      [sg.Text(rotation[0][0], key = '-R00-', s =(18, 1)), sg.Text(rotation[0][1], key = '-R01-', s =(18, 1)), sg.Text(rotation[0][2], key = '-R02-', s =(18, 1))],
                      [sg.Text(rotation[1][0], key = '-R10-', s =(18, 1)), sg.Text(rotation[1][1], key = '-R11-', s =(18, 1)), sg.Text(rotation[1][2], key = '-R12-', s =(18, 1))],
                      [sg.Text(rotation[2][0], key = '-R20-', s =(18, 1)), sg.Text(rotation[2][1], key = '-R21-', s =(18, 1)), sg.Text(rotation[2][2], key = '-R22-', s =(18, 1))]
                      ]

rotation_frame = sg.Frame("Robot 6D Solid Body Rotation Matrix", sg_rotation_layout, key = "-ROT_TABLE-", s = (500, 100))

# set up the current position, yaw and pitch output as a frame
sg_coords_layout = [
                    [sg.Text('X: '), sg.Text(' ', key = '-O_X-', s = (15, 1)), sg.Text('Y: '), sg.Text(' ', key = '-O_Y-', s = (15, 1)), sg.Text('Z: '), sg.Text(' ', key = '-O_Z-', s = (15, 1))],
                    [sg.Text('Global Yaw: '), sg.Text(' ', key = '-YAW-', s = (14, 1)), sg.Text('Global Pitch: '), sg.Text(' ', key = '-PITCH-', s = (20, 1))],
                    [sg.Text('Sent Yaw: '), sg.Text(' ', key = '-SENT_YAW-', s = (16, 1)), sg.Text('Sent Pitch: '), sg.Text(' ', key = '-SENT_PITCH-', s = (20, 1))],
                    [sg.Text('Error:  '), sg.Text(' ', key = '-E_YAW-', s = (19, 1)), sg.Text('Error: '), sg.Text(' ', key = '-E_PITCH-', s = (20, 1))],
                    [sg.Text('Minimum Yaw:  '), sg.Text(' ', key = '-MIN_YAW-', s = (12, 1)), sg.Text('Minimum Pitch: '), sg.Text(' ', key = '-MIN_PITCH-', s = (20, 1))]
                    ]
sg_coords_frame = sg.Frame("Current Servo Position and Rotational Angles", sg_coords_layout, key= '-COORDS-', s = (500, 150))




# set up the target position and target yaw + pitch as a frame with buttons associated
target_menu = ['Targetting',['Via QNS', 'Manual Input']]
sg_target_layout = [
                    [sg.Text("Aiming Location: ")],
                    [sg.Text('X: '), sg.Text(target[x], key = '-T_X-'), sg.Text('Y: '), sg.Text(target[y], key = '-T_Y-'), sg.Text('Z: '), sg.Text(target[z], key = '-T_Z-')],
                    [sg.Button('Manual Input', key = '-MANUAL_INPUT-', enable_events = True), sg.Button('Via QNS', key = '-QNS_INPUT-', enable_events = True)],
                    [sg.Text("Insect Location: ")],
                    [sg.Text('X: '), sg.Text(target[x], key = '-I_X-'), sg.Text('Y: '), sg.Text(target[y], key = '-I_Y-'), sg.Text('Z: '), sg.Text(target[z], key = '-I_Z-')],
                    [sg.Text("Tracking Type: "), sg.Radio("Single Marker", "Track", key = '-SINGLE_MARKER-'), sg.Radio("Multiple Markers", "Track", key = '-MULTI_MARKER-'), sg.B("Update", enable_events = True, key = '-MARKER_SELECTION_UPDATE-')],
                    [sg.B('Track Insect', key = '-MARKER_TRACKING-', enable_events = True)],
                    [sg.Text("Stimuli Remaining: "), sg.Input( stimuli, key = "-STIMULI-", enable_events = True, s = (10, 10)), sg.Button("Reload", key = "-RELOAD-")],
                    [sg.Text("Sensitivity: "), sg.Input(distance_to_fire, key = '-SENSITIVITY-', s = (17, 20)), sg.Text('mm'), sg.B("Update", key = '-SENSITIVITY_UPDATE-', enable_events = True)],
                    [sg.Button("ACTIVATE", key = "-ACTIVATE-", enable_events = True), sg.B("FIRE", key = "-FIRE-", enable_events = True)]
                   ]
sg_target_frame = sg.Frame("Targeting and Fire Control", sg_target_layout, key = '-D_COORDS-', s = (500, 325))

left_col =  [[rotation_frame], [sg_coords_frame],  [sg_target_frame], [sg.Button(button_text = "Calibrate", enable_events=True, key = '-CALIBRATE-'), sg.Button(button_text = "Aim", enable_events=True, key = '-AIM-'), sg.Exit()]]


layout = [
            [sg.Column(left_col), sg.Column(right_col)]
        ]


slider_enable = False

window = sg.Window("GTNG", layout)

while True:
    # time.sleep(delta_time)
    event, values = window.read(timeout = 20)
    print(event, values)

    angles_to_send = '0 0'
        
    # The exit command
    if event == "Exit" or event == sg.WIN_CLOSED or event in (None, 'Exit'):
        try:
            Servo.close()
        except:
            print("")

        try:
            Servo3d.close()
        except:
            print("")

        try:
            insect.close()
        except:
            print("")

        break

    # Calibration Button Pressed with arduino connected
    if event == "-CALIBRATE-" and arduino_connected == True and qns_connected == True:
        # do the calibration
        try:
            calibration_success, yaw_min, pitch_min, min_vector = calibration(arduino, rotation)
        except:
            sg.popup("Something went wrong but it was handled. Retry what you were doing")

        if calibration_success == True:
            sg.popup("Calibration Success!", keep_on_top=True)
            window['-MIN_YAW-'].update(value = yaw_min)
            window['-MIN_PITCH-'].update(value=pitch_min)

        else:
            sg.popup("Calibration not successful")   
    elif event == "-CALIBRATE-" and arduino_connected == False and qns_connected == True:
        sg.popup("Please connect to Arduino", keep_on_top=True)
        calibration_success = False
    elif event == "-CALIBRATE-" and arduino_connected == True and qns_connected == False:
        sg.popup("Please connect to QNS", keep_on_top=True)
        calibration_success = False
    elif event == "-CALIBRATE-" and arduino_connected == False and qns_connected == False:
        sg.popup("Please connect to QNS and Arduino", keep_on_top=True)
        calibration_success = False
    
    # Connection to arduino
    if event == "-A_CONNECT-":
        try:
            arduino_port = values['-PORT_ADD-']
            arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)
            arduino_connected = True
            sg.popup("Connection Successfull", keep_on_top=True)
            window['-PORT_ADD-'].update(disabled = True)
            window['-A_CONNECT-'].update(disabled = True)
            window['-A_CONNECTED_STATUS-'].update(value = 'True')
        except:
            sg.popup("Connection Unsuccessfull: Please re-enter the port exacty as you see it and ensure the Arduino is connected", keep_on_top=True)
            
    # Connection to QNS
    if event == "-QNS_CONNECT-" and qns_connected == False:
        try:
            Servo = MoCapStream.MoCap(stream_type="6d")
            Servo3d = MoCapStream.MoCap(stream_type="3d_labelled")
            sg.popup("QNS Connection Successfull: Tracking 6D object...", keep_on_top=True)
            qns_connected = True
            window['-QNS_CONNECTED_STATUS-'].update(value = 'True')
            window['-QNS_ADD-'].update(disabled = True)
            window['-QNS_CONNECT-'].update(disabled=True)
        except:
            sg.popup("QNS Connection Unsuccessfull: Please re-enter the IP exacty as you see it", keep_on_top=True)
            

    # targeting manual input to centre location
    if event == "-MANUAL_INPUT-":
        choice, target = make_target_popup()
        window['-T_X-'].update(value = target[x])
        window['-T_Y-'].update(value = target[y])
        window['-T_Z-'].update(value = target[z])
    
    # Targeting with QNS system to location
    if event == '-QNS_INPUT-':
        sg.popup("Close when target position is ready", keep_on_top=True)
        try:
            target = gather_qns_target()
            window['-T_X-'].update(value=target[x])
            window['-T_Y-'].update(value=target[y])
            window['-T_Z-'].update(value=target[z])
        except:
            sg.popup("Error: Could not gather target", keep_on_top=True)
        # sg.popup("QNS Targeting Unsuccessfull: FUNCTIONALITY NOT ADDED", keep_on_top=True)
    
    # Aiming at the desired target location
    if event == '-AIM-' and calibration_success == True:
        if target[0] == np.nan:
            sg.popup("Please enter a valid target location", keep_on_top=True)
        else:

            # returns angles from the position
            aimed_yaw_angle, aimed_pitch_angle, yaw_error, pitch_error = aim_at(target, arduino, back_marker, centre_marker, front_marker, side_marker)

            # Displays them
            yaw_from_min = np.around(aimed_yaw_angle, 2)
            pitch_from_min = np.around(aimed_pitch_angle, 2)
            window['-SENT_YAW-'].update(value = yaw_from_min)
            window['-SENT_PITCH-'].update(value = pitch_from_min)
            window['-E_YAW-'].update(value = yaw_error)
            window['-E_PITCH-'].update(value = pitch_error)


    elif event == '-AIM-' and calibration_success == False:
        sg.popup("Please calibrate first", keep_on_top=True)

    if event == '-SLIDER_BUTTON-':
        slider_enable = not slider_enable
        if slider_enable == True:
            window['-SLIDER_BUTTON-'].update('Disable')
        elif slider_enable == False:
            window['-SLIDER_BUTTON-'].update('Enable')

    if event == '-MARKER_UPDATE-':
        back_marker = int(values['-BACK_MARKER-']) - 1
        centre_marker = int(values['-CENTRE_MARKER-']) - 1
        front_marker = int(values['-FRONT_MARKER-']) - 1
        side_marker = int(values['-SIDE_MARKER-']) - 1

    if event == '-RELOAD-':
        stimuli = int(values['-STIMULI-'])
        
    if event == '-MARKER_SELECTION_UPDATE-':
        if values['-SINGLE_MARKER-'] == True:
            marker_type = 1
        elif values['-MULTI_MARKER-'] == True:
            marker_type = 2

    if event == '-ACTIVATE-':
        if activate == False:
            window['-ACTIVATE-'].update('DEACTIVATE')
            activate = True
            sg.popup("WARNING: Activation of the device requires the flight arena to be closed, with all users outside of the flight arena and wearing proper personal safety equipment", keep_on_top=True)
        elif activate == True:
            window['-ACTIVATE-'].update('ACTIVATE')
            activate = False
    
    if event == '-SENSITIVITY_UPDATE-':
        distance_to_fire = float(values['-SENSITIVITY-'])

    if event == '-FIRE-' and activate == True:
        send_angle(arduino, 0, 'f')
        stimuli = stimuli - 1
        window['-STIMULI-'].update(value=stimuli)
        
    
    # CODE TO MAKE THE OBJECT TRACK THE INSECT LOCATION   
    if event == '-MARKER_TRACKING-':
        if currently_tracking == True:
            currently_tracking = False
            window['-MARKER_TRACKING-'].update('Track Insect')
            try:
                insect.close()
            except:
                sg.popup("Cannot communicate with QTM")
        elif marker_type == 1 and currently_tracking == False:
            insect = MoCapStream.MoCap(stream_type="3d_unlabelled")
            currently_tracking = True
            window['-MARKER_TRACKING-'].update('Stop Tracking')
        elif marker_type == 2 and currently_tracking == False:
            sg.popup("Functionality not added... sorry!", keep_on_top=True)


    if slider_enable == True:
        angles_to_send = "a:" + str(values['-YAW_SLIDER-']) + " " + str(values['-PITCH_SLIDER-'])
        arduino.write(bytes(angles_to_send, 'utf-8'))


    # 6D Update, streaming the new position from QTM
    if qns_connected == True:
        try:
            index = 0
            # updates the rotation matrix
            # Pulls the 6d data and position once so everything is synched
            for row in range(3):
                for col in range(3):
                    rotation[row][col] = np.around(Servo.rotation[0][index],2)
                    index = index + 1
                    key = '-R' + str(row) + str(col) + '-'
                    window[key].update(value=rotation[row][col])
            # updates the sensed yaw and pitch
            yaw_angle, pitch_angle = angles_from_rot(rotation[2][0], rotation[2][1], rotation[2][2], rotation[0][0], rotation[1][0])
            window['-YAW-'].update(value=yaw_angle)
            window['-PITCH-'].update(value=pitch_angle)
            # Remember to pull the GTNG's position too!
            position = np.around([Servo.position[0], Servo.position[1], Servo.position[2]],2)
            window['-O_X-'].update(value=position[0])
            window['-O_Y-'].update(value=position[1])
            window['-O_Z-'].update(value=position[2])
            window.refresh()
        except:
            sg.popup("Cannot Pull 6D Data. Please restart QTM and this software", keep_on_top=True)
            break

    if currently_tracking == True:
        try:
            window['-I_X-'].update(value = np.around(insect.marker[0][0],3))
        except:
            window['-I_X-'].update(value= np.nan)

        try:
            window['-I_Y-'].update(value=np.around(insect.marker[0][1],3))
        except:
            window['-I_Y-'].update(value=np.nan)

        try:
            window['-I_Z-'].update(value=np.around(insect.marker[0][2],3))
        except:
            window['-I_Z-'].update(value=np.nan)

            
    # The Firing Control Part DO THIS NEXT!
    if activate == True and currently_tracking == True and stimuli != 0 and qns_connected == True:
        start = time.time()
        # we need to find the distance between the target to the line of the barrel vector
        # first we need to calculate the barrel direction vector
        barrel_vector = [Servo3d.marker[front_marker][0] - Servo3d.marker[back_marker][0],
                         Servo3d.marker[front_marker][1] - Servo3d.marker[back_marker][1],
                         Servo3d.marker[front_marker][2] - Servo3d.marker[back_marker][2]]
        barrel_norm = barrel_vector / np.linalg.norm(barrel_vector)
        while True:
            finish = time.time()
            if finish - start > 1:              ##<- THIS IS A BOTTLE NECK. A BIG BOTTLENECK AFFECTING PERFORMANCE AS THE BREAK POINT CAN ONLY OCCUR ONCE EVERY >
                #print(Servo3d.marker)

                #print(insect.marker)
                #for i in range(1):
                try:
                    target = [insect.marker[0][0], insect.marker[0][1], insect.marker[0][2]]
                        #print("target " + str(target))
                except:
                    target = [np.nan, np.nan, np.nan]
                    print('Insect not in view of cameras')
                    break
                #scalar1 = 0
                # while scalar1 <= 1000:
                #     try:
                #         if target[0] > (Servo3d.marker[front_marker][0] + scalar1 * barrel_norm[0]) - distance_to_fire and target[0] < (Servo3d.marker[2][0] + scalar1 * barrel_norm[0]) + distance_to_fire and target[1] > (Servo3d.marker[front_marker][1] + scalar1 * barrel_norm[1]) - distance_to_fire and target[1] < (Servo3d.marker[front_marker][1] + scalar1 * barrel_norm[1]) + distance_to_fire and target[2] > (Servo3d.marker[front_marker][2] + scalar1 * barrel_norm[2]) - distance_to_fire and target[2] < (Servo3d.marker[front_marker][2] + scalar1 * barrel_norm[2]) + distance_to_fire:
                #             send_angle(arduino, 0, 'f')
                #             stimuli = stimuli - 1
                #             window['-STIMULI-'].update(value=stimuli)
                #             break
                #         scalar1 = scalar1 + 100
                #     except:
                #         print("")

                try:
                    # We use triangle geometry to find the distance between the barrel vector and the insect
                    # Step 1: project a point 1050 mm away from the device
                    pointb = [1000 * barrel_norm[0] + Servo3d.marker[front_marker][0], 1000 * barrel_norm[1] + Servo3d.marker[front_marker][1], 1000 * barrel_norm[2] + Servo3d.marker[front_marker][2]]

                    # step 2: find lengths of the sides
                    base = np.sqrt((Servo3d.marker[front_marker][0] - pointb[0])**2 + (Servo3d.marker[front_marker][1] - pointb[1])**2 + (Servo3d.marker[front_marker][2] - pointb[2])**2)
                    sidea = np.sqrt((Servo3d.marker[front_marker][0] - target[0]) ** 2 + (Servo3d.marker[front_marker][1] - target[1]) ** 2 + (Servo3d.marker[front_marker][2] - target[2]) ** 2)
                    sidec = np.sqrt((pointb[0] - target[0]) ** 2 + (pointb[1] - target[1]) ** 2 + (pointb[2] - target[2]) ** 2)

                    # step 3: Use herons formula to find the area
                    area = 0.25 * np.sqrt((sidea + base + sidec) * (-sidea + base + sidec) * (sidea - base + sidec) * (sidea + base - sidec))

                    # step 4: use the rearranged area equation to find height
                    height = (2*area)/base
                    print(height)

                    # Check height against distance_to_fire
                    if height <= distance_to_fire:
                        send_angle(arduino, 0, 'f')
                        stimuli = stimuli - 1
                        window['-STIMULI-'].update(value=stimuli)
                        start = time.time()
                except:
                    print("Error in firing algorithm")

                break

    # If the stimuli are out
    elif stimuli == 0 and activate == True and currently_tracking == True and qns_connected == True:
        sg.popup("Please reload stimuli. Device deactivated")
        activate = False
        window['-ACTIVATE-'].update('ACTIVATE')
    elif activate == True and currently_tracking == False and stimuli != 0 and qns_connected == True:
        sg.popup("Please track an insect")
        activate = False
        window['-ACTIVATE-'].update('ACTIVATE')
    elif activate == True and currently_tracking == True and stimuli != 0 and qns_connected == False:
        sg.popup("Please connect to QTM")
        activate = False
        window['-ACTIVATE-'].update('ACTIVATE')

              
window.close()
