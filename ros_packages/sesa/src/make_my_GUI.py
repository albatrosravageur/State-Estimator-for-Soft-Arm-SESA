#!/usr/bin/python

from codecs import decode
import sys
import yaml
from PyQt4 import QtGui
from GUI_class import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import rospkg, rospy

rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
config_file = package_path+'/config/'+rospy.get_param('/config_file')

def set_state(parameter, state):
    with open(config_file) as f:
        doc = yaml.safe_load(f)
    if type(parameter) is not list:
        raise "parameter not a list in make_my_GUI"
    if len(parameter) ==1:
        doc[parameter[0]] = state
    if len(parameter) ==2:
        doc[parameter[0]][parameter[1]] = state
    if len(parameter) ==3:
        doc[parameter[0]][parameter[1]][parameter[2]] = state        

    with open(config_file, 'w') as f:
        yaml.dump(doc, f)

def set_state_in_list(parameter, state, index):
    with open(config_file) as f:
        doc = yaml.safe_load(f)
    if type(parameter) is not list:
        raise "parameter not a list in make_my_GUI"
    if len(parameter) ==1:
        doc[parameter[0]][index] = state
    if len(parameter) ==2:
        doc[parameter[0]][parameter[1]][index] = state
    if len(parameter) ==3:
        doc[parameter[0]][parameter[1]][parameter[2]][index] = state        

    with open(config_file, 'w') as f:
        yaml.dump(doc, f)

def get_state(parameter):
    with open(config_file) as f:
        doc = yaml.load(f)
        f.close()
    if type(parameter) is not list:
        raise "parameter not a list in make_my_GUI"
    if len(parameter) == 1:
        state = doc[parameter[0]] 
    if len(parameter) == 2:
        state =  doc[parameter[0]][parameter[1]]
    if len(parameter) == 3:
        state = doc[parameter[0]][parameter[1]][parameter[2]]      
    return state

def set_rostopic_echo(ui):
    topics = get_state(['topics_per_mode'])
    for topic in topics[get_state(['mode'])]:
        widget = getattr(ui,'topic_'+str(topics[get_state(['mode'])].index(topic)+1))
        widget.setText(topic)
        if topic is not 'rosout':
            widget.setEnabled(topic is not '')

def get_state_in_list(parameter, index):
    with open(config_file) as f:
        doc = yaml.safe_load(f)
        f.close()
    if type(parameter) is not list:
        raise "parameter not a list in make_my_GUI"
    if len(parameter) ==1:
        state = doc[parameter[0]][index]
    if len(parameter) ==2:
        state = doc[parameter[0]][parameter[1]][index]
    if len(parameter) ==3:
        state = doc[parameter[0]][parameter[1]][parameter[2]][index]         
    return state

def set_combox(widget, param, states):
    state = get_state(param)
    widget.clear()
    widget.addItem(state)
    for other_state in states:
        if state not in other_state: 
            widget.addItem(other_state)

def enable_widgets(ui):
    mode, source = ui.mode.currentText(), ui.source.currentText()

    ui_rosbag_play = [ui.play_file,ui.play_rate,ui.play_start]
    ui_rosbag_record=[ui.record_file ,ui.tick_quat  ,ui.tick_acc  ,ui.tick_marker]
    ui_rviz=[ui.markers_size, ui.tick_show_markers, ui.tick_RVIZ ]
    ui_arduino = [ui.arduino_baud,  ui.arduino_boot, ui.arduino_port ]
    ui_prev_calibration = [ ui.calibration_saved_file, ui.calibration_use_saved ]
    ui_calibrate = [ ui.calibrate_file ]
    ui_model = [ ui.arm_length, ui.arm_radius, ui.N_Markers, ui.pos_IMU_1, ui.pos_IMU_2, ui.pos_IMU_3, ui.pos_IMU_4 \
        , ui.pos_IMU_5, ui.pos_IMU_6, ui.pos_IMU_7, ui.pos_IMU_8, ui.seg_1, ui.seg_2, ui.seg_3, ui.seg_4, ui.seg_5 ,ui.seg_6 \
            , ui.seg_7 , ui.seg_8, ui.marker_1, ui.marker_2, ui.marker_3, ui.marker_4, ui.marker_5, ui.marker_6, ui.marker_7, ui.marker_8\
             ,  ui.base_quat_w,  ui.base_quat_x,  ui.base_quat_y,  ui.base_quat_z, ui.imuActivate_1, ui.imuActivate_2, ui.imuActivate_3, \
                 ui.imuActivate_4, ui.imuActivate_5, ui.imuActivate_6, ui.imuActivate_7, ui.imuActivate_8]

    for module in [ui_rviz, ui_rosbag_record, ui_model]:
        for widget in module:
                widget.setEnabled('stream' in mode)
                try:
                    if module is ui_rviz: widget.setChecked('stream' in mode)
                    if module is ui_rosbag_record and 'calibrate' in mode : widget.setChecked(False)
                except:
                    continue
    
    for widget in ui_calibrate:
        widget.setEnabled('arduino' in source and 'calibrate' in mode)
    
    for widget in ui_prev_calibration:
        widget.setEnabled('arduino' in source and 'stream' in mode)
    
    for widget in ui_arduino:
        if 'arduino' in source:
            widget.setEnabled(True)
        else:
            widget.setEnabled(False)
            try:
                widget.setChecked(False)
            except:
                continue
    
    for widget in ui_rosbag_play:
        widget.setEnabled('my_rosbag' in source  and 'stream' in mode )

    ui.button_letsgo.setEnabled('arduino' in source or 'stream' in mode)

def cb_mode(ui):
    mode = str(ui.mode.currentText())
    set_state(['mode'], mode)
    enable_widgets(ui)
    ui.arduino_boot.setChecked(True) #State changed, so reboot the arduino
    set_rostopic_echo(ui)
    if 'calibrate' in mode:
        set_state(['source'],'arduino')
        set_combox(ui.source, ['source'], ['arduino','my_rosbag','no_source'])
    ui.source.setEnabled('stream' in mode)
    ui.topic_1.setChecked(get_state(['topics',str(ui.topic_1.text())]))
    ui.topic_2.setChecked(get_state(['topics',str(ui.topic_2.text())]))
    ui.topic_3.setChecked(get_state(['topics',str(ui.topic_3.text())]))
    if 'stream' in mode: ui.topic_4.setChecked(get_state(['topics','markers'])) 

def cb_source(ui):
    set_state(['source'], str(ui.source.currentText()))
    enable_widgets(ui)

def make_my_GUI():
    
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    # Bottom buttons
    ui.button_letsgo.clicked.connect(MainWindow.close)
    ui.button_quit.clicked.connect(sys.exit)
    
    # MAIN PARAMETERS
    ## Source
    ui.source.currentIndexChanged.connect(lambda:cb_source(ui))
    set_combox(ui.source, ['source'], ['arduino','my_rosbag','no_source'])
    ## Mode
    ui.mode.currentIndexChanged.connect(lambda:cb_mode(ui))
    set_combox(ui.mode, ['mode'],["stream","calibrate"])

    # MODEL
    ## Length arm
    ui.arm_length.valueChanged.connect(lambda:set_state(['arm_length'],  ui.arm_length.value()))
    ui.arm_length.setValue(get_state(['arm_length']))
    ## Radius arm
    ui.arm_radius.valueChanged.connect(lambda:set_state(['arm_radius'],  ui.arm_radius.value()))
    ui.arm_radius.setValue(get_state(['arm_radius']))
    ## #IMUs
    ## positions
    ui.pos_IMU_1.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_1.value(),1))
    ui.pos_IMU_2.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_2.value(),2))
    ui.pos_IMU_3.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_3.value(),3))
    ui.pos_IMU_4.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_4.value(),4))
    ui.pos_IMU_5.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_5.value(),5))
    ui.pos_IMU_6.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_6.value(),6))
    ui.pos_IMU_7.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_7.value(),7))
    ui.pos_IMU_8.valueChanged.connect(lambda:set_state_in_list(['imus','positions'], ui.pos_IMU_8.value(),8))
    ui.pos_IMU_1.setValue(get_state_in_list(['imus','positions'],1))
    ui.pos_IMU_2.setValue(get_state_in_list(['imus','positions'],2))
    ui.pos_IMU_3.setValue(get_state_in_list(['imus','positions'],3))
    ui.pos_IMU_4.setValue(get_state_in_list(['imus','positions'],4))
    ui.pos_IMU_5.setValue(get_state_in_list(['imus','positions'],5))
    ui.pos_IMU_6.setValue(get_state_in_list(['imus','positions'],6))
    ui.pos_IMU_7.setValue(get_state_in_list(['imus','positions'],7))
    ui.pos_IMU_8.setValue(get_state_in_list(['imus','positions'],8))

    ## #segments per IMU
    ui.seg_1.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_1.value())),0))
    ui.seg_2.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_2.value())),1))
    ui.seg_3.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_3.value())),2))
    ui.seg_4.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_4.value())),3))
    ui.seg_5.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_5.value())),4))
    ui.seg_6.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_6.value())),5))
    ui.seg_7.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_7.value())),6))
    ui.seg_8.valueChanged.connect(lambda:set_state_in_list(['segments'],int(str(ui.seg_8.value())),7))
    ui.seg_1.setValue(get_state_in_list(['segments'],0))
    ui.seg_2.setValue(get_state_in_list(['segments'],1))
    ui.seg_3.setValue(get_state_in_list(['segments'],2))
    ui.seg_4.setValue(get_state_in_list(['segments'],3))
    ui.seg_5.setValue(get_state_in_list(['segments'],4))
    ui.seg_6.setValue(get_state_in_list(['segments'],5))
    ui.seg_7.setValue(get_state_in_list(['segments'],6))
    ui.seg_8.setValue(get_state_in_list(['segments'],7))
    ##Markers
    ui.N_Markers.valueChanged.connect(lambda:set_state(['markers','amount'],  ui.N_Markers.value()))
    ui.N_Markers.setValue(int(get_state(['markers','amount'])))
    ui.marker_1.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_1.value(),0))
    ui.marker_2.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_2.value(),1))
    ui.marker_3.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_3.value(),2))
    ui.marker_4.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_4.value(),3))
    ui.marker_5.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_5.value(),4))
    ui.marker_6.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_6.value(),5))
    ui.marker_7.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_7.value(),6))
    ui.marker_8.valueChanged.connect(lambda:set_state_in_list(['markers','positions'], ui.marker_8.value(),7))
    ui.marker_1.setValue(get_state_in_list(['markers','positions'],0))
    ui.marker_2.setValue(get_state_in_list(['markers','positions'],1))
    ui.marker_3.setValue(get_state_in_list(['markers','positions'],2))
    ui.marker_4.setValue(get_state_in_list(['markers','positions'],3))
    ui.marker_5.setValue(get_state_in_list(['markers','positions'],4))
    ui.marker_6.setValue(get_state_in_list(['markers','positions'],5))
    ui.marker_7.setValue(get_state_in_list(['markers','positions'],6))
    ui.marker_8.setValue(get_state_in_list(['markers','positions'],7))

    ## Markers activation #TODO #1
    ui.imuActivate_1.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_1.isChecked()), 0))
    ui.imuActivate_2.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_2.isChecked()), 1))
    ui.imuActivate_3.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_3.isChecked()), 2))
    ui.imuActivate_4.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_4.isChecked()), 3))
    ui.imuActivate_5.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_5.isChecked()), 4))
    ui.imuActivate_6.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_6.isChecked()), 5))
    ui.imuActivate_7.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_7.isChecked()), 6))
    ui.imuActivate_8.stateChanged.connect(lambda:set_state_in_list(['imus','enabled'], bool(ui.imuActivate_8.isChecked()), 7))

    ui.imuActivate_1.setChecked(get_state_in_list(['imus','enabled'],0))
    ui.imuActivate_2.setChecked(get_state_in_list(['imus','enabled'],1))
    ui.imuActivate_3.setChecked(get_state_in_list(['imus','enabled'],2))
    ui.imuActivate_4.setChecked(get_state_in_list(['imus','enabled'],3))
    ui.imuActivate_5.setChecked(get_state_in_list(['imus','enabled'],4))
    ui.imuActivate_6.setChecked(get_state_in_list(['imus','enabled'],5))
    ui.imuActivate_7.setChecked(get_state_in_list(['imus','enabled'],6))
    ui.imuActivate_8.setChecked(get_state_in_list(['imus','enabled'],7))

    ## quaternions
    ui.base_quat_w.valueChanged.connect(lambda:set_state(['imus','offset_quaternion','w'], float(ui.base_quat_w.value())))
    ui.base_quat_x.valueChanged.connect(lambda:set_state(['imus','offset_quaternion','x'], float(ui.base_quat_x.value())))
    ui.base_quat_y.valueChanged.connect(lambda:set_state(['imus','offset_quaternion','y'], float(ui.base_quat_y.value())))
    ui.base_quat_z.valueChanged.connect(lambda:set_state(['imus','offset_quaternion','z'], float(ui.base_quat_z.value())))
    ui.base_quat_w.setValue(get_state(['imus','offset_quaternion','w']))
    ui.base_quat_x.setValue(get_state(['imus','offset_quaternion','x']))
    ui.base_quat_y.setValue(get_state(['imus','offset_quaternion','y']))
    ui.base_quat_z.setValue(get_state(['imus','offset_quaternion','z']))

    # ROSBAG
    ## Play
    ui.play_file.textChanged.connect(lambda:set_state(['my_rosbag','to_play','file'], str(ui.play_file.text())))
    ui.play_rate.valueChanged.connect(lambda:set_state(['my_rosbag','to_play','rate'], float(ui.play_rate.value())))
    ui.play_start.valueChanged.connect(lambda:set_state(['my_rosbag','to_play','start'], float(ui.play_start.value())))
    ui.play_file.setText(get_state(['my_rosbag','to_play','file']))
    ui.play_rate.setValue(get_state(['my_rosbag','to_play','rate']))
    ui.play_start.setValue(get_state(['my_rosbag','to_play','start']))

    ## Record
    ui.record_file.textChanged.connect(lambda:set_state(['my_rosbag','to_record','file'], str(ui.record_file.text())))
    ui.tick_quat.stateChanged.connect(lambda:set_state(['my_rosbag','to_record','quaternions'], bool(ui.tick_quat.isChecked())))
    ui.tick_acc.stateChanged.connect(lambda:set_state(['my_rosbag','to_record','accelerometers'], bool(ui.tick_acc.isChecked())))
    ui.tick_marker.stateChanged.connect(lambda:set_state(['my_rosbag','to_record','markers'], bool(ui.tick_marker.isChecked())))
    ui.record_file.setText(get_state(['my_rosbag','to_record','file']))
    ui.tick_quat.setChecked(get_state(['my_rosbag','to_record','quaternions']))
    ui.tick_acc.setChecked(get_state(['my_rosbag','to_record','accelerometers']))
    ui.tick_marker.setChecked(get_state(['my_rosbag','to_record','markers']))

    # VISUALIZATION
    ui.markers_size.valueChanged.connect(lambda:set_state(['markers','radius'],  float(ui.markers_size.value())))
    ui.tick_show_markers.stateChanged.connect(lambda:set_state(['markers','show'], bool(ui.tick_show_markers.isChecked())))
    ui.tick_RVIZ.stateChanged.connect(lambda:set_state(['rviz_gui'], bool(ui.tick_RVIZ.isChecked())))
    ui.markers_size.setValue(get_state(['markers','radius']))
    ui.tick_show_markers.setChecked(get_state(['markers','show']))
    ui.tick_RVIZ.setChecked(get_state(['rviz_gui']))
    
    # ARDUINO
    set_state(['arduino','boot'], False)
    ui.arduino_baud.valueChanged.connect(lambda:set_state(['arduino','baud'], int(ui.arduino_baud.value())))
    ui.arduino_boot.stateChanged.connect(lambda:set_state(['arduino','boot'], bool(ui.arduino_boot.isChecked())))
    ui.arduino_port.textChanged.connect(lambda:set_state(['arduino','port'], str(ui.arduino_port.text())))
    ui.arduino_baud.setValue(get_state(['arduino','baud']))
    ui.arduino_boot.setChecked(get_state(['arduino','boot']))
    ui.arduino_port.setText(get_state(['arduino','port']))

    # Calibration
    ui.calibrate_file.textChanged.connect(lambda:set_state(['calib','calibrate_file'], str(ui.calibrate_file.text())))
    ui.calibration_saved_file.textChanged.connect(lambda:set_state(['calib','saved_file'], str(ui.calibration_saved_file.text())))
    ui.calibration_use_saved.stateChanged.connect(lambda:set_state(['calib','use_saved'], bool(ui.calibration_use_saved.isChecked())))
    ui.use_fast_mag_calib.stateChanged.connect(lambda:set_state(['calib','use_fast_mag'], bool(ui.use_fast_mag_calib.isChecked())))
    ui.imu_calib_id.valueChanged.connect(lambda:set_state(['calib','id'], ui.imu_calib_id.value()))

    ui.calibrate_file.setText(get_state(['calib','calibrate_file']))
    ui.calibration_saved_file.setText(get_state(['calib','saved_file']))
    ui.calibration_use_saved.setChecked(get_state(['calib','use_saved']))
    ui.use_fast_mag_calib.setChecked(get_state(['calib','use_fast_mag']))
    ui.imu_calib_id.setValue(get_state(['calib','id']))
    
    # Show topics
    ui.topic_1.stateChanged.connect(lambda:set_state(['topics',str(ui.topic_1.text())],bool(ui.topic_1.isChecked())))
    ui.topic_2.stateChanged.connect(lambda:set_state(['topics',str(ui.topic_2.text())],bool(ui.topic_2.isChecked())))
    ui.topic_3.stateChanged.connect(lambda:set_state(['topics',str(ui.topic_3.text())],bool(ui.topic_3.isChecked())))
    ui.topic_4.stateChanged.connect(lambda:set_state(['topics',str(ui.topic_4.text())],bool(ui.topic_4.isChecked())))
    ui.topic_1.setChecked(get_state(['topics',str(ui.topic_1.text())]))
    ui.topic_2.setChecked(get_state(['topics',str(ui.topic_2.text())]))
    ui.topic_3.setChecked(get_state(['topics',str(ui.topic_3.text())]))
    if 'stream' in get_state(['mode']): ui.topic_4.setChecked(get_state(['topics','markers'])) 

    # Launch GUI
    MainWindow.show()
    app.exec_()
