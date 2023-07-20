#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import Bool
from behavior_expression_skill_msg.msg import FaceInfo
from behavior_expression_skill_msg.srv import Face
from behavior_expression_skill_msg.msg import DanceStartInfo
from enum import Enum

from PA2Face import PA2Face
from DanceManager import DanceManager
from queue import Queue
from collections import deque
from queue import Queue

"""
rostopic pub -1 /behavior_expression/face/start_sign behavior_expression_skill_msg/FaceInfo "time_list: [1000, 3000, 2000]
emotion_zone: 'happy'
pleasure: 0.0
arousal: 0.0"
"""
class Mode(Enum):
    STAND_BY = 0
    COMMUNICATION = 1
    DANCE = 2
    STOP = 3
    REACTIVE = 4


class FaceSelectorNode:
    PERIOD_STANDBY = 10 #default play time (sec)
    DANCE_OFFSET = 0

    def __init__(self):
        self.dance_ing = Queue()
        self.face = PA2Face()
        self.dance = DanceManager()

        rospy.Subscriber("/behavior_expression/face/start_sign", FaceInfo, self.sub_face_start)
        rospy.Subscriber("/behavior_expression/face/stop_sign", Bool, self.sub_stop)
        rospy.Subscriber("/behavior_expression/face/dance/start_sign", DanceStartInfo, self.sub_dance_start)
        rospy.Subscriber("/behavior_expression/face/reactive/start_sign", Bool, self.sub_reactive)

        self.pub_done = rospy.Publisher('/behavior_expression/face/done_sign', Bool, queue_size=10)

        self.handle_standby = threading.Timer(self.PERIOD_STANDBY, self.thread_stand_by)
        self.handle_communication = threading.Timer(0, self.thread_communication, args=(None, None,))
        self.handle_dance = threading.Timer(0, self.thread_dance)
        self.dance_end = threading.Timer(0, self.__dance_end_event)
        rospy.on_shutdown(self.shutdown)
        self.thread_handle(Mode.STAND_BY, None)


    def shutdown(self):
        self.thread_handle(Mode.STOP, None)


    def update(self):
        try:
            self.cur_p = rospy.get_param("/behavior_expression/emotion/pleasure")
            self.cur_a = rospy.get_param("/behavior_expression/emotion/arousal")
            self.cur_zone = rospy.get_param("/behavior_expression/emotion/zone")
            return True
        except Exception as e:
            rospy.logerr("iTAMP::FaceSelectorNode::timer_start(): check rosparam -> %s", e)
        return False

    def send_cmd(self, name, level, speed, times):
        rospy.wait_for_service('/behavior_expression/face/param')
        try:
            srv = rospy.ServiceProxy('/behavior_expression/face/param', Face)
            res = srv(name=name, level=level, speed=speed, times=times)
            print("SEND: ", name)
            return res.result
        except Exception as e:
            rospy.logerr("iTAMP::FaceSelectorNode::send_cmd() to FaceControllerNode: %s", e)
        return False

    def thread_handle(self, mode: Mode, msg):
        try:
            print("=======MODE=====: ", mode)
            print(msg)
            if mode == Mode.STAND_BY:
                #print("STAND-BY======")
                self.handle_standby = threading.Timer(self.PERIOD_STANDBY, self.thread_stand_by)
                self.handle_standby.start()
            elif mode == Mode.COMMUNICATION:
                #print("SPEECH======")
                self.speech_durations, self.play_time, self.play_result = deque(msg.time_list), 0, True
                name, level = self.face.shortest(msg.emotion_zone, msg.pleasure, msg.arousal)
                self.handle_communication = threading.Timer(0, self.thread_communication, args=(name, level,))
                self.handle_communication.start()

            elif mode == Mode.STOP:
                #print("STOP======")
                self.dance_ing.queue.clear()
                self.handle_standby.cancel()
                self.handle_communication.cancel()
                self.handle_dance.cancel()
                self.dance_end.cancel()

            elif mode == Mode.DANCE:
                #print("DANCE======")
                self.dance_start = 0
                self.dance_ing.queue.clear()
                self.dance_file_path, self.dance_play_time = msg.path, (msg.play_time - self.DANCE_OFFSET)

                self.dance.file_load(self.dance_file_path)
                self.dance_end = threading.Timer((self.dance_play_time/1000.0), self.__dance_end_event)
                self.dance_end.start()
                self.handle_dance = threading.Timer(0, self.thread_dance)
                self.handle_dance.start()

            elif mode == Mode.REACTIVE:
                #print("REACTIVE======")
                success = False
                if self.update() is True:
                    face_name, level = self.face.shortest(self.cur_zone, self.cur_p, self.cur_a)
                    success = self.send_cmd(face_name, level, 1, 1)
                self.pub_done.publish(success)


        except Exception as e:
            rospy.logerr("iTAMP::FaceSelectorNode::thread_handle(): %s", e)


    def __dance_end_event(self):
        self.dance_ing.put(True)

    def thread_dance(self):
        result = True
        self.dance_face_info = self.dance.get_face()

        if (self.dance_face_info == True) or (self.dance_face_info == False):
            result = self.dance_face_info
            self.dance.file_load(self.dance_file_path)
            self.dance_start = 0
            self.handle_dance = threading.Timer(0, self.thread_dance)
        else:
            try:
                duration = self.dance_face_info.start_time - self.dance_start
                result = self.send_cmd(self.dance_face_info.name, self.dance_face_info.level, self.dance_face_info.speed, self.dance_face_info.freq)
                self.dance_start = self.dance_face_info.start_time
                self.handle_dance = threading.Timer(duration/1000.0, self.thread_dance)

            except Exception as e:
                rospy.logerr("iTAMP::FaceSelectorNode::thread_dance(): %s", e)
                result = False
                self.dance_ing.put(True)

        if len(self.dance_ing.queue) > 0:
            self.thread_handle(Mode.STOP, None)
            self.thread_handle(Mode.STAND_BY, None)
            self.pub_done.publish(result)

        else:
            self.handle_dance.start()

    def thread_stand_by(self):
        #print("TH________________STAND-BY")
        try:
            if self.update() is True:
                face_name, level = self.face.shortest(self.cur_zone, self.cur_p, self.cur_a)
                self.send_cmd(face_name, level, 1, 1)
            self.handle_standby = threading.Timer(self.PERIOD_STANDBY, self.thread_stand_by)
            self.handle_standby.start()
        except Exception as e:
            rospy.logerr("iTAMP::FaceSelectorNode::thread_stand_by(): %s", e)
        #print("STAND-BY---->END\n")

    def thread_communication(self, face_name, level):
        #print("TH________________SPEECH")
        try:
            if self.play_result is False:
                self.pub_done.publish(False)
                self.thread_handle(Mode.STOP, None)
                self.thread_handle(Mode.STAND_BY, None)
            elif len(self.speech_durations) == 0:
                self.pub_done.publish(True)
                self.thread_handle(Mode.STOP, None)
                self.thread_handle(Mode.STAND_BY, None)
            else:
                self.play_time = self.play_time + self.speech_durations.popleft()
                if self.play_time >= 1000:  # ms
                    self.play_result = self.send_cmd(face_name, level, 1, 1)
                    self.handle_communication = threading.Timer((self.play_time / 1000.0), self.thread_communication, args=(face_name, level,))
                    self.play_time = 0
                else:
                    if len(self.speech_durations) != 0:
                        self.handle_communication = threading.Timer(0, self.thread_communication, args=(face_name, level,))
                    else:
                        self.play_result = self.send_cmd(face_name, level, 1, 1)
                        self.handle_communication = threading.Timer(1, self.thread_communication, args=(face_name, level,))

                self.handle_communication.start()
        except Exception as e:
            rospy.logerr("iTAMP::FaceSelectorNode::thread_communication(): %s", e)


    def sub_face_start(self, msg):
        self.thread_handle(Mode.STOP, None)
        self.thread_handle(Mode.COMMUNICATION, msg)

    def sub_dance_start(self, msg):
        self.thread_handle(Mode.STOP, None)
        self.thread_handle(Mode.DANCE, msg)

    def sub_stop(self, msg):
        self.thread_handle(Mode.STOP, None)
        self.thread_handle(Mode.STAND_BY, None)
        self.pub_done.publish(True)

    def sub_reactive(self, msg):
        self.thread_handle(Mode.STOP, None)
        self.thread_handle(Mode.REACTIVE, None)
        self.pub_done.publish(True)




import time
if __name__ == '__main__':
    rospy.init_node('itamp_face_selector', anonymous=False)
    selector = FaceSelectorNode()
    rospy.spin()

