#!/usr/bin/env python3
import socket
import rospy
import sys
from FindID import FindID
from behavior_expression_skill_msg.srv import Face, FaceResponse


class FacePlayerNode:
    HOST = '127.0.0.1'
    PORT = 8000

    def __init__(self):
        self.connect()
        self.face_set = {'faceNormal': 3, 'faceLaugh': 4, 'faceWinkSurprise': 2, 'faceHappy': 3, 'faceLove1': 6,
                       'faceLove2': 6, 'faceCheering': 4, 'faceRespect': 2, 'faceThanks': 1, 'faceSatisfaction': 1,
                       'faceImpressed': 3, 'faceSadness': 4, 'faceFear': 3, 'faceFirmEmergency': 2, 'faceDislike': 3,
                       'faceConcentrate': 3, 'facePride': 2, 'faceRelief': 3, 'faceSleep': 2, 'faceFiller': 4, 'faceHearing': 2}
        rospy.Service('/behavior_expression/face/param', Face, self.server)

    def server(self, req):
        result = self.play(req.name, int(req.level), int(req.speed), int(req.times))
        if result is False:
            if self.connect() is True:
                result = self.play(req.name, int(req.level), int(req.speed), int(req.times))
            else:
                return FaceResponse(False)
        return FaceResponse(result)
    
    
########아래부분부터 unity 연결



    def connect(self) -> bool:
        try:
            self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_sock.connect((self.HOST, self.PORT))
        except Exception as e:
            rospy.logerr("iTAMP::FaceControllerNode::connect(): %s", e)
        rospy.sleep(0.01)
        return True

               
                

    def play(self, face: str, level: int, speed: int, times: int) -> bool:
        if face in self.face_set:
            if (level <= self.face_set[face]) and (level > 0):
                try:
                    self.face_id = self.idprocess.find_id(face, level)
                    packet = str(self.face_id) + "|" + str(speed) + "|" + str(times)
                    self.client_sock.sendall(packet.encode())
                    #rospy.sleep(1)
                    return True
                except Exception as e:
                    rospy.logerr("iTAMP::FaceControllerNode::play(): %s", e)
        return False




    # def play(self, face: str, level: int, speed: int, times: int) -> bool:
    #     if face in self.face_set:
    #         if (level <= self.face_set[face]) and (level > 0):
    #             try:
    #                 packet = face + "|" + str(level) + "|" + str(speed) + "|" + str(times)
    #                 self.client_sock.sendall(packet.encode())
    #                 #rospy.sleep(1)
    #                 return True
    #             except Exception as e:
    #                 rospy.logerr("iTAMP::FaceControllerNode::play(): %s", e)
    #     return False



    def disconnect(self) -> bool:
        try:
            self.client_sock.close()
        except AttributeError:
            return False
        return True


if __name__ == '__main__':
    rospy.init_node('itamp_white_face_player', anonymous=False)
    controller = FacePlayerNode()
    rospy.spin()
