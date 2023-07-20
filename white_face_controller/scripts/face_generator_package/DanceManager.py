import os
from xml.etree.ElementTree import parse


class FaceInfo(object):
    def __init__(self, name, level, speed, start_time, freq):
        self.name, self.level, self.speed, self.start_time, self.freq = name, level, speed, start_time, freq


class DanceManager(object):

    def __init__(self):
        pass

    def file_load(self, path):
        try:
            #f_lst = os.listdir(self.db_path)
            #f_name = ''.join(title.split()) + DanceManager.FILE_EXTENSION
            file = parse(path).getroot()
            self.face_iter = self.__get_face__(file)
        except Exception as e:
            print("iTAMP::white_face_controller::DanceManager::file_load()-> {}".format(e))
            return False
        return True

    def get_face(self):
        try:
            return next(self.face_iter)
        except StopIteration:
            return True
        except Exception as e:
            print("iTAMP::white_face_controller::DanceManager::get_face()-> {}".format(e))
            return False

    def __get_face__(self, file):
        for item in file.iter("face"):
            yield FaceInfo(item.findtext("name"), int(item.findtext("level")), int(item.findtext("speed")),
                           int(item.findtext("start_time")), int(item.findtext("freq")))



if __name__ == '__main__':
    dance = DanceManager("/home/kist/rosjava_itamp/src/white_face_controller/dance_db/")
    re = dance.file_load("test")
    print(re)

    while True:
        info = dance.get_face()
        if (info == True) or (info == False):
            print(info)
            print("DONE!")
            break
        print(info.name)

    print("=================")
    dance.file_load("test")
    while True:
        info = dance.get_face()
        if (info == True) or (info == False):
            print(info)
            print("DONE!")
            break
        print(info.name)

