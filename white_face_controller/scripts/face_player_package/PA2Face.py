import numpy as np
from enum import Enum


class EM(Enum):
    """
    Emotion 카테고리
    """
    N = "neutral"
    A = "angry"
    SU = "surprise"
    SA = "sad"
    H = "happy"


class PA2Face:
    NAME_IDX = 0
    LEVEL_IDX = 1
    zone_db = {
        EM.N.value: [('faceNormal', 1), ('faceLaugh', 1)],
        EM.H.value: [('faceThanks', 1), ('faceSatisfaction', 1), ('faceRespect', 1), ('faceRespect', 2), ('faceLove1', 5), ('faceLove1', 6), ('faceHappy', 1), ('faceHappy', 2), ('faceHappy', 3), ('faceLaugh', 2), ('faceCheering', 1), ('faceCheering', 2)],
        EM.SA.value: [('facefaceFear', 1), ('facefaceFear', 2), ('facefaceFear', 3), ('faceSadness', 1), ('faceSadness', 2), ('faceSadness', 3), ('faceSadness', 4)],
        EM.SU.value: [('faceLaugh', 4), ('faceLaugh', 3), ('faceWinkSurprise', 4), ('faceLove1', 4), ('faceLove1', 2), ('faceLove1', 1), ('faceLove2', 1), ('faceLove2', 2), ('faceLove2', 3), ('faceLove2', 4), ('faceLove2', 5), ('faceLove2', 6), ('faceConcentrate', 1), ('faceConcentrate', 2), ('faceImpressed', 2), ('faceImpressed', 1), ('faceImpressed', 3)],
        EM.A.value: [('faceFirmEmergency', 1), ('faceFirmEmergency', 2), ('faceDislike', 1), ('faceDislike', 2), ('faceDislike', 3)]

    }


    db = {
        #Neutral
        ('faceLaugh', 1): (0.2, -0.8),
        ('faceNormal', 1): (0.0, -0.2),

        #Surprise-Positive
        ('faceLaugh', 4): (0.1, 0.9),
        ('faceLaugh', 3): (0.1, 0.7),
        ('faceWinkSurprise', 4): (0.1, 0.5),
        ('faceLove1', 4): (0.0, 0.9),
        ('faceLove1', 2): (0.0, 0.7),
        ('faceLove1', 1): (0.0, 0.5),
        ('faceLove2', 1): (0.2, 0.7),
        ('faceLove2', 2): (0.2, 0.5),
        ('faceLove2', 3): (0.2, 0.3),
        ('faceLove2', 4): (0.3, 0.9),
        ('faceLove2', 5): (0.3, 0.7),
        ('faceLove2', 6): (0.3, 0.5),
        # Surprise-Negative
        ('faceConcentrate', 1): (-0.1, 0.9),
        ('faceConcentrate', 2): (-0.1, 0.7),
        ('faceImpressed', 3): (-0.1, 0.5),
        ('faceImpressed', 2): (-0.1, 0.3),
        ('faceImpressed', 1): (-0.1, 0.1),

        #Sad
        ('facefaceFear', 3): (-0.9, -0.5),
        ('facefaceFear', 2): (-0.8, -0.5),
        ('facefaceFear', 1): (-0.7, -0.2),

        ('faceSadness', 4): (-0.5, -0.1),
        ('faceSadness', 3): (-0.5, -0.3),
        ('faceSadness', 2): (-0.1, -0.3),
        ('faceSadness', 1): (-0.1, -0.5),


        #Angry
        ('faceFirmEmergency', 2): (-0.9, 0.9),
        ('faceFirmEmergency', 1): (-0.6, 0.9),
        ('faceDislike', 3): (-0.6, 0.7),
        ('faceDislike', 2): (-0.6, 0.5),
        ('faceDislike', 1): (-0.9, 0.1),

        #Happy
        ('faceThanks', 1): (0.5, 0.9),
        ('faceSatisfaction', 1): (0.5, 0.8),
        ('faceRespect', 1): (0.3, 0.9),
        ('faceRespect', 2): (0.3, 0.8),
        ('faceLove1', 5): (0.9, 0.9),
        ('faceLove1', 6): (0.9, 0.8),
        ('faceHappy', 1): (0.1, 0.1),
        ('faceHappy', 2): (0.1, 0.3),
        ('faceHappy', 3): (0.1, 0.5),
        ('faceLaugh', 2): (0.3, 0.1),
        ('faceCheering', 1): (0.3, 0.5),
        ('faceCheering', 2): (0.3, 0.7),

    }
    def shortest(self, em, cur_p, cur_a):
        face_lst = self.zone_db.get(em)
        name_lst, dist_lst = [], []
        for name in face_lst:
            p, a = self.db.get(name)
            dist = np.linalg.norm(np.array([p, a]) - np.array([cur_p, cur_a]))
            name_lst.append(name)
            dist_lst.append(dist)
        min_idx = dist_lst.index(min(dist_lst))
        return name_lst[min_idx][self.NAME_IDX], name_lst[min_idx][self.LEVEL_IDX]

if __name__ == '__main__':
    face = PA2Face()
    n, l = face.shortest('surprise', 0.5, 0.5)
    print(n, l)


