class FindID:

    faceID_db = {1:['faceNormal',1],
                2:['faceNormal',2],
                3:['faceNormal',3],
                11:['faceLaugh',1],
                12:['faceLaugh',2],
                13:['faceLaugh',3],
                14:['faceLaugh',4],
                21:['faceHappy',1],
                22:['faceHappy',2],
                23:['faceHappy',3],
                31:['faceLove1',1],
                32:['faceLove1',2],
                33:['faceLove1',3],
                34:['faceLove1',4],
                35:['faceLove1',5],
                36:['faceLove1',6],
                41:['faceLove2',1],
                42:['faceLove2',2],
                43:['faceLove2',3],
                44:['faceLove2',4],
                45:['faceLove2',5],
                46:['faceLove2',6],
                51:['faceCheering',1],
                52:['faceCheering',2],
                53:['faceCheering',3],
                54:['faceCheering',4],
                61:['faceRespect',1],
                62:['faceRespect',2],
                71:['faceThanks',1],
                81:['faceSatisfaction',1],
                91:['faceWinkSuprise',1],
                92:['faceWinkSuprise',2],
                101:['faceImpressed',1],
                102:['faceImpressed',2],
                103:['faceImpressed',3],
                111:['faceSadness',1],
                112:['faceSadness',2],
                113:['faceSadness',3],
                114:['faceSadness',4],
                121:['faceFear',1],
                122:['faceFear',2],
                123:['faceFear',3],
                131:['faceFireEmergency',1],
                132:['faceFireEmergency',2],
                141:['faceDislike',1],
                142:['faceDislike',2],
                143:['faceDislike',3],
                151:['faceConcentrate',1],
                152:['faceConcentrate',2],
                153:['faceConcentrate',3],
                161:['facePride',1],
                162:['facePride',2],
                171:['faceRelief',1],
                172:['faceRelief',2],
                173:['faceRelief',3],
                181:['faceSleep',1],
                182:['faceSleep',2],
                191:['faceFiller',1],
                192:['faceFiller',2],
                193:['faceFiller',3],
                194:['faceFiller',4],
                201:['faceHearing',1],
                202:['faceHearing',2]
                }


    def find_id(self, facename, level):
        find_result = []
        target_values = [str(facename), level]
        for key, values in self.faceID_db.items():
            if all(value == target_values[i] for i, value in enumerate(values)):
                find_result.append(key)
        return find_result


if __name__ == '__main__':
    idprocess = FindID()
    idresult = idprocess.find_id('faceFiller', 1)
    print(idresult)