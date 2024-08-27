import numpy as np

class LabelDataConverter:
    """Convert .label binary data to instance id and rgb"""
    
    def __init__(self, labelscan):

        self.convertdata(labelscan)

    def convertdata(self, labelscan):
        
        self.semantic_id = []
        self.rgb_id = []

        for counting in range(len(labelscan)):
            sem_id = int(labelscan[counting]) & 0xFFFF
            rgb = self.get_rgb(sem_id)

            #print(sem_id)
            #print(hex(rgb))

            self.semantic_id.append(sem_id)
            self.rgb_id.append(rgb)

    def get_rgb(self, sem_id):
        RGB_id = 0
        if sem_id==0:
            RGB_id = 0x000000
        elif sem_id==1:
            RGB_id = 0xff0000
        elif sem_id==10:
            RGB_id = 0x6496f5
        elif sem_id==11:
            RGB_id = 0x64e6f5
        elif sem_id==13:
            RGB_id = 0x6450fa
        elif sem_id==15:
            RGB_id = 0x1e3c96
        elif sem_id==16:
            RGB_id = 0x0000ff
        elif sem_id==18:
            RGB_id = 0x501eb4
        elif sem_id==20:
            RGB_id = 0x0000ff
        elif sem_id==30:
            RGB_id = 0xff1e1e
        elif sem_id==31:
            RGB_id = 0xff28c8
        elif sem_id==32:
            RGB_id = 0x961e5a
        elif sem_id==40:
            RGB_id = 0xff00ff
        elif sem_id==44:
            RGB_id = 0xff96ff
        elif sem_id==48:
            RGB_id = 0x4b004b
        elif sem_id==49:
            RGB_id = 0xaf004b
        elif sem_id==50:
            RGB_id = 0xffc800
        elif sem_id==51:
            RGB_id = 0xff7832
        elif sem_id==52:
            RGB_id = 0xff9600
        elif sem_id==60:
            RGB_id = 0x96ffaa
        elif sem_id==70:
            RGB_id = 0x00af00
        elif sem_id==71:
            RGB_id = 0x873c00
        elif sem_id==72:
            RGB_id = 0x96f050
        elif sem_id==80:
            RGB_id = 0xfff096
        elif sem_id==81:
            RGB_id = 0xff0000
        elif sem_id==99:
            RGB_id = 0x32ffff
        elif sem_id==252:
            RGB_id = 0x6496f5
        elif sem_id==253:
            RGB_id = 0xff28c8
        elif sem_id==254:
            RGB_id = 0xff1e1e
        elif sem_id==255:
            RGB_id = 0x961e5a
        elif sem_id==256:
            RGB_id = 0x0000ff
        elif sem_id==257:
            RGB_id = 0x6450fa
        elif sem_id==258:
            RGB_id = 0x501eb4
        elif sem_id==259:
            RGB_id = 0x0000ff
        else:
            RGB_id = 0x000000

        return RGB_id
