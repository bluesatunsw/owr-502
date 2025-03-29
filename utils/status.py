#!/usr/bin/python

from dataclasses import dataclass
from os import system
from time import time

@dataclass
class Motor:
    name: str
    indx: int

    mode: int = 0
    setp: float = 0

    ppos: float = 0
    pvel: float = 0
    apos: float = 0
    avel: float = 0
    
    vout: float = 0
    ista: float = 0
    isup: float = 0

    def show(self):
        print(f"{self.indx}\t{self.name}\t{['DIS','VOL','TRQ','VEL','POS'][self.mode]}\t{self.setp:.2f}\t\t{self.ppos:.2f}\t{self.pvel:.2f}\t{self.apos:.2f}\t{self.avel:.2f}\t\t{self.vout:.2f}\t{self.ista:.2f}\t{self.isup:.2f}")

motors = [
    Motor("FLD", 1),
    Motor("FLT", 2),
    Motor("FRD", 3),
    Motor("FRT", 4),
    Motor("BLD", 5),
    Motor("BLT", 6),
    Motor("BRD", 7),
    Motor("BRT", 8),
    Motor("EEA", 9),
    Motor("EEB", 10),
]
print_period = 0.1

next_print = time()

while True:
    try:
        frame = input()
        mty = frame[16:23]
        did = int(frame[23], 16)
        dat = frame[32:].replace(" ", "")

        if mty == "10C200C":
            mod = int(dat[:2], 16)
            stp = struct.unpack('!f', bytes(reversed(bytes.fromhex(dat[3:]))))[0]
            motors[did-1].mode = mod
            motors[did-1].setp = stp
        elif mty == "10C400C":
            pos = struct.unpack('!f', bytes(reversed(bytes.fromhex(dat[:8]))))[0]
            vel = struct.unpack('!f', bytes(reversed(bytes.fromhex(dat[8:]))))[0]
            motors[did-1].ppos = pos
            motors[did-1].pvel = vel
        elif mty == "10C4014":
            pos = struct.unpack('!f', bytes(reversed(bytes.fromhex(dat[:8]))))[0]
            vel = struct.unpack('!f', bytes(reversed(bytes.fromhex(dat[8:]))))[0]
            motors[did-1].apos = pos
            motors[did-1].avel = vel
        elif mty == "10C401C":
            vou = int.from_bytes(bytes.fromhex(dat[:4]), byteorder="big", signed=True)
            ist = int.from_bytes(bytes.fromhex(dat[5:8]), byteorder="big", signed=True)
            isu = int.from_bytes(bytes.fromhex(dat[8:12]), byteorder="big", signed=True)
            motors[did-1].vout = vou / 100
            motors[did-1].ista = ist / 100
            motors[did-1].isup = isu / 100
        else:
            continue
    except:
        pass
    if time() > next_print:
        system("clear")
        print("INDX\tNAME\tMODE\tSETP\t\tPPOS\tPVEL\tAPOS\tAVEL\t\tVOUT\tISTA\tISUP")
        for m in motors:
            m.show()
        next_print += print_period
