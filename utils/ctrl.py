#!/usr/bin/python

from os import system
from sys import exit
from struct import pack
from math import atan2, pi, hypot
from time import sleep
from signal import signal, SIGINT
from argparse import ArgumentParser

test = False
turn_ticks = 15

drive_ratio = 14
turn_ratio = 60
bucket_ratio = 1600

def command(dev: int, mod: int, val: float):
    dat = "".join("{:02X}".format(x) for x in reversed(bytearray(pack("f", val))))
    cmd = f"cansend can0 10C200C{dev:01X}#{mod:02X}{dat}"
    if test:
        print(cmd)
    else:
        for i in range(3):
            system(cmd)

def stop_drive():
    for d in range(8):
        command(d, 0, 0.0)

track_width = 0.6
track_length = 1.0
# FL, FR, BL, BR
module_offsets = [
    (track_length/2, track_width/2),
    (track_length/2, -track_width/2),
    (-track_length/2, track_width/2),
    (-track_length/2, -track_width/2)
]

def drive(vx: float, vy: float, vr: float, active: bool = False):
    stop_drive()
    # drive velocity and turn angle for each module
    setpoints = []

    for m in module_offsets:
        mvx = vx - vr*m[0]
        mvy = vy - vr*p[1]
        ang = atan2(mvy, mvx)
        spd = hypot(mvy, mvx)

        if (abs(ang) < pi/2):
            setpoints += [spd, ang]
        else:
            setpoints += [-spd, atan2(-mvy, -mvx)]

    # set turn angles
    for j in range(turn_ticks):
        for i in range(4):
            command(i*2 + 2, 4, setpoints[i][1] * turn_ratio)
        sleep(0.1)
    if not active:
        stop_drive()
    # run wheels
    while True:
        for i in range(4):
            command(i*2 + 1, 3, setpoints[i][0] * drive_ratio)
        sleep(0.1)

def bucket(a: float, b: float):
    # Set the position of stepper A and B
    command(9, 4, a*bucket_ratio)
    command(10, 4, b*bucket_ratio)

def exit_handler(sig, frame):
    stop_drive()
    exit(0)

signal(SIGINT, exit_handler)

parser = ArgumentParser(prog="Sussy Controller", description="Anta baka?")
parser.add_argument("--drive", type=float, nargs=3, default=[0,0,0], help="Drive with forward, left and turn velocity, runs forever")
parser.add_argument("--bucket", type=float, nargs=2, default=[0,0], help="Sets the positions of the stepper motors on the bucket")
args = parser.parse_args()

do_drive = args.drive[0] != 0 or args.drive[1] != 0 or args.drive[2] != 0
do_bucket = args.bucket[0] != 0 or args.bucket[1]!= 0

if not do_drive and not do_bucket:
    parser.print_help()
    stop_drive()
    exit(0)

if do_bucket:
    bucket(*args.bucket)
if do_drive:
    drive(*args.drive)

