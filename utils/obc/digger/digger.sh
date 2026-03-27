#!/bin/bash

SHOULDER_ADDR=2000
BUCKET_ADDR=2010
PAVER_ADDR=2020

SLOW_VELOCITY=0.2
FAST_VELOCITY=1.0
PAVER_SLOW=0.4

function linac {
    echo "linac $1 $2"
    yakut -i "CAN(can.media.socketcan.SocketCANMedia('can-all', 64),50)" \
	pub --count 2 $1:reg.udral.service.actuator.common.sp.Scalar.0.1\
       "{value: $2}"	
}

echo $0

NAME=${0#./}

echo $NAME

case $NAME in
    s)
	echo "lowercase"
        linac $SHOULDER_ADDR $SLOW_VELOCITY
	;;
    S)
	echo "uppercase"
        linac $SHOULDER_ADDR $FAST_VELOCITY
	;;
    b)
        linac $BUCKET_ADDR $SLOW_VELOCITY
	;;
    B)
        linac $BUCKET_ADDR $FAST_VELOCITY
	;;
    p)
        linac $PAVER_ADDR $PAVER_SLOW
	;;
    P)
        linac $PAVER_ADDR $FAST_VELOCITY
	;;
    sr)
        linac $SHOULDER_ADDR -$SLOW_VELOCITY
	;;
    SR)
        linac $SHOULDER_ADDR -$FAST_VELOCITY
	;;
    br)
        linac $BUCKET_ADDR -$SLOW_VELOCITY
	;;
    BR)
        linac $BUCKET_ADDR -$FAST_VELOCITY
	;;
    pr)
        linac $PAVER_ADDR -$PAVER_SLOW
	;;
    PR)
	linac $PAVER_ADDR -$FAST_VELOCITY
	;;
    X | x)
        linac $SHOULDER_ADDR 0 &
        linac $BUCKET_ADDR  0 &
	linac $PAVER_ADDR 0
	;;
    xs)
        linac $SHOULDER_ADDR 0
	;;
    xb)
        linac $BUCKET_ADDR 0
	;;
    xp)
        linac $PAVER_ADDR 0
	;;
esac
