function steer {
    yakut -i "CAN(can.media.socketcan.SocketCANMedia('can-all',64),44)" pub -N 1 \
        3000:reg.udral.physics.dynamics.rotation.Planar "{kinematics: {angular_position: $1 }}" \
        3010:reg.udral.physics.dynamics.rotation.Planar "{kinematics: {angular_position: $2 }}" \
        3020:reg.udral.physics.dynamics.rotation.Planar "{kinematics: {angular_position: $3 }}" \
        3030:reg.udral.physics.dynamics.rotation.Planar "{kinematics: {angular_position: $4 }}"
}

function drive {
    yakut -i "CAN(can.media.socketcan.SocketCANMedia('can-vesc',8),43)" pub -N 1 \
        3050:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: $1}" \
        3060:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: $2}" \
        3070:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: $3}" \
        3080:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: $4}"
}


NAME=${0,,}
NAME=${NAME#./}

VELOCITY="0.1"
if [[ "$0" =~ [[:upper:]] ]] 
then
   VELOCITY="0.3"
fi


if [[ "$NAME" =~ "x" ]] 
then
    drive 0.0 0.0 0.0 0.0
    echo "stopped!"
    exit 0
fi

# Command turn motors
case $NAME in
    w | s)
        steer 0.0 0.0 0.0 0.0 
        ;;
  
    q | e)
        steer 1.57 1.57 1.57 1.57
        ;;

    a | d)
        steer -1.04 1.06 1.06 -1.04
        ;;
esac


echo -n "Waiting for steering motion... "
echo -n "3... "
sleep 1
echo -n "2... "
sleep 1
echo -n "1... "
sleep 1
echo "0!"

# drive motors, invert right side
case $NAME in
    w | q)
        drive $VELOCITY -$VELOCITY $VELOCITY -$VELOCITY
        ;;
      
    s | e)
        drive -$VELOCITY $VELOCITY -$VELOCITY $VELOCITY
        ;;

    a)
        drive -$VELOCITY -$VELOCITY -$VELOCITY -$VELOCITY
        ;;
      
    d)
        drive $VELOCITY $VELOCITY $VELOCITY $VELOCITY
        ;;
esac
