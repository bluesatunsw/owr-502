yakut pub -N 1 \
    3060:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: 0.3}"
sleep 3
yakut pub -N 1 \
    3060:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: 0.0}"
