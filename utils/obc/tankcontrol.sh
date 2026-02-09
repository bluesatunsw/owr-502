print_usage () {
    echo "Usage: $0 <left_speed> <right_speed> [<duration>]"
    echo "Sets speed on left and right sides of the rover as proportions of 1.0 or -1.0 (e.g. 0.06, -0.741)."
    echo "Speeds of 0.0 stop the whole side of the rover."
    echo "Optionally sets the duration to pulse for, in seconds."
}

if [[ "$#" -lt 2 || "$#" -gt 3 ]]; then
    print_usage
    exit 1
fi

printf -v LEFTSTR "{value: $1}"
printf -v RIGHTSTR "{value: $2}"

yakut pub -N 1 \
    3050:reg.udral.service.actuator.common.sp.Scalar.0.1 "$LEFTSTR" \
    3060:reg.udral.service.actuator.common.sp.Scalar.0.1 "$RIGHTSTR" \
    3070:reg.udral.service.actuator.common.sp.Scalar.0.1 "$LEFTSTR" \
    3080:reg.udral.service.actuator.common.sp.Scalar.0.1 "$RIGHTSTR"

if [[ -n "$3" ]]; then
    sleep $3
    yakut pub -N 1 \
        3050:reg.udral.service.actuator.common.sp.Scalar.0.1 '{value: 0.0}' \
        3060:reg.udral.service.actuator.common.sp.Scalar.0.1 '{value: 0.0}' \
        3070:reg.udral.service.actuator.common.sp.Scalar.0.1 '{value: 0.0}' \
        3080:reg.udral.service.actuator.common.sp.Scalar.0.1 '{value: 0.0}'
fi
