#!/bin/bash
# NOTE: uses Bash 4

board_ids=("bluepill" "nucleo-g474" "g431" "f405")

if [[ -z "$1" ]]
then
	echo "Usage: $0 <board-ID>"
	echo "Available board-IDs:"

	for id in ${board_ids[@]}
	do
		echo "- $id"
	done

	exit 1
fi

if [[ "$1" == "bluepill" ]]
then
	cargo run --release --target thumbv7m-none-eabi --features stm32f103
elif [[ "$1" == "g431" ]]
then
	cargo run --release --target thumbv7em-none-eabihf --features stm32g431
elif [[ "$1" == "nucleo-g474" ]]
then
	cargo run --release --target thumbv7em-none-eabihf --features stm32g474
elif [[ "$1" == "f405" ]]
then
	cargo run --release --target thumbv7em-none-eabihf --features stm32f405
fi
