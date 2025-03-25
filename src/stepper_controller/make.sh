#!/bin/sh
arduino-cli compile -b STMicroelectronics:stm32:GenF0 -e --board-options "pnum=GENERIC_F072RBTX" --build-property "build.extra_flags=\"-ffast-math\"" --build-property "build.extra_flags=\"-O3\""
