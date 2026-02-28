#!/bin/bash
while read -r fname; do
    file="$(basename "$fname" ".kicad_pcb")"

    echo "Checking PCB: $file..."
    #kicad-cli sch erc --severity-all --output "out/erc-$file" "$fname"
    #kicad-cli pcb drc --severity-all --output "out/drc-$file" "$fname"

    # Check if kicad project has a .kicad_dru file accompanying it
    if [ ! -f "$(dirname "$fname")/$file.kicad_dru" ]; then 
        echo "::error file=$fname::KiCAD Design Rules File Missing!"
    fi

    # TODO report an error on library mismatch with symbol/footprint
    # TODO report an error if a common footprint is used (that has a bluesat replacement)
    # TODO report an error if a CubeMX footprint is used, should only use the symbol
    # TODO report an error if a sym-lib-table or fp-lib-table contains /home or C drive?
    # TODO report an error if a board doesn't have a custom design rules file set


    echo ""
done < <(find "$PWD" -type f -name "*.kicad_pcb")
