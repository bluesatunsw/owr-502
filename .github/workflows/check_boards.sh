#!/bin/bash
exitcode=0

while read -r fname; do
    proj_path="${fname%.kicad_pro}"
    proj_name="$(basename "$fname" ".kicad_pro")"

    echo "::group::$proj_name"

    # Check if kicad project has a .kicad_dru file accompanying it
    if [ ! -f "$proj_path.kicad_dru" ]; then 
        echo "::error file=$fname::KiCAD Design Rules File Missing!"
        exitcode=$((exitcode|1))
    fi

    # TODO report an error on library mismatch with symbol/footprint
    # TODO report an error if a common footprint is used (that has a bluesat replacement)
    # TODO report an error if a CubeMX footprint is used, should only use the symbol
    # TODO report an error if a sym-lib-table or fp-lib-table contains /home or C drive?

    echo "Checking design files: $proj_name..."

    if ! kicad-cli sch erc --severity-all --exit-code-violations --output "out/erc-$proj_name" "$proj_path.kicad_sch"; then
        echo "::error file=$proj_name::ERC Error!"
        exitcode=$((exitcode|2))
    fi

    if kicad-cli pcb drc --severity-all --exit-code-violations --output "out/drc-$proj_name" "$proj_path.kicad_pcb"; then
        echo "::error file=$proj_name::DRC Error!"
        exitcode=$((exitcode|4))
    fi


    echo "::endgroup::"
    echo ""
done < <(find "$PWD" -type f -name "*.kicad_pro")

exit $exitcode
