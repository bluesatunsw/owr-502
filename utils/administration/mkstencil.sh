#!/bin/bash
while read -r fname; do
    kicad-cli pcb export svg --mode-multi --layers F.Paste,B.Paste --fit-page-to-board --exclude-drawing-sheet --drill-shape-opt 0 -o out "$fname"

    # Remove empty svg's
    file=$(basename "$fname" .kicad_pcb)

    if ! (cat "out/$file-F_Paste.svg" | grep "path" > /dev/null); then
        echo "Removing empty file out/$file-F_Paste.svg"
        rm "out/$file-F_Paste.svg"
    fi
    
    if ! (cat "out/$file-B_Paste.svg" | grep "path" > /dev/null); then
        echo "Removing empty file out/$file-B_Paste.svg"
        rm "out/$file-B_Paste.svg"
    fi
    
    echo ""
done < <(find "$PWD" -type f -name "*.kicad_pcb")
