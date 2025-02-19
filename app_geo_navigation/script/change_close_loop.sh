closed_loop=(
    "0 1 2 3 4 5 1 0 3 2 1 5 4"  # scientific_park_04.osm
    "6 0 3 7 8 4 0 6 7 3 4 8"    # scientific_park_04.osm
    "9 6 7 10 11 5 6 9 10 7 8 11" # scientific_park_04.osm  
    "12 9 10 13 14 11 9 12 13 10 11 14" # scientific_park_04.osm
)

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <loop_number>"
    exit 1
fi

loop_number=$1

if ! [[ "$loop_number" =~ ^[0-9]+$ ]]; then
    echo "Error: Argument is not an integer."
    exit 1
fi

if [ "$loop_number" -lt 1 ] || [ "$loop_number" -gt ${#closed_loop[@]} ]; then
    echo "Error: Argument must be between 1 and ${#closed_loop[@]}."
    exit 1
fi

selected_loop=${closed_loop[$((loop_number-1))]}

rosparam set /global_planning/closed_loop "$selected_loop"
            