closed_loop=(
    "0 1 24 1" # scientific_park_04.osm   1
    "24 25 26" # scientific_park_04.osm   2
    "26 27 24" # scientific_park_04.osm  3
    "1 0 26 0" # scientific_park_04.osm    4
    "27 24 27 24 1" # scientific_park_04.osm 5
    "25 26 27" # scientific_park_04.osm     6
    "25 26 25 26 0" # scientific_park_04.osm   7 
    "0 25 24" # scientific_park_04.osm   8
    "1 27 26" # scientific_park_04.osm   9
    "1 0 25 0" # scientific_park_04.osm   10
    "6 7 8 21 22 19" # scientific_park_04.osm  11
    "19 22 21 8 7 6" # scientific_park_04.osm    12
    "8 21 22" # scientific_park_04.osm 13
    "8 21 8" # scientific_park_04.osm     14
    "8 7 6" # scientific_park_04.osm   15
    "22 21 22 19" # scientific_park_04.osm   16
    "29 28 29" # scientific_park_04.osm   17
    "28" # scientific_park_04.osm   18
    "29" # scientific_park_04.osm  19
    "28 29" # scientific_park_04.osm    20
    "28 29 28" # scientific_park_04.osm 21
    "28 29" # scientific_park_04.osm     22
    "15 16" # scientific_park_04.osm   23 
    "16 15" # scientific_park_04.osm   24
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
            