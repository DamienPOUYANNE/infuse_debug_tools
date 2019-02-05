#!/bin/bash

function usage(){
    printf "Create video with stereo images pairs of near, front and nav cameras\n\n"

	printf "Usage: $0 [-h] -i <input images directory> -o <output directory for video>\n\n"
    printf "* This script needs imagemagick and ffmpeg --> sudo apt install -y imagemagick ffmpeg\n"
    printf "* The input directory must have the same organisation as defined by the infuse-data-extractor tool of the infuse-debug-tools library.\n\n"
    exit 1;
}

while getopts ":hi:o:" option; do
    case "${option}" in
        h)
            usage
            ;;
        i)
            root_data_dir=${OPTARG}
            ;;
        o)
            output_dir=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))
if [ -z "${root_data_dir}" ] || [ -z "${output_dir}" ]; then
    usage
fi

# Test if output directory exist
if [ ! -d "${output_dir}" ]; then
  mkdir ${output_dir}
fi

# Init variables
declare -a sensorNames=("nav_cam" "front_cam" "rear_cam")
index=0
nb_img=0
init=0

# Create allPairs directory
mkdir -p ${root_data_dir}/allPairs/

# For each bag file directory
for dir in ${root_data_dir}/my_bag*/; do

    echo "Scanning ${dir} directory ..."
    
    # For each sensor
    for sensor in "${sensorNames[@]}"
    do
        # Create output directories
        mkdir -p ${dir}/${sensor}/stereoPairs/
        mkdir -p ${dir}/${sensor}/left/pngs/
        mkdir -p ${dir}/${sensor}/right/pngs/

        nb_img=0
        index=${init}

        # For each left image
        for f in ${dir}/${sensor}/left/data/*.png; do
            # Get rosbag name
            filename=$(basename "$f")
            fname="${filename%.*}"

            # Convert index to '05d' format
            printf -v index_display "%05d" ${index}

            # Add image id on the left corner of each left image, set resolution to 640x480 and convert it to png
            convert -geometry 640x480 ${dir}/${sensor}/left/data/${filename} \
		    -font Helvetica -pointsize 36 -fill blue -draw "text 50,50 '${index_display}'" \
		    ${dir}/${sensor}/left/pngs/${fname}.png

            # Set resolution of right image to 640x480 and convert it to png 
            convert -geometry 640x480 ${dir}/${sensor}/right/data/${filename} \
		    ${dir}/${sensor}/right/pngs/${fname}.png

            # Create pair image with the left and right ones 
            convert ${dir}/${sensor}/left/pngs/${fname}.png \
                    ${dir}/${sensor}/right/pngs/${fname}.png \
                    +append ${dir}/${sensor}/stereoPairs/${fname}.png

            nb_img=$((nb_img+1))
            index=$((index+1))
        done
    done

    index=${init}

    for fPairs in ${dir}/nav_cam/stereoPairs/*.png; do
        # Get rosbag name
        filename=$(basename "${fPairs}")

        # Convert index to '05d' format
        printf -v index_display "%05d" ${index}

        # Create image from a concatenation of the rear, front and nav cam stereo pairs
        convert ${dir}/nav_cam/stereoPairs/${filename} \
                ${dir}/front_cam/stereoPairs/${filename} \
                ${dir}/rear_cam/stereoPairs/${filename} \
                -append ${root_data_dir}/allPairs/${index_display}.png
        
        index=$((index+1))
    done

    init=$((init+${nb_img}))

done

# Convert the images sequence into video file
ffmpeg -framerate 15 -pattern_type glob -i ${root_data_dir}/allPairs/'*.png' -c:v ffv1 ${output_dir}/allPairs-video.avi