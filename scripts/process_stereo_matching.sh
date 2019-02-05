#!/bin/bash

function usage(){
    printf "Extract all images from rosbags of a complete dataset\n\n"

	printf "Usage: $0 [-h] -c <calibration file directory> -i <input rosbags directory> -o <output directory for extracted data> -w <catkin workspace directory>\n\n"
    exit 1;
}

while getopts ":hc:i:o:w:" option; do
    case "${option}" in
        h)
            usage
            ;;
        c)
            calib_path=${OPTARG}
            ;;
        i)
            data_dir=${OPTARG}
            ;;
        o)
            output_dir=${OPTARG}
            ;;
        w)
            catkin_dir=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))
if [ -z "${calib_path}" ] || [ -z "${data_dir}" ] || [ -z "${output_dir}" ] || [ -z "${catkin_dir}" ]; then
    usage
fi

# Test if output directory exist
if [ ! -d "${output_dir}" ]; then
  mkdir $output_dir
fi

# For each rosbag of input directory
for f in $data_dir/*.bag; do
    # Get rosbag name
    filename=$(basename "$f")
    fname="${filename%.*}"

    # Run infuse_stereo_matching tool
    ${catkin_dir}/devel/lib/infuse_debug_tools/infuse_stereo_matching -n --calibration_file_path=$calib_path $output_dir/$fname $f
    # Create disparity_all_dataset.txt with all dataset disparity metadata 
    cat $output_dir/$fname/nav_disparity/disparity_all_metadata.txt >> $output_dir/disparity_all_dataset.txt
done
