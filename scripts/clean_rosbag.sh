#!/bin/bash

function usage(){
    printf "Dataset rosbags cleaner\n\n"

	printf "Usage: $0 [-h] -c <calibration file directory> -t <topic to clean> -o <output directory for extracted data>\n\n"
    exit 1;
}

while getopts ":hc:t:o:" option; do
    case "${option}" in
        h)
            usage
            ;;
        c)
            calib_path=${OPTARG}
            ;;
        t)
            topic=${OPTARG}
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
if [ -z "${calib_path}" ] || [ -z "${topic}" ] || [ -z "${output_dir}" ]; then
    usage
fi

# Test if output directory exist
if [ ! -d "${output_dir}" ]; then
  mkdir $output_dir
fi

roslaunch infuse_debug_tools debug_dataset_cleaning.launch output_path:=$output_dir topics_to_connect:=$topic calibration_file_path:=$calib_path