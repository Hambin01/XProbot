#!/bin/bash
	 
root_dir="/home/hambin/map_stream"
deleted_files=$(find "$root_dir" -type f -name "*.cpp" -print -delete)
echo "$deleted_files"
