#!/bin/bash
while read line; do
	date +%T
	echo "text from input: $line"
done < "$1"
