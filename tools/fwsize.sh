#!/bin/bash

usage() {
	BINNAME=$(basename $0)
	echo "${BINNAME} computes MCU memory footprint and show bargraphs summary"
	echo "usage:"
	echo "	${BINNAME} <GNU size> <ELF file> <Flash size in bytes> <RAM size in bytes>"
}

# Following function grabbed from StackOverflow:
#   https://stackoverflow.com/a/52581824/1708389
PROGRESS_BAR_WIDTH=50  # progress bar length in characters

draw_progress_bar() {
	# Arguments: current value, max value, unit of measurement (optional)
	local __value=$1
	local __max=$2
	local __unit=${3:-""}  # if unit is not supplied, do not display it

	# Calculate percentage
	if (( $__max < 1 )); then __max=1; fi  # anti zero division protection
	local __percentage=$(( 100 - ($__max*100 - $__value*100) / $__max ))

	# Rescale the bar according to the progress bar width
	local __num_bar=$(( $__percentage * $PROGRESS_BAR_WIDTH / 100 ))

	# Draw progress bar
	printf "["
	for b in $(seq 1 $__num_bar); do printf "#"; done
	for s in $(seq 1 $(( $PROGRESS_BAR_WIDTH - $__num_bar ))); do printf " "; done
	printf "] $__percentage%% ($__value / $__max $__unit)\r"
}

#
# Script entry point
#
if [ $# -lt 4 ]; then
	usage
	exit 1
fi

# Script arguments
SIZE=${1}
ELFFILE=${2}
FLASH_SIZE=${3}
RAM_SIZE=${4}

# Call GNU size and parse outputs fields
SIZE_FIELDS=$(${SIZE} --format=berkley ${ELFFILE} | tail -n +2 | tr -s " \t")
TEXT=$(echo ${SIZE_FIELDS} | cut -f 1 -d ' ')
DATA=$(echo ${SIZE_FIELDS} | cut -f 2 -d ' ')
BSS=$(echo ${SIZE_FIELDS} | cut -f 3 -d ' ')

FLASH_USAGE=$((${TEXT} + ${DATA}))
RAM_USAGE=$((${DATA} + ${BSS}))

# Draw progress bars
draw_progress_bar ${FLASH_USAGE} ${FLASH_SIZE} "Flash" ; printf "\n"
draw_progress_bar ${RAM_USAGE} ${RAM_SIZE} "RAM" ; printf "\n"

exit 0
