#!/bin/bash

#title           :check-codingrules.sh
#description     :This script will check coding rules in all .c and .h files into mcu-firmware repository.
#author          :stephen
#date            :20181124
#version         :1
#usage           :./check-codingrules.sh
#==============================================================================

# WARNING : This script must to be in GITROOT/tools directory, it will not work if you copy it elsewhere.

# Bash code to get the real dir of the script, even if it's called from another path.
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

UNCRUSTIFY_OUTPUT_DIR="${DIR}/../../uncrustify-output"

CUR_DIR=$(pwd)
cd "${DIR}/../"

if [ -e "/usr/bin/uncrustify"]; then
    echo "uncrustify binary not found, please install it. On Ubuntu : apt install uncrustify"
    exit 1
fi

# Check *.c files
BAD_FILES=""
echo "Scan .c files ----"
C_FILES=$(find . -name '*.c')
for FILE in $C_FILES; do
    uncrustify -c "$DIR/codingrules-uncrustify-riot.cfg" --check $FILE
    if [ $? -eq 1 ]; then
        BAD_FILES+="${FILE}"$'\n'
    fi
done

# Check *.h files
echo "Scan .h files ----"
H_FILES=$(find . -name '*.h')
for FILE in $H_FILES; do
    uncrustify -c "$DIR/codingrules-uncrustify-riot.cfg" -l C --check $FILE
    if [ $? -eq 1 ]; then
        BAD_FILES+="${FILE}"$'\n'
    fi
done

# Check result of scan
if [ -z "${BAD_FILES[0]}" ]; then
    echo "All files are good ! You are good to go"
    exit 0
else
    read -n 1 -p "Some files aren't correct, do you want to automatically generate corrected file ? (y/N) " answer
    [ -z "$answer" ] && answer="N"
    if [ ${answer^^} = "Y" ]; then
        echo -e "\n"
        mkdir -p "$UNCRUSTIFY_OUTPUT_DIR"
        # Output generated file into $UNCRUSTIFY_OUTPUT_DIR. The dir will have the same directory as the mcu repository
        while read line; do
            uncrustify -c "$DIR/codingrules-uncrustify-riot.cfg" --prefix "$UNCRUSTIFY_OUTPUT_DIR" -q $line
        done <<< "$BAD_FILES"
        echo "Done !"
    else
        echo -e "\n"
        cd $CUR_DIR
        echo "That's all folks !"
        exit 0
    fi

    read -n 1 -p "Launch meld to check diff ? (y/N) " answer
    [ -z "$answer" ] && answer="N"
    if [ ${answer^^} = "Y" ]; then
        echo -e "\n"
        if [ -e /usr/bin/meld ]; then
            meld "$DIR/../" "$UNCRUSTIFY_OUTPUT_DIR"
        else
            echo "meld binary not found, install it with : apt install meld"
        fi
    fi
fi

cd $CUR_DIR
echo "That's all folks !"
exit 0
