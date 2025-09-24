#!/bin/bash

EXCLUDE_DIRECTORIES="venv .venv bin"
CPPCHECK_OPTIONS="$CPPCHECK_OPTIONS --suppress=unusedStructMember"

if [ $# -gt 1 ] || [ $# -eq 1 ] && [ "$1" != "apply" ]; then
    echo "Error... Usage:"
    echo "  $ tools/check-codingrules.sh [apply]"
    exit 1
fi

apply=$1

if [ -n "$apply" ]; then
    while true; do
        read -p "Are you sure to apply corrections to all your source files [Yy/Nn] ? " yn
        case $yn in
            [Yy]* ) break;;
            [Nn]* ) exit;;
            * ) echo "Please answer yes with [Yy] or no with [Nn].";;
        esac
    done
fi

# Filter directories
for directory in $EXCLUDE_DIRECTORIES; do find_exclude="$find_exclude $sep -not -path '*/$directory/*'"; sep='-a'; done
find_exclude=$(echo $find_exclude | tr -s ' ')

####################
### clang-format ###
####################

echo "=== CLANG-FORMAT CHECK ==="

FILES_TO_CHECK=$( sh -c "find \( $find_exclude \) -a \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \)" )
for FILE in ${FILES_TO_CHECK}; do
    if [ -z "$apply" ]; then
        clang-format $FILE | diff --color -u $FILE -
        if [ $? -ne 0 ]; then
            EXIT_STATUS=1
        fi
    else
        clang-format -i $FILE
    fi
done

################
### cppcheck ###
################

echo "=== CPPCHECK CHECK ==="

FILES_TO_CHECK=$( sh -c "find \( $find_exclude \) -a \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \)" )
cppcheck --std=c++17 --enable=style,performance,portability --force --error-exitcode=2 --quiet -j 1 \
    --check-level=exhaustive \
    --template="{file}:{line}: {severity} ({id}): {message}"         \
    --inline-suppr ${DEFAULT_SUPPRESSIONS} ${CPPCHECK_OPTIONS} ${@}  \
    ${FILES_TO_CHECK}

if [ $? -ne 0 ]; then
    EXIT_STATUS=$(( $EXIT_STATUS + 1 ))
fi

exit ${EXIT_STATUS}

