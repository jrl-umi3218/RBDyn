# This should be sourced, not called.

# More verbose handling for 'set -e'.
#
# Show a traceback if we're using bash, otherwise just a message.
# Downloaded from: https://gist.github.com/kergoth/3885825

on_exit () {
    ret=$?
    case $ret in
        0)
            ;;
        *)
            echo >&2 "Exiting with $ret from a shell command"
            ;;
    esac
}

on_error () {
    local ret=$?
    local FRAMES=${#BASH_SOURCE[@]}

    echo >&2 "Traceback (most recent call last):"
    for ((frame=FRAMES-2; frame >= 0; frame--)); do
        local lineno=${BASH_LINENO[frame]}

        printf >&2 '  File "%s", line %d, in %s\n' "${BASH_SOURCE[frame+1]}" "$lineno" "${FUNCNAME[frame+1]}"
        sed >&2 -n "${lineno}s/^[ 	]*/    /p" "${BASH_SOURCE[frame+1]}" || true
    done
    printf >&2 "Exiting with %d\n" "$ret"
    exit $ret
}

case "$BASH_VERSION" in
    '')
        trap on_exit EXIT
        ;;
    *)
        set -o errtrace
        trap on_error ERR
        ;;
esac
