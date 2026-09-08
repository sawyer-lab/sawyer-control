#!/usr/bin/env bash
set -Eeuo pipefail

if [[ -n "${BRIO_DEVICE:-}" ]]; then
    [[ -c "$BRIO_DEVICE" ]] || {
        printf 'BRIO_DEVICE is not a video device: %s\n' "$BRIO_DEVICE" >&2
        exit 1
    }
    readlink -f "$BRIO_DEVICE"
    exit 0
fi

for link in /dev/v4l/by-id/*[Ll]ogitech*[Bb][Rr][Ii][Oo]*-video-index0; do
    [[ -e "$link" ]] || continue
    readlink -f "$link"
    exit 0
done

exit 1
