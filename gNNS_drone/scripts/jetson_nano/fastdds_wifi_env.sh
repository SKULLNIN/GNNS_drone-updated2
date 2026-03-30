#!/usr/bin/env bash
# Optional Fast DDS profile for WiFi + large images (source on Jetson and laptop).
# Skip with: export GNNS_SKIP_FASTDDS_WIFI=1
if [[ "${GNNS_SKIP_FASTDDS_WIFI:-0}" == "1" ]]; then
    return 0 2>/dev/null || exit 0
fi
_FDIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_GNNS_ROOT="$(cd "$_FDIR/../.." && pwd)"
_FXML="$_GNNS_ROOT/config/fastdds_wifi_images.xml"
if [[ -f "$_FXML" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_FXML"
fi
unset _FDIR _GNNS_ROOT _FXML
