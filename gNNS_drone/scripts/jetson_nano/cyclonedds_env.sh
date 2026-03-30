#!/usr/bin/env bash
# ================================================================
# cyclonedds_env.sh — shared CycloneDDS URI setup for gNNS bridge
# ================================================================
# Legacy / optional. Main scripts use Fast DDS (rmw_fastrtps_cpp).
# Do not use set -u — breaks when mixed with ROS setup.bash.
set -eo pipefail

# Source after PROJECT_ROOT is set (directory containing config/).
#
#   export CYCLONE_IFACE=wlan0   # optional: pin to WiFi / Ethernet NIC
#
# Sets CYCLONEDDS_URI to a generated XML (or falls back to static config).
# ================================================================
gnns_set_cyclonedds_uri() {
    local cfg="${PROJECT_ROOT}/config/cyclonedds.xml"
    if [[ ! -f "$cfg" ]]; then
        echo "[gNNS] Warning: missing $cfg — not setting CYCLONEDDS_URI" >&2
        return 0
    fi

    local repl
    if [[ -n "${CYCLONE_IFACE:-}" ]]; then
        repl="<NetworkInterface name=\"${CYCLONE_IFACE}\" presence_required=\"true\" prefer_multicast=\"true\"/>"
    else
        repl='<NetworkInterface autodetermine="true" prefer_multicast="true"/>'
    fi

    local out
    out="$(mktemp /tmp/gnns-cyclonedds.XXXXXX.xml)"
    if ! python3 -c "
import pathlib, sys
cfg, out, repl = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), sys.argv[3]
t = cfg.read_text()
m = '<!--GNNS_CYCLONE_IFACE-->'
if m not in t:
    raise SystemExit('missing marker')
out.write_text(t.replace(m, repl))
" "$cfg" "$out" "$repl" 2>/dev/null; then
        echo "[gNNS] Warning: CycloneDDS template failed — using static file" >&2
        export CYCLONEDDS_URI="file://${cfg}"
        return 0
    fi
    export CYCLONEDDS_URI="file://${out}"
}
