#!/usr/bin/env bash
# Discover the Sawyer's IPv4 address and the host interface used to reach it.
#
# Diagnostics are written to stderr. Successful stdout is safe to evaluate:
#   eval "$(./find_robot.sh)"
#
# Optional overrides:
#   ROBOT_HOSTNAME       mDNS hostname (default: 021607CP00070.local)
#   ROBOT_IP             explicit robot IPv4 address
#   ROBOT_IP_HINT        last-known robot IPv4 address (default: 192.168.1.105)
#   HOST_IP_HINT         host address used if robot-link DHCP fails (default: 192.168.1.103)
#   ROBOT_NET_PREFIX     fallback network prefix length (default: 24)
#   ROBOT_CONNECTION    NetworkManager fallback profile name (default: Sawyer robot)

log() {
    printf '%s\n' "$*" >&2
}

is_ipv4() {
    local ip="$1" octet
    local IFS=.
    local -a parts

    read -r -a parts <<< "$ip"
    [[ ${#parts[@]} -eq 4 ]] || return 1

    for octet in "${parts[@]}"; do
        [[ "$octet" =~ ^[0-9]+$ ]] || return 1
        ((10#$octet >= 0 && 10#$octet <= 255)) || return 1
    done
}

is_link_local_ipv6() {
    [[ "${1,,}" == fe80:* ]]
}

resolve_ipv4() {
    local hostname="$1" candidate=""

    if command -v avahi-resolve >/dev/null 2>&1; then
        candidate=$(timeout 3 avahi-resolve -4 -n "$hostname" 2>/dev/null \
            | awk 'NR == 1 {print $2}') || true
        if is_ipv4 "$candidate"; then
            printf '%s\n' "$candidate"
            return 0
        fi
    fi

    candidate=$(getent ahostsv4 "$hostname" 2>/dev/null \
        | awk 'NR == 1 {print $1}') || true
    if is_ipv4 "$candidate"; then
        printf '%s\n' "$candidate"
        return 0
    fi

    if command -v dig >/dev/null 2>&1; then
        while IFS= read -r candidate; do
            if is_ipv4 "$candidate"; then
                printf '%s\n' "$candidate"
                return 0
            fi
        done < <(timeout 3 dig +short A "$hostname" 2>/dev/null || true)
    fi

    return 1
}

resolve_link_local_ipv6() {
    local hostname="$1" candidate=""

    if command -v avahi-resolve >/dev/null 2>&1; then
        candidate=$(timeout 3 avahi-resolve -6 -n "$hostname" 2>/dev/null \
            | awk 'NR == 1 {print $2}') || true
        candidate="${candidate%%%*}"
        if is_link_local_ipv6 "$candidate"; then
            printf '%s\n' "$candidate"
            return 0
        fi
    fi

    while IFS= read -r candidate; do
        candidate="${candidate%%%*}"
        if is_link_local_ipv6 "$candidate"; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done < <(getent ahostsv6 "$hostname" 2>/dev/null | awk '{print $1}' || true)

    return 1
}

candidate_interfaces() {
    local iface

    while IFS= read -r iface; do
        iface="${iface%%@*}"
        case "$iface" in
            lo|docker*|veth*|br-*|virbr*|vmnet*|tun*|tap*|tailscale*|wg*|zt*|podman*)
                continue
                ;;
        esac

        ip link show dev "$iface" 2>/dev/null | grep -q '<[^>]*UP[^>]*>' || continue
        printf '%s\n' "$iface"
    done < <(ip -o link show 2>/dev/null | awk -F': ' '{print $2}')
}

interface_ipv4() {
    local iface="$1" address

    while IFS= read -r address; do
        address="${address%%/*}"
        if is_ipv4 "$address"; then
            printf '%s\n' "$address"
            return 0
        fi
    done < <(ip -4 -o address show dev "$iface" scope global 2>/dev/null \
        | awk '{print $4}')

    return 1
}

probe_ipv4() {
    local target="$1"
    # Deliberately do not force an interface. ROS uses the normal route too;
    # accepting a forced-interface ping can produce a false positive.
    ping -4 -n -c 1 -W 1 "$target" >/dev/null 2>&1
}

probe_link_local_ipv6() {
    local iface="$1" target="$2"
    ping -6 -n -c 1 -W 1 "${target}%${iface}" >/dev/null 2>&1
}

route_to_ipv4() {
    local target="$1" route iface host_ip

    route=$(ip -4 route get "$target" 2>/dev/null) || return 1
    route="${route%%$'\n'*}"
    iface=$(awk '{for (i = 1; i <= NF; i++) if ($i == "dev") {print $(i+1); exit}}' \
        <<< "$route")
    host_ip=$(awk '{for (i = 1; i <= NF; i++) if ($i == "src") {print $(i+1); exit}}' \
        <<< "$route")

    [[ -n "$iface" ]] && is_ipv4 "$host_ip" || return 1
    printf '%s %s\n' "$iface" "$host_ip"
}

find_reachable_ipv4() {
    local target="$1" route iface host_ip

    route=$(route_to_ipv4 "$target") || return 1
    read -r iface host_ip <<< "$route"
    log "    Trying $iface ($host_ip) -> $target ..."
    if probe_ipv4 "$target"; then
        printf '%s %s\n' "$iface" "$host_ip"
        return 0
    fi

    return 1
}

find_link_local_interface() {
    local target="$1" iface

    while IFS= read -r iface; do
        log "    Checking robot identity on $iface via IPv6 ..."
        if probe_link_local_ipv6 "$iface" "$target"; then
            printf '%s\n' "$iface"
            return 0
        fi
    done < <(candidate_interfaces)

    return 1
}

run_network_command() {
    local output direct_error

    if output=$("$@" 2>&1); then
        [[ -n "$output" ]] && log "    $output"
        return 0
    fi
    direct_error="$output"

    if command -v sudo >/dev/null 2>&1 && output=$(sudo -n "$@" 2>&1); then
        [[ -n "$output" ]] && log "    $output"
        return 0
    fi

    log "  x Could not run: $*"
    [[ -n "$direct_error" ]] && log "    $direct_error"
    return 1
}

networkmanager_profile_exists() {
    local profile="$1" name

    while IFS= read -r name; do
        [[ "$name" == "$profile" ]] && return 0
    done < <(nmcli -t -f NAME connection show 2>/dev/null || true)

    return 1
}

configure_static_ipv4() {
    local iface="$1" host_ip="$2" prefix="$3"
    local profile="${ROBOT_CONNECTION:-Sawyer robot}"

    log "  DHCP supplied no IPv4 address; configuring $host_ip/$prefix on $iface ..."

    if command -v nmcli >/dev/null 2>&1; then
        if networkmanager_profile_exists "$profile"; then
            run_network_command nmcli connection modify "$profile" \
                connection.interface-name "$iface" \
                connection.autoconnect yes \
                connection.autoconnect-priority 100 \
                ipv4.method manual \
                ipv4.addresses "$host_ip/$prefix" \
                ipv4.gateway "" \
                ipv4.never-default yes \
                ipv4.may-fail no \
                ipv4.link-local enabled \
                ipv6.method link-local || return 1
        else
            run_network_command nmcli connection add \
                type ethernet \
                ifname "$iface" \
                con-name "$profile" \
                connection.autoconnect yes \
                connection.autoconnect-priority 100 \
                ipv4.method manual \
                ipv4.addresses "$host_ip/$prefix" \
                ipv4.never-default yes \
                ipv4.may-fail no \
                ipv4.link-local enabled \
                ipv6.method link-local || return 1
        fi

        run_network_command nmcli connection up "$profile" ifname "$iface" || return 1
        return 0
    fi

    log "  NetworkManager is unavailable; applying a temporary address."
    run_network_command ip link set dev "$iface" up || return 1
    run_network_command ip address replace "$host_ip/$prefix" dev "$iface"
}

collect_ipv4_candidates() {
    local hostname="$1" explicit_ip="$2" hint_ip="$3"
    local resolved_ip="" candidate seen=" "

    resolved_ip=$(resolve_ipv4 "$hostname") || true

    for candidate in "$explicit_ip" "$resolved_ip" "$hint_ip"; do
        [[ -n "$candidate" ]] || continue
        is_ipv4 "$candidate" || continue
        [[ "$seen" == *" $candidate "* ]] && continue
        printf '%s\n' "$candidate"
        seen+="$candidate "
    done
}

find_reachable_from_candidates() {
    local hostname="$1" explicit_ip="$2" hint_ip="$3"
    local candidate result

    while IFS= read -r candidate; do
        result=$(find_reachable_ipv4 "$candidate") || continue
        printf '%s %s\n' "$candidate" "$result"
        return 0
    done < <(collect_ipv4_candidates "$hostname" "$explicit_ip" "$hint_ip")

    return 1
}

find_reachable_on_interface() {
    local iface="$1" hostname="$2" explicit_ip="$3" hint_ip="$4"
    local host_ip candidate route route_iface

    while IFS= read -r candidate; do
        route=$(route_to_ipv4 "$candidate") || continue
        read -r route_iface host_ip <<< "$route"
        [[ "$route_iface" == "$iface" ]] || continue
        log "    Retrying $iface ($host_ip) -> $candidate ..."
        if probe_ipv4 "$candidate"; then
            printf '%s %s %s\n' "$candidate" "$iface" "$host_ip"
            return 0
        fi
    done < <(collect_ipv4_candidates "$hostname" "$explicit_ip" "$hint_ip")

    return 1
}

emit_discovery() {
    local robot_ip="$1" host_ip="$2" iface="$3" hostname="$4"

    printf 'ROBOT_IP=%q\n' "$robot_ip"
    printf 'HOST_IP=%q\n' "$host_ip"
    printf 'ROBOT_IFACE=%q\n' "$iface"
    printf 'ROBOT_HOSTNAME=%q\n' "$hostname"
}

discover_robot() {
    local hostname="${ROBOT_HOSTNAME:-021607CP00070.local}"
    local explicit_ip="${ROBOT_IP:-}"
    local robot_hint="${ROBOT_IP_HINT:-${ROBOT_IP:-}}"
    local host_hint="${HOST_IP_HINT:-${HOST_IP:-}}"
    local prefix="${ROBOT_NET_PREFIX:-24}"
    local result robot_ip iface host_ip ipv6 retry retries

    if [[ -n "$explicit_ip" ]] && ! is_ipv4 "$explicit_ip"; then
        log "x ROBOT_IP must be an IPv4 address, got: $explicit_ip"
        return 2
    fi
    if [[ -n "$robot_hint" ]] && ! is_ipv4 "$robot_hint"; then
        log "x ROBOT_IP_HINT must be an IPv4 address, got: $robot_hint"
        return 2
    fi
    if [[ -n "$host_hint" ]] && ! is_ipv4 "$host_hint"; then
        log "x HOST_IP_HINT must be an IPv4 address, got: $host_hint"
        return 2
    fi
    if [[ ! "$prefix" =~ ^[0-9]+$ ]] || ((prefix < 1 || prefix > 30)); then
        log "x ROBOT_NET_PREFIX must be between 1 and 30, got: $prefix"
        return 2
    fi
    if [[ -n "$robot_hint" && "$robot_hint" == "$host_hint" ]]; then
        log "x Robot and host fallback addresses must be different."
        return 2
    fi

    log "Searching for Sawyer robot $hostname ..."
    result=$(find_reachable_from_candidates "$hostname" "$explicit_ip" "$robot_hint") || true
    if [[ -n "$result" ]]; then
        read -r robot_ip iface host_ip <<< "$result"
        log "  OK Robot reachable via $iface (host: $host_ip, robot: $robot_ip)"
        emit_discovery "$robot_ip" "$host_ip" "$iface" "$hostname"
        return 0
    fi

    log "  IPv4 discovery failed; locating the physical robot link ..."
    ipv6=$(resolve_link_local_ipv6 "$hostname") || true
    if [[ -z "$ipv6" ]]; then
        log "x The robot did not publish an IPv4 or link-local IPv6 address."
        log "  Check that the controller is fully booted and Ethernet is connected."
        return 1
    fi

    iface=$(find_link_local_interface "$ipv6") || true
    if [[ -z "$iface" ]]; then
        log "x $hostname resolved to $ipv6, but no local interface can reach it."
        return 1
    fi
    log "  OK Verified Sawyer on $iface at ${ipv6}%${iface}"

    host_ip=$(interface_ipv4 "$iface") || true
    if [[ -z "$host_ip" ]]; then
        if [[ -z "$host_hint" || -z "$robot_hint" ]]; then
            log "x The robot link has no IPv4 address. Set ROBOT_IP_HINT and HOST_IP_HINT in runtime.env."
            return 1
        fi
        if [[ "${ROBOT_AUTO_CONFIGURE:-1}" != 1 ]]; then
            log "x $iface has no IPv4 address and automatic configuration is disabled."
            return 1
        fi
        configure_static_ipv4 "$iface" "$host_hint" "$prefix" || {
            log "x Could not configure the robot interface automatically."
            log "  Run once with permission to manage NetworkManager connections."
            return 1
        }
    fi

    retries="${ROBOT_DISCOVERY_RETRIES:-5}"
    [[ "$retries" =~ ^[0-9]+$ ]] || retries=5
    for ((retry = 0; retry <= retries; retry++)); do
        result=$(find_reachable_on_interface "$iface" "$hostname" "$explicit_ip" "$robot_hint") || true
        if [[ -n "$result" ]]; then
            read -r robot_ip iface host_ip <<< "$result"
            log "  OK Robot reachable via $iface (host: $host_ip, robot: $robot_ip)"
            emit_discovery "$robot_ip" "$host_ip" "$iface" "$hostname"
            return 0
        fi
        ((retry < retries)) && sleep 1
    done

    log "x Sawyer is present on $iface, but its IPv4 address is not reachable."
    log "  Tried mDNS and the known address $robot_hint from host address $host_hint."
    log "  The controller's IPv4/ROS service may need to finish booting or be restarted."
    return 1
}

main() {
    discover_robot
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -uo pipefail
    main "$@"
fi
