#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source scripts/env.sh" >&2
  exit 2
fi

_lio_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
_lio_bin="${FAST_LIO2_BAZEL_BIN:-$_lio_root/bazel-bin}"
_lio_core_bin="$_lio_bin/external/wheelos_core+/cyber"
_lio_cyber_path="${FAST_LIO2_CYBER_PATH:-$_lio_root/modules/fast_lio2}"

if [[ ! -f "$_lio_cyber_path/conf/cyber.pb.conf" ]]; then
  echo "Cyber configuration is missing: $_lio_cyber_path/conf/cyber.pb.conf" >&2
  return 1
fi
for _lio_tool in \
  "$_lio_core_bin/mainboard/mainboard" \
  "$_lio_core_bin/tools/cyber_recorder/cyber_recorder"; do
  if [[ ! -x "$_lio_tool" ]]; then
    echo "Core tool is missing: $_lio_tool; build //:core_tools with --config=dev." >&2
    return 1
  fi
done

export APOLLO_ROOT_DIR="$_lio_root"
export CYBER_PATH="$_lio_cyber_path"
export CYBER_IP="${CYBER_IP:-127.0.0.1}"
export CYBER_DOMAIN_ID="${CYBER_DOMAIN_ID:-80}"
for _lio_tool_dir in \
  "$_lio_core_bin/mainboard" \
  "$_lio_core_bin/tools/cyber_recorder"; do
  case ":$PATH:" in
    *":$_lio_tool_dir:"*) ;;
    *) PATH="$_lio_tool_dir:$PATH" ;;
  esac
done
export PATH

unset _lio_root _lio_bin _lio_core_bin _lio_cyber_path _lio_tool _lio_tool_dir
