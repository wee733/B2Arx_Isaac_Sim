# shellcheck shell=bash
# Shared helpers for the B2ARX shell entry points.
#
# Source this file; it only defines functions and has no side effects. All
# helpers are `set -u` safe. Callers may set B2ARX_LOG_PREFIX before calling
# fail() to label error messages.

# fail MESSAGE...: print an error and exit with status 1.
fail() {
  printf '%s: ERROR: %s\n' "${B2ARX_LOG_PREFIX:-b2arx}" "$*" >&2
  exit 1
}

# source_setup FILE LABEL: source an upstream-generated setup file.
# Generated setup files are not `set -u` clean, so nounset is relaxed only
# while sourcing and restored afterwards.
source_setup() {
  local setup_file="$1"
  local label="$2"
  local had_nounset=false

  if [[ ! -f "$setup_file" ]]; then
    printf 'Missing %s setup: %s\n' "$label" "$setup_file" >&2
    return 1
  fi
  [[ $- == *u* ]] && had_nounset=true
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  if [[ "$had_nounset" == true ]]; then
    set -u
  fi
}

# require_package NAME: print the install prefix of a ROS package, or fail.
# Use as: prefix="$(require_package name)" || exit
require_package() {
  local package_name="$1"
  local package_prefix

  package_prefix="$(ros2 pkg prefix "$package_name" 2>/dev/null)" ||
    fail "ROS package '$package_name' is not discoverable. ${B2ARX_PACKAGE_HINT:-Source the workspace that provides it.}"
  printf '%s\n' "$package_prefix"
}

# require_packages NAME...: fail unless every ROS package is discoverable.
# Queries the package index once instead of once per package.
require_packages() {
  local package_list
  local package_name

  package_list="$(ros2 pkg list 2>/dev/null)" ||
    fail "could not list ROS packages; is the ROS environment sourced?"
  for package_name in "$@"; do
    grep -Fxq -- "$package_name" <<<"$package_list" ||
      fail "ROS package '$package_name' is not discoverable. ${B2ARX_PACKAGE_HINT:-Source the workspace that provides it.}"
  done
}

# print_dry_run WORD...: print a shell-quoted command prefixed by [DRY-RUN].
print_dry_run() {
  printf '[DRY-RUN]'
  printf ' %q' "$@"
  printf '\n'
}

# is_launch_true VALUE: launch-style boolean test (true/1/yes/on, any case).
is_launch_true() {
  case "${1,,}" in
    true | 1 | yes | on) return 0 ;;
    *) return 1 ;;
  esac
}

# launch_argument_value NAME ARG...: print the value of NAME:=value from ARGs.
# The last occurrence wins, matching ros2 launch. Returns 1 when absent.
launch_argument_value() {
  local name="$1"
  local argument
  local value=""
  local found=1
  shift

  for argument in "$@"; do
    if [[ "$argument" == "$name:="* ]]; then
      value="${argument#"$name:="}"
      found=0
    fi
  done
  [[ $found -eq 0 ]] && printf '%s\n' "$value"
  return "$found"
}
