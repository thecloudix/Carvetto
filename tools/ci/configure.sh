#!/usr/bin/env bash
# T-0232: CMake configure for CI with a source cache and a bounded retry on download failures.
#
#   tools/ci/configure.sh <cmake configure args...>      (run from the repo root)
#
# - FETCHCONTENT_SOURCE_DIR_<NAME> is pre-seeded for every source tree found in $CARVETTO_FC_CACHE
#   (default .fc-src, restored by actions/cache), so a cache hit needs no network access at all.
# - If configure fails and its log shows a download/network failure, it is re-run (up to 3 attempts,
#   15 s and 45 s backoff). Any other failure is final at once. SHA256/URL pins are never relaxed:
#   a downloaded file with the wrong hash is a hard failure (not matched by the retry pattern).
# - After a successful configure, newly downloaded source trees are copied into the cache directory.
# Keep in sync with tools/ci/configure.ps1 (Windows).
set -uo pipefail

cache=${CARVETTO_FC_CACHE:-.fc-src}
build=build
args=("$@")
# the build directory is the value after -B, to find build/_deps
for i in "${!args[@]}"; do
  if [ "${args[$i]}" = "-B" ]; then build=${args[$((i + 1))]}; fi
done

seed=()
if [ -d "$cache" ]; then
  for d in "$cache"/*-src; do
    [ -d "$d" ] || continue
    n=$(basename "$d" -src)
    up=$(printf '%s' "$n" | tr '[:lower:]' '[:upper:]')
    seed+=("-DFETCHCONTENT_SOURCE_DIR_${up}=$(cd "$d" && pwd)")
    echo "configure.sh: using cached sources for $n"
  done
fi

# Failure texts of CMake's download step, curl and git (HTTP 5xx, DNS, TLS, timeouts, resets).
pattern='(Each download failed|download failed|Failed to download|error: downloading|HTTP response code|status_code: *[1-9]|Could not resolve|Couldn.t connect|Connection (timed out|reset|refused)|Operation timed out|Timeout was reached|SSL connect error|TLS connect error|Recv failure|unable to access|RPC failed|early EOF|fetch-pack|Failed to connect|Temporary failure|Bad Gateway|Service Unavailable|Gateway Time-?out|git clone.*failed|Failed to clone)'

delays=(15 45)
attempt=1
max=3
while :; do
  cmake "${seed[@]}" "${args[@]}" 2>&1 | tee configure.log
  rc=${PIPESTATUS[0]}
  [ "$rc" -eq 0 ] && break
  if [ "$attempt" -lt "$max" ] && grep -Eqi "$pattern" configure.log; then
    d=${delays[$((attempt - 1))]}
    echo "::warning title=CMake configure: download failed::attempt $attempt of $max failed on a download error, retrying in ${d}s"
    sleep "${CARVETTO_RETRY_SLEEP:-$d}"
    attempt=$((attempt + 1))
    continue
  fi
  echo "::error title=CMake configure failed (attempt $attempt of $max)::$(tail -30 configure.log | tr '\n' '|')"
  exit 1
done

mkdir -p "$cache"
for d in "$build"/_deps/*-src; do
  [ -d "$d" ] || continue
  t="$cache/$(basename "$d")"
  if [ ! -d "$t" ]; then
    cp -a "$d" "$t" && echo "configure.sh: cached $(basename "$d")"
  fi
done
exit 0
