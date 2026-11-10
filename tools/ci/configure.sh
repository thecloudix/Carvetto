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
  case "${args[$i]}" in -B?*) build=${args[$i]#-B} ;; esac
done

# T-0234: content hash of a source tree (every file path + SHA-256, sorted, then hashed again).
tree_hash() {
  (cd "$1" && find . -path ./.git -prune -o \( -type f -o -type l \) -print | LC_ALL=C sort |
    while IFS= read -r f; do
      if [ -L "$f" ]; then printf 'L %s %s\n' "$f" "$(readlink "$f")"; else printf 'F %s %s\n' "$f" "$(sha256sum < "$f" | cut -d' ' -f1)"; fi
    done | sha256sum | cut -d' ' -f1)
}

seed=()
if [ -d "$cache" ]; then
  for d in "$cache"/*-src; do
    [ -d "$d" ] || continue
    n=$(basename "$d" -src)
    # T-0234: FETCHCONTENT_SOURCE_DIR skips CMake's hash check, so re-verify against the manifest
    # written when the tree was first downloaded (and verified) by CMake. Mismatch -> not used.
    want=$(cat "$cache/$n-src.sha256" 2>/dev/null || true)
    if [ -z "$want" ] || [ "$(tree_hash "$d")" != "$want" ]; then
      echo "configure.sh: cached sources for $n failed manifest verification, discarding"
      rm -rf "$d" "$cache/$n-src.sha256"
      continue
    fi
    up=$(printf '%s' "$n" | tr '[:lower:]' '[:upper:]')
    seed+=("-DFETCHCONTENT_SOURCE_DIR_${up}=$(cd "$d" && pwd)")
    echo "configure.sh: using cached sources for $n"
  done
fi

# Failure texts of CMake's download step, curl and git (HTTP 5xx, DNS, TLS, timeouts, resets).
pattern='(Each download failed|download failed|Failed to download|error: downloading|HTTP response code|status_code: *[1-9]|Could not resolve|Couldn.t connect|Connection (timed out|reset|refused)|Operation timed out|Timeout was reached|SSL connect error|TLS connect error|Recv failure|unable to access|RPC failed|early EOF|fetch-pack|Failed to connect|Temporary failure|Bad Gateway|Service Unavailable|Gateway Time-?out|git clone.*failed|Failed to clone)'

hashpattern='(hash mismatch|HASH mismatch|does not match expected|verifying file|expected hash|actual hash)'
delays=(15 45)
attempt=1
max=3
while :; do
  cmake "${seed[@]}" "${args[@]}" 2>&1 | tee configure.log
  rc=${PIPESTATUS[0]}
  [ "$rc" -eq 0 ] && break
  # T-0234: only the LAST error counts: retry iff the last download-failure line comes after the
  # last hash/verification-failure line (a later SHA mismatch must not be retried).
  last_dl=$(grep -Eni "$pattern" configure.log | tail -1 | cut -d: -f1)
  last_hash=$(grep -Eni "$hashpattern" configure.log | tail -1 | cut -d: -f1)
  if [ "$attempt" -lt "$max" ] && [ -n "$last_dl" ] && [ "$last_dl" -gt "${last_hash:-0}" ]; then
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
    # T-0234: store without .git (smaller cache entries) plus a content-hash manifest.
    if cp -a "$d" "$t" && rm -rf "$t/.git"; then
      tree_hash "$t" > "$t.sha256" && echo "configure.sh: cached $(basename "$d")"
    else
      rm -rf "$t" "$t.sha256"
    fi
  fi
done
exit 0
