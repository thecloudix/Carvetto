# T-0232: CMake configure for CI with a source cache and a bounded retry on download failures.
# Windows twin of tools/ci/configure.sh (same behaviour; keep them in sync).
#
#   pwsh tools/ci/configure.ps1 -S . -B build -G Ninja ...      (run from the repo root)
#
# - FETCHCONTENT_SOURCE_DIR_<NAME> is pre-seeded for every source tree in $env:CARVETTO_FC_CACHE
#   (default .fc-src, restored by actions/cache): a cache hit needs no network access.
# - A configure failure whose log shows a download/network failure is re-run (3 attempts, 15 s and
#   45 s backoff). Anything else, including a SHA256 mismatch, fails at once.
# - After success, newly downloaded source trees are copied into the cache directory.
$cmakeArgs = $args
$cache = if ($env:CARVETTO_FC_CACHE) { $env:CARVETTO_FC_CACHE } else { '.fc-src' }
$build = 'build'
for ($i = 0; $i -lt $cmakeArgs.Count; $i++) {
  if ($cmakeArgs[$i] -eq '-B' -and $i + 1 -lt $cmakeArgs.Count) { $build = $cmakeArgs[$i + 1] }
  elseif ($cmakeArgs[$i] -like '-B?*') { $build = $cmakeArgs[$i].Substring(2) }
}

# T-0234: content hash of a source tree (relative path + SHA-256 of every file, sorted, hashed again).
function Get-TreeHash($dir) {
  $root = (Resolve-Path $dir).Path
  $lines = Get-ChildItem -Path $root -Recurse -File -Force | Where-Object { $_.FullName.Substring($root.Length) -notmatch '^[\\/]\.git([\\/]|$)' } |
    ForEach-Object { ($_.FullName.Substring($root.Length) -replace '\\', '/') + ' ' + (Get-FileHash -Algorithm SHA256 $_.FullName).Hash.ToLowerInvariant() } |
    Sort-Object { $_ } -CaseSensitive
  $bytes = [Text.Encoding]::UTF8.GetBytes(($lines -join "`n"))
  $sha = [Security.Cryptography.SHA256]::Create()
  return ([BitConverter]::ToString($sha.ComputeHash($bytes)) -replace '-', '').ToLowerInvariant()
}

$seed = @()
if (Test-Path $cache) {
  foreach ($d in Get-ChildItem -Path $cache -Directory -Filter '*-src') {
    $n = $d.Name.Substring(0, $d.Name.Length - 4)
    # T-0234: FETCHCONTENT_SOURCE_DIR skips CMake's hash check; verify against the manifest.
    $mf = Join-Path $cache "$($d.Name).sha256"
    $want = if (Test-Path $mf) { (Get-Content $mf -Raw).Trim() } else { '' }
    if (-not $want -or (Get-TreeHash $d.FullName) -ne $want) {
      Write-Host "configure.ps1: cached sources for $n failed manifest verification, discarding"
      Remove-Item -Recurse -Force $d.FullName
      Remove-Item -Force $mf -ErrorAction SilentlyContinue
      continue
    }
    $seed += "-DFETCHCONTENT_SOURCE_DIR_$($n.ToUpperInvariant())=$($d.FullName -replace '\\', '/')"
    Write-Host "configure.ps1: using cached sources for $n"
  }
}

$pattern = '(Each download failed|download failed|Failed to download|error: downloading|HTTP response code|status_code: *[1-9]|Could not resolve|Couldn.t connect|Connection (timed out|reset|refused)|Operation timed out|Timeout was reached|SSL connect error|TLS connect error|Recv failure|unable to access|RPC failed|early EOF|fetch-pack|Failed to connect|Temporary failure|Bad Gateway|Service Unavailable|Gateway Time-?out|git clone.*failed|Failed to clone)'
$hashPattern = '(hash mismatch|HASH mismatch|does not match expected|verifying file|expected hash|actual hash)'
$delays = @(15, 45)
$max = 3
$attempt = 1
while ($true) {
  cmake @seed @cmakeArgs 2>&1 | Tee-Object -FilePath configure.log
  if ($LASTEXITCODE -eq 0) { break }
  # T-0234: only the last error counts (a later hash mismatch must not be retried).
  $lastDl = (Select-String -Path configure.log -Pattern $pattern | Select-Object -Last 1).LineNumber
  $lastHash = (Select-String -Path configure.log -Pattern $hashPattern | Select-Object -Last 1).LineNumber
  if ($attempt -lt $max -and $lastDl -and $lastDl -gt [int]$lastHash) {
    $d = $delays[$attempt - 1]
    Write-Host "::warning title=CMake configure: download failed::attempt $attempt of $max failed on a download error, retrying in ${d}s"
    Start-Sleep -Seconds $d
    $attempt++
    continue
  }
  $tail = (Get-Content configure.log -Tail 40) -join '%0A'
  Write-Host "::error title=CMake configure failed (attempt $attempt of $max)::$tail"
  exit 1
}

New-Item -ItemType Directory -Force -Path $cache | Out-Null
foreach ($d in Get-ChildItem -Path "$build/_deps" -Directory -Filter '*-src' -ErrorAction SilentlyContinue) {
  $t = Join-Path $cache $d.Name
  if (-not (Test-Path $t)) {
    Copy-Item -Recurse -Path $d.FullName -Destination $t
    Remove-Item -Recurse -Force (Join-Path $t '.git') -ErrorAction SilentlyContinue   # smaller cache entries
    Set-Content -Path "$t.sha256" -Value (Get-TreeHash $t) -NoNewline
    Write-Host "configure.ps1: cached $($d.Name)"
  }
}
exit 0
