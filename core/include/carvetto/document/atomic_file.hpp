#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>

namespace carvetto::document {

/// Where `writeFileAtomic` should fail on purpose (tests only; the default never fails early).
enum class AtomicFailPoint {
    None,
    AfterTempWrite,  ///< The temporary file is complete, then the write "fails" before any rename.
    BeforeRename,    ///< The backup (if any) is made, then the final rename "fails".
};

struct AtomicWriteOptions {
    /// Keep the previous content of an existing target as "<path>.bak" (replaced on each save).
    bool keepBackup{false};
    /// Test hook: inject a failure at this step. The target must be left as it was.
    AtomicFailPoint failAt{AtomicFailPoint::None};
};

/// Writes `bytes` to `path` crash-safely (T-0357): the bytes go to "<path>.tmp", which is flushed and
/// fsync'ed (FlushFileBuffers on Windows), the previous file is copied to "<path>.bak" when asked, and
/// only then is the temporary file renamed over `path`. At every moment `path` holds either the old
/// complete file or the new complete one. On failure the temporary file is removed, `path` is untouched
/// and false is returned with `error` set to a sentence naming the step.
bool writeFileAtomic(const std::filesystem::path& path, std::span<const std::uint8_t> bytes, std::string& error,
                     const AtomicWriteOptions& options = {});

}  // namespace carvetto::document
