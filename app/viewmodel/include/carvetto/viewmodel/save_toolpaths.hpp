#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/machine.hpp"

// Save toolpaths (T-0224, ADR-0034): Qt-free logic behind the Save Toolpaths dialog.
//
//   * `discoverPosts` lists the shipped posts (posts/*.pp) and the user's own .pp files, with the
//     name, units and extension each declares; a post that does not parse is listed with its error.
//   * `prepareSave` copies everything posting needs (records, job, post text) into a `SaveJob`;
//     `generateOutput(job)` and `writeOutput(output)` are pure functions of their argument, so
//     the app runs them on a worker thread and no `Document` is read across the asynchronous gap.
//   * The text of a file is exactly what `carvetto-cli post` writes for the same post and
//     toolpaths (same PostContext, same core code); the tests compare the bytes.
//   * One file for all toolpaths, one per toolpath, or one per tool (toolpaths of the same tool
//     in job order, tools in order of first use). File names come from a template with
//     {job} {toolpath} {tool} {n}; the values are made safe as file names (no separators,
//     reserved names or control characters), names that collide get _2, _3, ...
//   * Nothing is overwritten unless asked; files are staged under temporary names and renamed in
//     place after all of them were written.
//
// Units are those of the post (the post divides by 25.4 itself).

namespace carvetto::viewmodel {

/// A post-processor the user can pick.
struct PostInfo {
    /// Display name (the post's POST_NAME, or the file name when it has none).
    std::string name;
    std::filesystem::path path;
    /// File extension without the dot (the post's FILE_EXTENSION, "nc" when none).
    std::string extension;
    post::Units units{post::Units::Millimetres};
    bool shipped{false};
    /// Non-empty when the file cannot be used (unreadable, too large, does not parse).
    std::string error;
    /// The post defines a TOOLCHANGE block.
    bool hasToolChange{false};

    bool usable() const { return error.empty(); }
};

/// Posts in `shippedDir` (sorted by file name) followed by those in `userDir` (sorted). Either
/// directory may be empty or missing. Files over 1 MiB are listed with an error.
std::vector<PostInfo> discoverPosts(const std::filesystem::path& shippedDir, const std::filesystem::path& userDir);

/// Index of the post a machine selects by default (`defaultPost` matched against the post's name,
/// file name or file stem), or nullopt.
std::optional<std::size_t> defaultPostFor(const std::vector<PostInfo>& posts, const tooling::Machine& machine);
/// Indices of the posts a machine lists as `associatedPosts` (same matching, in the machine's
/// order); all posts when it lists none.
std::vector<std::size_t> postsForMachine(const std::vector<PostInfo>& posts, const tooling::Machine& machine);
/// Indices of the posts with the given units.
std::vector<std::size_t> postsWithUnits(const std::vector<PostInfo>& posts, post::Units units);

enum class OutputMode { SingleFile, FilePerToolpath, FilePerTool };

/// The default file name template of a mode.
std::string defaultNameTemplate(OutputMode mode);

/// `text` as a file name stem: ASCII letters, digits, space and `-_.()` and any non-ASCII UTF-8
/// kept, everything else becomes `_`; leading and trailing dots and spaces are trimmed; reserved
/// Windows device names get a `_` prefix; longer than 100 bytes is cut at a character boundary.
/// Empty input gives "".
std::string sanitizeFileStem(const std::string& text);

struct SaveOptions {
    OutputMode mode{OutputMode::SingleFile};
    std::filesystem::path directory;
    /// Empty = `defaultNameTemplate(mode)`.
    std::string nameTemplate;
    /// Name of the job for {job}: usually the document's file stem.
    std::string jobName{"job"};
};

/// Everything posting needs, by value.
struct SaveJob {
    std::vector<document::ToolpathRecord> records;
    document::Job job;
    PostInfo post;
    std::string postText;
    SaveOptions options;
    /// Records that were asked for but have no computed moves (skipped).
    std::vector<std::string> notComputed;
    /// Set when the job cannot run; `generateOutput` reports it.
    std::string error;
};

/// What posting needs from the document, copied (records share their moves): the dialog keeps
/// this, never the `Document`.
struct SaveSource {
    std::vector<document::ToolpathRecord> records;
    document::Job job;
    /// Records that were asked for but have no computed moves (left out).
    std::vector<std::string> notComputed;
};

/// Copies the records `ids` (document order; empty = all) and the job. Records without moves
/// are left out and named in `notComputed`.
SaveSource makeSaveSource(const document::Document& doc, const std::vector<document::ToolpathId>& ids);

/// Reads the post file and builds the job. Errors (no toolpaths, unusable post, unreadable file)
/// are in `error`.
SaveJob prepareSave(const SaveSource& source, const PostInfo& post, const SaveOptions& options);
/// `prepareSave(makeSaveSource(doc, ids), post, options)`.
SaveJob prepareSave(const document::Document& doc, const std::vector<document::ToolpathId>& ids, const PostInfo& post,
                    const SaveOptions& options);

struct OutputFile {
    std::filesystem::path path;
    std::string text;
    /// Names of the toolpaths in this file, in order.
    std::vector<std::string> toolpaths;
    /// The path exists already.
    bool exists{false};
};

struct SaveOutput {
    bool ok{false};
    /// Why not (post errors name the line); empty when ok.
    std::string error;
    std::vector<OutputFile> files;
    /// Post warnings, "<file>: <warning>", and notes (skipped toolpaths, tool change hints).
    std::vector<std::string> warnings;
    /// Tool changes between consecutive toolpaths inside the files.
    std::size_t toolChanges{0};
};

/// Runs the post. `cancel` (optional) is checked between files. Pure: touches no document; it
/// only looks whether the target paths exist. Safe on a worker thread.
SaveOutput generateOutput(const SaveJob& job, const std::atomic<bool>* cancel = nullptr);

/// The first `maxLines` lines of the first file, with a trailing "... (N more lines)" line.
std::string previewText(const SaveOutput& output, std::size_t maxLines);

struct WriteResult {
    bool ok{false};
    std::string error;
    std::vector<std::filesystem::path> written;
};

/// Writes the files (bytes as generated, no newline conversion). Refuses existing files unless
/// `overwrite`, and a missing directory. Every file is first created under its own random
/// temporary name in the target folder, exclusively and without following links, then moved into
/// place: without `overwrite` by an operation that fails when the target exists (so a file that
/// appeared meanwhile is never replaced and the files this call created are taken away again),
/// with `overwrite` by a rename that replaces the target entry itself (a symlink is replaced, not
/// followed). On failure temporary files are removed and the error names files already replaced.
WriteResult writeOutput(const SaveOutput& output, bool overwrite);

namespace detail {
/// Test seam: called with the staged temporary files after all were written and before any is
/// moved into place. Pass an empty function to remove it.
void setCommitHook(std::function<void(const std::vector<std::filesystem::path>&)> hook);
}  // namespace detail

}  // namespace carvetto::viewmodel
