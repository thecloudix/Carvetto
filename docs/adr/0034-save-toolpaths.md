# ADR-0034: Save Toolpaths (post selection, output files, threading)

Date: 2026-11-08 · Status: proposed (T-0224). (The card named ADR 0033, which T-0130 took for the text core.)

## Context
The core posts toolpaths (ADR-0004, ADR-0011) and `carvetto-cli post` writes one file. The app needs a
dialog to choose the post and machine, write one or several files, preview the G-code and report
errors, without a second code path that could drift from the CLI.

## Decision
1. **Same code as the CLI.** `viewmodel/save_toolpaths` builds the `PostContext` exactly as
   `cmdPost` does (file name, extension, directory and path from the output path, safe Z = the highest
   of the posted toolpaths, home, material extents from the job, rotary diameter; `[DATE]`/`[TIME]`
   stay empty so output is deterministic) and calls `PostProcessor::run`. Tests compare the text with
   the CLI's file byte for byte for all shipped posts, single file and split. Inch or millimetre output
   is the post's own `UNITS`; the dialog only filters the list by it.
2. **Posts.** `posts/*.pp` shipped (found beside the executable, in `../share/carvetto/posts`, or the
   source tree in development builds) plus `<app data>/posts/*.pp`. Each is parsed to list its
   `POST_NAME`, extension, units and whether it has a TOOLCHANGE block; one that fails is listed with
   its error and cannot be chosen. Files over 1 MiB are refused. A machine (from the tool database)
   narrows the list to its `associatedPosts` and selects its `defaultPost`; with no machine all posts
   are offered. The default post matches the job's display units.
3. **Files.** Modes: one file; one per toolpath; one per tool (toolpaths of the same tool in job order,
   tools in order of first use, each file starts with its own tool: the tool change between tools is
   then the operator's file change). In one file the post's TOOLCHANGE block handles changes, and a
   post without one raises a warning. The file name is a template ({job} {toolpath} {tool} {n}); every
   substituted value and literal is mapped to `[A-Za-z0-9 ()._-]` and valid UTF-8, trimmed of dots and
   spaces, reserved Windows device names get a `_` prefix, length is capped at 100 bytes; names that
   collide (case-insensitively) get `_2`, `_3`. The extension comes from the post. Nothing outside the
   chosen folder can be named.
4. **Safety.** The post's own rules (T-0133/T-0134: no injection, finite numbers, spindle rules) apply
   and their refusals are shown. Existing files are only replaced when asked (the dialog asks once);
   all files are staged under temporary names and renamed after every one was written; staged files
   are removed on failure.
5. **Threading.** `prepareSave` copies the records (their moves are shared and immutable) and the job
   into a `SaveJob`; `generateOutput` and `writeOutput` are pure functions of their argument. The
   dialog runs them on a worker thread, at most one at a time, re-running when a setting changed
   meanwhile; results return through a queued call. The dialog keeps a `SaveSource` copy and never a
   `Document` reference across the asynchronous gap.

## Consequences
The app has no tool database or toolpath session yet (T-0221): the dialog posts every calculated
toolpath of the document, and the machine list is empty until the window is given machines
(`MainWindow::setMachines`). T-0221 should pass `ToolpathSession::enabledToolpaths()` as the ids and the
database's machines.
