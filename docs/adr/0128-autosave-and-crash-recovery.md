# ADR-0128: Autosave, crash recovery and crash-safe saves

Status: accepted (T-0357)

- Saves go through `document::writeFileAtomic`: bytes to `<file>.tmp`, `fflush` + `fsync` (`_commit` on Windows),
  the previous file copied to `<file>.bak` (File > Save only; one generation), then an atomic `rename` over the
  target. At every instant the target is the old or the new complete file; any failure removes the temporary
  and leaves the target alone. `saveCarv(path)` uses the same writer without a backup; DXF export too.
- Autosave never touches the user's file: `viewmodel::RecoveryStore` keeps one snapshot (`autosave.carv` plus
  `autosave.meta` with the original path) in `<AppLocalData>/recovery`. The Options preferences
  `editing/autosaveEnabled` / `autosaveMinutes` drive a `QTimer`; a tick writes a snapshot while the document
  is dirty and removes it when it is clean. A clean exit (after the save/discard prompt) or switching autosave
  off deletes it, so a snapshot found at start-up means the last session was killed.
- At start-up the app asks "Recover unsaved work on X?". Recovery loads the snapshot as the document under its
  original path, marked unsaved (the original file is not overwritten until the user saves). A damaged
  snapshot is reported and deleted so start-up is never blocked.
- One slot, not one per process: two simultaneous instances share it (last write wins). Accepted for 0.1.
- Caps: `FileSession::open` refuses files over `kMaxOpenFileBytes` (1.5 GiB) before reading; the loader's own
  ADR-0013 limits and `bad_alloc` handling already turn oversized or hostile content into a `CarvError`.
