# ADR-0051: Bitmap pixel store in `.carv` (T-0273)

Date: 2026-11-10 · Status: proposed (T-0273, extends ADR-0003 and ADR-0013; follows the embedded-font design of ADR-0033 A1)

## Context
`BitmapObject` held only a placement and an opaque `sourceRef`. Owner decision 2026-11-10: the pixel
data is stored inside the `.carv` file so a job is portable and bitmap-driven toolpaths (Photo V-carve,
T-0275) can read it.

## Decision
1. The encoded file (PNG, JPEG, Netpbm, ...) is embedded verbatim as `BitmapAsset{key, bytes}`, where
   key = FNV-1a 64 of the bytes (16 hex digits, same as fonts). Identical images are stored once;
   save -> load -> save is byte exact. Core stays codec-free: `decodeBitmapObject` reads Netpbm itself and
   takes an app-supplied `BitmapDecoder` (QImage) for everything else.
2. A bitmap object refers to its image by putting the key in `source_ref`. Any other `source_ref` is
   still an opaque legacy reference (no pixels); such files load and save unchanged.
3. Format minor 1.6: archive members `bitmaps/<key>.img`, written only for images that a bitmap object
   names; the minor is 1.6 only then, so other documents are byte-identical to before. A 1.5 or older
   reader treats the members as unknown archive members and writes them back, so nothing is lost.
   `CarvExtras::withMember` reserves exactly `bitmaps/<16 hex>.img`.
4. Caps: 64 MiB per image, 256 MiB and 256 images per document. Loading rejects (never crashes) a member
   whose content hash differs from its name, an empty one, or one over a cap; saving refuses a bitmap
   object whose key has no embedded image. `Document::checkIntegrity` reports `MissingBitmap`.
5. Import is one undo step (`makeImportBitmapCommand`: `AddBitmapAssetCommand` + `AddObjectCommand`).
   Undo removes the image only if that command added it. Unreferenced images stay in memory until save
   and are then dropped.

## Consequences
Files grow by the encoded image size. A 64-bit non-cryptographic hash can collide in principle; a
different image with an equal key is refused ("a different image has the same key") rather than merged.
