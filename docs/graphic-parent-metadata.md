# KiCad 10.0.6 graphic ownership metadata

KiCad 10.0.6 adds read-only `parent` field 6 to BoardGraphicShape and BoardText. Preserve it during protobuf decoding and re-encoding. Older schemas dropped the field, causing guarded footprint presentation updates to refuse live edits as lossy. Newly constructed items leave the parent unset for KiCad to assign.

Source: https://github.com/KiCad/kicad-source-mirror/blob/10.0.6/api/proto/board/board_types.proto

Verified in the V1 cleanup integration with live documentation-graphic and field updates, full footprint readback, and unchanged pad geometry/nets.
