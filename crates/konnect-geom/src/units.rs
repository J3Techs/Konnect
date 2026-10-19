//! Board units: KiCad's internal nanometre.

/// One board coordinate or length, in nanometres.
///
/// `i32` is enough for any real board (KiCad's own limit is about ±2.1 m) and
/// halves the memory of every raster, index entry and GPU buffer against
/// `i64`. Arithmetic widens before it can overflow — see the crate docs.
pub type Coord = i32;

/// Nanometres per millimetre.
pub const NM_PER_MM: f64 = 1_000_000.0;

/// The largest coordinate magnitude [`mm_to_nm`] accepts: 2^30 nm, about
/// 1,074 mm. The headroom below `i32::MAX` lets a bounding box be inflated by
/// any realistic clearance without overflowing.
pub const MAX_COORD: Coord = 1 << 30;

/// Millimetres (as KiCad files and IPC messages carry them) to nanometres,
/// rounded to the nearest nanometre — the same rounding `konnect-ipc` uses
/// when it builds items, so a round trip through the plan is exact.
///
/// `None` for a non-finite value or one beyond [`MAX_COORD`]: a coordinate
/// that cannot be represented is refused, never clamped onto the board.
pub fn mm_to_nm(mm: f64) -> Option<Coord> {
    if !mm.is_finite() {
        return None;
    }
    let nm = (mm * NM_PER_MM).round();
    if nm.abs() > f64::from(MAX_COORD) {
        return None;
    }
    Some(nm as Coord)
}

/// Nanometres to millimetres. Accepts `i64` so widened lengths convert too.
pub fn nm_to_mm(nm: i64) -> f64 {
    nm as f64 / NM_PER_MM
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kicad_six_decimal_millimetres_convert_exactly() {
        assert_eq!(mm_to_nm(0.1), Some(100_000));
        assert_eq!(mm_to_nm(248.8), Some(248_800_000));
        assert_eq!(mm_to_nm(-0.199999), Some(-199_999));
        assert_eq!(mm_to_nm(439.458691), Some(439_458_691));
    }

    #[test]
    fn round_trip_through_millimetres_is_exact() {
        for nm in [0, 1, -1, 199_999, 248_800_000, 439_458_691, MAX_COORD] {
            assert_eq!(mm_to_nm(nm_to_mm(i64::from(nm))), Some(nm));
        }
    }

    #[test]
    fn unrepresentable_values_are_refused() {
        assert_eq!(mm_to_nm(f64::NAN), None);
        assert_eq!(mm_to_nm(f64::INFINITY), None);
        assert_eq!(mm_to_nm(2_000.0), None);
    }
}
