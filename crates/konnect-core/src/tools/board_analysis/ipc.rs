//! Reference math: IPC-2152 conductor current capacity, IPC-2221B Table 6-1
//! electrical clearance, Onderdonk fusing and copper resistance.
//!
//! IPC-2152 publishes charts, not equations. The capacity here is the
//! least-squares power fit to IPC-2152 Figure 5-2 (the generic chart:
//! 0.070 in polyimide board, 3 oz copper, still air, no plane) published with
//! the smps.us PCB calculator:
//!
//!   A [mil²] = (117.555 · ΔT^-0.913 + 1.15) · I^(0.84 · ΔT^-0.108 + 1.159)
//!
//! fitted for 2–100 °C rise and up to 30 A. Its authors' check: 10 A at
//! ΔT = 20 °C, chart 500 mil², fit 513 mil² (within 3 %). The generic chart
//! is deliberately conservative and applies to internal and external
//! conductors alike; IPC-2152's modifier charts (copper thickness, board
//! thickness, planes) are applied by the caller as one correction factor,
//! which scales the allowed rise: ΔT_virtual = ΔT / correction.

/// mil² per mm².
const MIL2_PER_MM2: f64 = 1550.0031;

/// IPC-2152 (Figure 5-2 fit): the cross-section, mm², that carries `amps`
/// at a rise of `rise_c` °C.
pub(crate) fn ipc2152_area_mm2(amps: f64, rise_c: f64) -> f64 {
    if amps <= 0.0 {
        return 0.0;
    }
    let k = 117.555 * rise_c.powf(-0.913) + 1.15;
    let e = 0.84 * rise_c.powf(-0.108) + 1.159;
    k * amps.powf(e) / MIL2_PER_MM2
}

/// IPC-2152 (Figure 5-2 fit): the current, A, a cross-section of
/// `area_mm2` carries at a rise of `rise_c` °C. The exact inverse of
/// [`ipc2152_area_mm2`].
pub(crate) fn ipc2152_current(area_mm2: f64, rise_c: f64) -> f64 {
    if area_mm2 <= 0.0 {
        return 0.0;
    }
    let k = 117.555 * rise_c.powf(-0.913) + 1.15;
    let e = 0.84 * rise_c.powf(-0.108) + 1.159;
    (area_mm2 * MIL2_PER_MM2 / k).powf(1.0 / e)
}

/// The conductor width, mm, copper `thickness_mm` thick needs for `amps`.
pub(crate) fn ipc2152_width_mm(amps: f64, rise_c: f64, thickness_mm: f64) -> f64 {
    ipc2152_area_mm2(amps, rise_c) / thickness_mm
}

/// A plated barrel's copper cross-section, mm²: an annulus of the plating
/// thickness inside a finished hole of `drill_mm` (the plating lines the
/// drilled hole, so the outer diameter is the drill).
pub(crate) fn barrel_area_mm2(drill_mm: f64, plating_mm: f64) -> f64 {
    let ro = drill_mm / 2.0;
    let ri = (ro - plating_mm).max(0.0);
    std::f64::consts::PI * (ro * ro - ri * ri)
}

/// Copper resistivity at `temp_c`, Ω·mm.
pub(crate) fn rho_at(temp_c: f64) -> f64 {
    super::common::RHO_CU_20C_OHM_MM * (1.0 + super::common::ALPHA_CU * (temp_c - 20.0))
}

/// Resistance, Ω, of a conductor `length_mm` long with cross-section
/// `area_mm2` at `temp_c`.
#[cfg(test)]
pub(crate) fn resistance(length_mm: f64, area_mm2: f64, temp_c: f64) -> f64 {
    rho_at(temp_c) * length_mm / area_mm2
}

/// Onderdonk's fusing current, A, for a copper cross-section `area_mm2`
/// heated from `ambient_c` for `seconds` (adiabatic, short pulses):
/// I = A[cmil] · sqrt(log10(1 + (Tm − Ta)/(234 + Ta)) / (33 · s)), Tm = 1083 °C.
pub(crate) fn onderdonk_fusing_current(area_mm2: f64, ambient_c: f64, seconds: f64) -> f64 {
    const CMIL_PER_MM2: f64 = 1973.525;
    let ratio = (1083.0 - ambient_c) / (234.0 + ambient_c);
    area_mm2 * CMIL_PER_MM2 * ((1.0 + ratio).log10() / (33.0 * seconds)).sqrt()
}

/// IPC-2221B Table 6-1 column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SpacingColumn {
    /// B1: internal conductors.
    B1,
    /// B2: external conductors, uncoated, sea level to 3050 m.
    B2,
    /// B3: external conductors, uncoated, above 3050 m.
    B3,
    /// B4: external conductors with permanent polymer coating, any elevation.
    B4,
    /// A5: external conductors with conformal coating over assembly.
    A5,
    /// A6: external component lead/termination, uncoated, sea level to 3050 m.
    A6,
    /// A7: external component lead/termination, with conformal coating.
    A7,
}

impl SpacingColumn {
    pub(crate) fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "B1" => Self::B1,
            "B2" => Self::B2,
            "B3" => Self::B3,
            "B4" => Self::B4,
            "A5" => Self::A5,
            "A6" => Self::A6,
            "A7" => Self::A7,
            _ => return None,
        })
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::B1 => "B1",
            Self::B2 => "B2",
            Self::B3 => "B3",
            Self::B4 => "B4",
            Self::A5 => "A5",
            Self::A6 => "A6",
            Self::A7 => "A7",
        }
    }

    fn index(self) -> usize {
        match self {
            Self::B1 => 0,
            Self::B2 => 1,
            Self::B3 => 2,
            Self::B4 => 3,
            Self::A5 => 4,
            Self::A6 => 5,
            Self::A7 => 6,
        }
    }
}

/// IPC-2221B Table 6-1 rows: upper voltage (DC or AC peak) and the minimum
/// spacing, mm, per column B1, B2, B3, B4, A5, A6, A7.
const TABLE_6_1: [(f64, [f64; 7]); 9] = [
    (15.0, [0.05, 0.1, 0.1, 0.05, 0.13, 0.13, 0.13]),
    (30.0, [0.05, 0.1, 0.1, 0.05, 0.13, 0.25, 0.13]),
    (50.0, [0.1, 0.6, 0.6, 0.13, 0.13, 0.4, 0.13]),
    (100.0, [0.1, 0.6, 1.5, 0.13, 0.13, 0.5, 0.13]),
    (150.0, [0.2, 0.6, 3.2, 0.4, 0.4, 0.8, 0.4]),
    (170.0, [0.2, 1.25, 3.2, 0.4, 0.4, 0.8, 0.4]),
    (250.0, [0.2, 1.25, 6.4, 0.4, 0.4, 0.8, 0.4]),
    (300.0, [0.2, 1.25, 12.5, 0.4, 0.4, 0.8, 0.8]),
    (500.0, [0.25, 2.5, 12.5, 0.8, 0.8, 1.5, 0.8]),
];
/// Above 500 V: mm per volt, per column.
const PER_VOLT_ABOVE_500: [f64; 7] = [0.0025, 0.005, 0.025, 0.00305, 0.00305, 0.00305, 0.00305];

/// IPC-2221B Table 6-1: the minimum conductor spacing, mm, for a voltage
/// between conductors of `volts` (DC or AC peak). Rows are bands of whole
/// volts (0–15, 16–30, …); a fractional voltage falls in the band of its
/// value rounded up.
pub(crate) fn ipc2221b_spacing_mm(volts: f64, column: SpacingColumn) -> f64 {
    let v = volts.abs().ceil();
    let c = column.index();
    for (limit, row) in TABLE_6_1 {
        if v <= limit {
            return row[c];
        }
    }
    v * PER_VOLT_ABOVE_500[c]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        ((a - b) / b).abs() <= tol
    }

    #[test]
    fn the_fit_reproduces_its_published_chart_check() {
        // 10 A at 20 °C: the IPC-2152 chart reads 500 mil², the fit 513.1.
        let mil2 = ipc2152_area_mm2(10.0, 20.0) * MIL2_PER_MM2;
        assert!((mil2 - 513.1).abs() < 0.5, "{mil2}");
        assert!(close(mil2, 500.0, 0.03));
    }

    /// The reference case: a 1.0 mm track on 35 µm F.Cu at ΔT = 10 °C, worked
    /// by hand from the fit.
    ///   A = 1.0 mm × 0.035 mm = 0.035 mm² = 54.25 mil²
    ///   K1 = 117.555 · 10^-0.913 + 1.15 = 117.555 · 0.122180 + 1.15 = 15.5130
    ///   K2 = 0.84 · 10^-0.108 + 1.159 = 0.84 · 0.779830 + 1.159 = 1.81406
    ///   I = (54.25 / 15.5130)^(1/1.81406) = 3.4971^0.551250 = 1.994 A
    #[test]
    fn a_one_millimetre_outer_track_matches_the_hand_calculation() {
        let i = ipc2152_current(1.0 * 0.035, 10.0);
        assert!(close(i, 1.994, 0.05), "{i}");
        assert!(close(i, 1.994, 0.005), "{i}");
        // And the width that carries it is the track again.
        assert!(close(ipc2152_width_mm(i, 10.0, 0.035), 1.0, 1e-9));
    }

    #[test]
    fn capacity_and_area_are_inverse_and_monotonic() {
        for (a, dt) in [(0.01, 5.0), (0.2, 30.0), (1.5, 80.0)] {
            let i = ipc2152_current(a, dt);
            assert!(close(ipc2152_area_mm2(i, dt), a, 1e-9));
            assert!(ipc2152_current(a * 2.0, dt) > i);
            assert!(ipc2152_current(a, dt * 2.0) > i);
        }
    }

    #[test]
    fn ipc2221b_table_values() {
        use SpacingColumn::*;
        assert_eq!(ipc2221b_spacing_mm(60.0, B2), 0.6);
        assert_eq!(ipc2221b_spacing_mm(60.0, B1), 0.1);
        assert_eq!(ipc2221b_spacing_mm(60.0, B4), 0.13);
        assert_eq!(ipc2221b_spacing_mm(12.0, B2), 0.1);
        assert_eq!(ipc2221b_spacing_mm(15.0, B2), 0.1);
        assert_eq!(ipc2221b_spacing_mm(15.5, B2), 0.1);
        assert_eq!(ipc2221b_spacing_mm(30.0, A6), 0.25);
        assert_eq!(ipc2221b_spacing_mm(48.0, B2), 0.6);
        assert_eq!(ipc2221b_spacing_mm(160.0, B2), 1.25);
        assert_eq!(ipc2221b_spacing_mm(400.0, B3), 12.5);
        assert!((ipc2221b_spacing_mm(1000.0, B2) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn copper_resistance_and_fusing() {
        // 100 mm of 1 mm × 35 µm at 20 °C: ρL/A = 1.7241e-5 · 100 / 0.035.
        let r = resistance(100.0, 0.035, 20.0);
        assert!(close(r, 0.049260, 1e-4), "{r}");
        // Sheet resistance of 0.5 oz (15.2 µm) ≈ 1.13 mΩ/sq; 1 oz ≈ 0.49.
        assert!(close(rho_at(20.0) / 0.0152, 0.001134, 1e-3));
        assert!(close(rho_at(25.0) / 0.035, 0.000502, 1e-2));
        // A 0.3 mm barrel with 20 µm plating.
        let a = barrel_area_mm2(0.3, 0.02);
        assert!(close(
            a,
            std::f64::consts::PI * (0.15 * 0.15 - 0.13 * 0.13),
            1e-12
        ));
        // Onderdonk: 1 mil² ≈ 1.273 cmil; 1 s from 25 °C fuses about 0.0335 A/cmil... only
        // its scaling is checked: four times the area, four times the current.
        let f = onderdonk_fusing_current(0.01, 25.0, 0.001);
        assert!(close(
            onderdonk_fusing_current(0.04, 25.0, 0.001),
            4.0 * f,
            1e-12
        ));
        assert!(close(
            onderdonk_fusing_current(0.01, 25.0, 0.004),
            f / 2.0,
            1e-12
        ));
    }
}
