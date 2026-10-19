//! WGSL sources. They are plain strings so they can be validated with naga
//! in default (GPU-less) builds and CI, not only when a device compiles
//! them.

/// The raster paint/unpaint kernel, twin of [`crate::cpu`].
pub const PAINT_WGSL: &str = include_str!("shaders/paint.wgsl");
/// Entry point of [`PAINT_WGSL`].
pub const PAINT_ENTRY: &str = "paint";
/// Workgroup size of [`PAINT_ENTRY`]; one workgroup paints one stamp.
pub const PAINT_WORKGROUP_SIZE: u32 = 256;
/// Byte size of the kernel's `Params` uniform.
pub const PAINT_PARAMS_BYTES: u64 = 32;

/// An empty kernel for launch-overhead measurements.
pub const NOOP_WGSL: &str = include_str!("shaders/noop.wgsl");
/// Entry point of [`NOOP_WGSL`].
pub const NOOP_ENTRY: &str = "noop";

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::PreparedStamp;

    /// Parse and fully validate with no optional capabilities: the kernels
    /// must run on any WebGPU-class adapter, lavapipe included.
    fn validate(source: &str) -> naga::Module {
        let module = naga::front::wgsl::parse_str(source)
            .unwrap_or_else(|e| panic!("WGSL parse failed:\n{}", e.emit_to_string(source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap_or_else(|e| panic!("WGSL validation failed:\n{}", e.emit_to_string(source)));
        module
    }

    fn entry<'a>(module: &'a naga::Module, name: &str) -> &'a naga::EntryPoint {
        module
            .entry_points
            .iter()
            .find(|ep| ep.name == name)
            .unwrap_or_else(|| panic!("entry point '{name}' not found"))
    }

    #[test]
    fn paint_kernel_validates_without_optional_capabilities() {
        let module = validate(PAINT_WGSL);
        let ep = entry(&module, PAINT_ENTRY);
        assert_eq!(ep.stage, naga::ShaderStage::Compute);
        assert_eq!(ep.workgroup_size, [PAINT_WORKGROUP_SIZE, 1, 1]);
    }

    #[test]
    fn noop_kernel_validates() {
        let module = validate(NOOP_WGSL);
        assert_eq!(entry(&module, NOOP_ENTRY).stage, naga::ShaderStage::Compute);
    }

    #[test]
    fn wgsl_struct_sizes_match_the_host_layout() {
        let module = validate(PAINT_WGSL);
        let mut layouter = naga::proc::Layouter::default();
        layouter.update(module.to_ctx()).unwrap();
        let size_of = |name: &str| {
            let (handle, _) = module
                .types
                .iter()
                .find(|(_, ty)| ty.name.as_deref() == Some(name))
                .unwrap_or_else(|| panic!("WGSL type '{name}' not found"));
            u64::from(layouter[handle].size)
        };
        assert_eq!(
            size_of("Stamp"),
            std::mem::size_of::<PreparedStamp>() as u64
        );
        assert_eq!(size_of("Params"), PAINT_PARAMS_BYTES);
    }
}
