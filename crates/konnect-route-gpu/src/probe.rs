//! Adapter facts recorded into run evidence. Plain data, so it is available
//! (and serializable) whether or not the `gpu` feature is built.

use serde::Serialize;

/// What the adapter offers that the router's kernels care about. The flags
/// are the adapter's, not the device's: a kernel that needs one must also
/// have been granted it when the device was requested.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize)]
pub struct AdapterProbe {
    pub name: String,
    pub backend: String,
    pub device_type: String,
    pub vendor_id: u32,
    pub device_id: u32,
    pub driver: String,
    pub driver_info: String,
    pub subgroup_min_size: u32,
    pub subgroup_max_size: u32,
    pub shader_int64: bool,
    /// u64 atomicMin/Max: the packed `(cost << 32 | index)` relax of the
    /// sweep kernel. Without it the sweep needs a two-pass u32 relax.
    pub shader_int64_atomic_min_max: bool,
    pub shader_int64_atomic_all_ops: bool,
    pub subgroup: bool,
    pub timestamp_query: bool,
    pub max_storage_buffer_binding_size_bytes: u64,
    pub max_buffer_size_bytes: u64,
    pub max_compute_workgroup_storage_size_bytes: u32,
    pub max_compute_invocations_per_workgroup: u32,
    pub max_compute_workgroups_per_dimension: u32,
}
