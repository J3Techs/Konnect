// An empty kernel for measuring launch and submit overhead. It binds
// nothing, so the measurement is the dispatch itself.

@compute @workgroup_size(64)
fn noop() {
}
