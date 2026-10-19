// Raster paint/unpaint: the WGSL twin of cpu.rs. Keep the two in step;
// the parity tests compare them bit for bit.
//
// One workgroup per stamp. Its invocations stride over the raster words
// of the stamp's bounding box on each of its layers, build a 4-cell
// coverage mask per word and apply a saturating per-byte +1 or -1 with a
// compare-exchange loop (WGSL has no u8 atomics). Saturating +-1 updates of
// one op commute, so the result is independent of scheduling.
//
// Integer only. Coordinates are tile-local Q4 (1/16 cell); cell (x, y)
// has its centre at (16x + 8, 16y + 8). The host's layout bounds keep all
// i32 products below 2^31; the one 64-bit comparison is done in two u32
// halves so the kernel needs no SHADER_INT64.

struct Params {
    width: u32,
    height: u32,
    layers: u32,
    row_words: u32,
    stamp_base: u32,
    stamp_count: u32,
    op: u32, // 0 = paint, 1 = unpaint
    // This dispatch's slot in `clamped`. One u32 per dispatch cannot wrap
    // (a dispatch visits at most 4M words of 4 cells); the host sums the
    // slots in u64, as the CPU counts.
    chunk: u32,
}

// Mirrors layout::PreparedStamp (48 bytes).
struct Stamp {
    ax: i32,
    ay: i32,
    bx: i32,
    by: i32,
    radius: u32,
    layer_first: u32,
    layer_last: u32,
    x0: u32,
    y0: u32,
    x1: u32,
    y1: u32,
    pad0: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> stamps: array<Stamp>;
@group(0) @binding(2) var<storage, read_write> grid: array<atomic<u32>>;
@group(0) @binding(3) var<storage, read_write> clamped: array<atomic<u32>>;

const WORKGROUP_SIZE: u32 = 256u;
const Q4_PER_CELL: i32 = 16;
const HALF_CELL: i32 = 8;

// Full 64-bit product of two u32 values as (low, high).
fn mul_wide(a: u32, b: u32) -> vec2<u32> {
    let a0 = a & 0xFFFFu;
    let a1 = a >> 16u;
    let b0 = b & 0xFFFFu;
    let b1 = b >> 16u;
    let p00 = a0 * b0;
    let p01 = a0 * b1;
    let p10 = a1 * b0;
    let p11 = a1 * b1;
    let mid = (p00 >> 16u) + (p01 & 0xFFFFu) + (p10 & 0xFFFFu);
    let lo = (p00 & 0xFFFFu) | (mid << 16u);
    let hi = p11 + (p01 >> 16u) + (p10 >> 16u) + (mid >> 16u);
    return vec2<u32>(lo, hi);
}

fn wide_le(a: vec2<u32>, b: vec2<u32>) -> bool {
    return a.y < b.y || (a.y == b.y && a.x <= b.x);
}

// Whether d (relative to the segment start) is within the capsule of
// radius sqrt(r2) around the segment 0..v; l2 = |v|^2. Same as cpu::covers.
fn covers(dx: i32, dy: i32, vx: i32, vy: i32, l2: i32, r2: i32) -> bool {
    let t = dx * vx + dy * vy;
    if (t <= 0) {
        return dx * dx + dy * dy <= r2;
    }
    if (t >= l2) {
        let ex = dx - vx;
        let ey = dy - vy;
        return ex * ex + ey * ey <= r2;
    }
    let cross = u32(abs(dx * vy - dy * vx));
    return wide_le(mul_wide(cross, cross), mul_wide(u32(r2), u32(l2)));
}

fn apply_word(index: u32, mask: u32) {
    var old = atomicLoad(&grid[index]);
    loop {
        var updated = old;
        var clamped_here = 0u;
        for (var k = 0u; k < 4u; k++) {
            let shift = 8u * k;
            if (((mask >> shift) & 1u) == 0u) {
                continue;
            }
            let cell = (old >> shift) & 0xFFu;
            if (params.op == 0u) {
                if (cell == 0xFFu) {
                    clamped_here += 1u;
                } else {
                    updated += 1u << shift;
                }
            } else {
                if (cell == 0u) {
                    clamped_here += 1u;
                } else {
                    updated -= 1u << shift;
                }
            }
        }
        var done = updated == old;
        if (!done) {
            let result = atomicCompareExchangeWeak(&grid[index], old, updated);
            done = result.exchanged;
            old = result.old_value;
        }
        if (done) {
            if (clamped_here != 0u) {
                atomicAdd(&clamped[params.chunk], clamped_here);
            }
            return;
        }
    }
}

@compute @workgroup_size(256)
fn paint(
    @builtin(workgroup_id) workgroup_id: vec3<u32>,
    @builtin(local_invocation_index) local_index: u32,
) {
    if (workgroup_id.x >= params.stamp_count) {
        return;
    }
    let s = stamps[params.stamp_base + workgroup_id.x];
    let vx = s.bx - s.ax;
    let vy = s.by - s.ay;
    let l2 = vx * vx + vy * vy;
    let r = i32(s.radius);
    let r2 = r * r;

    let word_x0 = s.x0 >> 2u;
    let words_per_row = (s.x1 >> 2u) - word_x0 + 1u;
    let words_per_layer = words_per_row * (s.y1 - s.y0 + 1u);
    let total = words_per_layer * (s.layer_last - s.layer_first + 1u);
    let plane_words = params.height * params.row_words;

    for (var i = local_index; i < total; i += WORKGROUP_SIZE) {
        let layer = s.layer_first + i / words_per_layer;
        let in_layer = i % words_per_layer;
        let y = s.y0 + in_layer / words_per_row;
        let word_x = word_x0 + in_layer % words_per_row;
        let dy = i32(y) * Q4_PER_CELL + HALF_CELL - s.ay;

        var mask = 0u;
        for (var k = 0u; k < 4u; k++) {
            let x = word_x * 4u + k;
            if (x < s.x0 || x > s.x1) {
                continue;
            }
            let dx = i32(x) * Q4_PER_CELL + HALF_CELL - s.ax;
            if (covers(dx, dy, vx, vy, l2, r2)) {
                mask |= 1u << (8u * k);
            }
        }
        if (mask != 0u) {
            apply_word(layer * plane_words + y * params.row_words + word_x, mask);
        }
    }
}
