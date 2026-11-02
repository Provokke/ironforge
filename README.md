<div align="center">

# IronForge

A small, original, modular C++20 game-engine core. Headless, no dependencies beyond the C++ standard library (the test framework, doctest, is vendored as a single header). The architecture borrows ideas from Godot (a node tree with signals, script...

![License](https://img.shields.io/badge/LICENSE-MIT-blue?style=for-the-badge)  ![Language](https://img.shields.io/badge/C%2B%2B20-f34b7d?style=for-the-badge)

Refer to [LICENSE](LICENSE) for the legal bits.

</div>

## 🏗️ Architecture

```
                       +---------------------------------------------+
   demo/scripts/*.fs   |  script/   forge script                     |
   ------------------> |  lexer -> Pratt parser/compiler -> bytecode |
                       |  -> VM (closures, upvalues, natives)        |
                       |  bindings.cpp: Node handles <-> script      |
                       +--------------------+------------------------+
                                            | drives
          +-----------------+    +----------v-----------+   +------------------+
          | memory/         |    | scene/               |   | physics/ (bonus) |
          | pool, frame,    |    | Node tree + signals  |   | spheres & boxes, |
          | free-list,      |    | ECS sparse sets      |   | BVH broadphase   |
          | tracking        |    | fixed-step loop      |   +--------+---------+
          +-----------------+    +----------+-----------+            |
                                            | gather MeshNodes       |
   +--------------+      +------------------v-----------------+      |
   | jobs/        |----->| render/  tile-binned rasterizer    |      |
   | work-stealing|      | vertex -> setup/clip/bin -> raster |      |
   | thread pool  |      | -> PNG (own encoder)               |      |
   +--------------+      +------------------------------------+      |
                                                                     |
   +-----------------------------------------------------------------v--+
   | math/  vec2/3/4, mat3/4, quat, transform, AABB, ray/plane/sphere/  |
   |        triangle tests, frustum      spatial/ dynamic BVH, octree   |
   +--------------------------------------------------------------------+
```

| Directory | Contents |
|---|---|
| `src/math` | vectors, column-major matrices (inverse, determinant, projection, look-at), quaternions (slerp, matrix conversion), transforms, AABB (Arvo transform), ray-plane/sphere/AABB/triangle (Moller-Trumbore), sphere/AABB overlap, frustum extraction + AABB classification |
| `src/memory` | `PoolAllocator` (O(1) fixed blocks), `FrameAllocator` (bump + markers), `FreeListAllocator` (address-ordered, first/best fit, split + coalesce, fragmentation stats, self-validation), `TrackingAllocator` (leak report by tag, peak, double-free detection) |
| `src/spatial` | `DynamicBVH`: fat AABBs, surface-area-cost insertion, SAH tree rotations on refit, AABB queries and ordered closest-hit ray casts. `LooseOctree` (k=2) with overflow list |
| `src/scene` | `Node` tree (ownership, unique names, path lookup with `..` and absolute paths, global transforms, deferred `queue_free`, typed `Signal<>` + named signals), `Registry` ECS (generational ids, sparse sets, multi-component `each`), `FixedStepLoop` (deterministic, spiral-of-death guard, interpolation alpha) |
| `src/jobs` | work-stealing pool: per-worker deques (LIFO own / FIFO steal), injection queue, `WaitGroup`, helping `wait`, `parallel_for` |
| `src/render` | meshes (cube, sphere, checker plane), rasterizer, PNG encoder/decoder (zlib stored blocks, CRC32, Adler-32), PPM writer, image diff |
| `src/script` | forge script: lexer, single-pass Pratt compiler to stack bytecode, disassembler, VM, scene bindings |
| `src/physics` | linear rigid bodies, sphere/box contacts, BVH broadphase, sequential impulses |
| `src/engine` | the shared demo scene (script + node tree -> draw list) |

### Rasterizer pipeline

1. **Vertex stage** (parallel over vertices): model -> clip space, normals by the inverse-transpose; whole objects are frustum-culled first.
2. **Setup + binning** (parallel over fixed-size triangle chunks): trivial reject, near-plane clipping (Sutherland-Hodgman on `z + w >= 0`), back-face culling, viewport transform, bin into 32x32 tiles (per-chunk bins, so no locks).
3. **Raster** (parallel over tiles): each tile clears itself, then walks the bins of every chunk *in submission order*: edge functions with a top-left fill rule, depth test, perspective-correct interpolation of colour/normal (attributes divided by w), Lambert + ambient, sqrt gamma.

Chunk size depends only on the triangle count, never the thread count, so the
image is **bit-identical for any number of threads** (tested).

### forge script

Dynamically typed: `nil`, booleans, numbers (double), strings, lists, functions/closures, engine nodes.

```
let ring = [];
fn make_counter() { let n = 0; return fn() { n = n + 1; return n; }; }
for (let i = 0; i < 10; i = i + 1) {
    let c = scene.spawn("cube", "Cube" + str(i));
    c.set_position(cos(i) * 4, 0.6, sin(i) * 4);
    ring.push(c);
}
core.connect("process", fn(dt) { ticks = ticks + 1; });   // engine signal -> closure
fn update(dt) { for (let i = 0; i < len(ring); i = i + 1) ring[i].yaw(dt); }
```

Statements: `let`, `fn`, `if/else`, `while`, C-style `for`, `break`, `continue`, `return`, `print`, blocks.
Expressions: arithmetic, comparison (numbers or strings), `and`/`or` (short-circuit), `!`, unary `-`, calls,
lambdas `fn (x) { ... }`, list literals, indexing (negative indices from the end), `.property`, `.method(...)`.
Builtins: `len push str num type sqrt sin cos floor abs min max range assert`; list methods `push pop len`.
Node API: `find get_child spawn set_position translate rotate yaw set_color queue_free connect emit`,
properties `name x y z scale path child_count parent alive`.

The compiler is a single pass (clox-style): the Pratt parser emits bytecode directly; locals are resolved
to stack slots and captured variables to upvalues at compile time. The VM uses open upvalues that point
at stack slots and are closed when the slot goes out of scope. Errors carry line numbers and a call trace:

```
$ ./build/forge tests/scripts/err_type.fs
[line 1] RuntimeError: operands of '-' must be numbers, got number and string
  at f (line 1)
  at <script> (line 2)
```

## 🚀 Quick start

Requirements: CMake >= 3.20, Ninja, GCC 13 or Clang 18 (C++20). Builds with `-Wall -Wextra -Werror -Wpedantic -Wshadow`.

```sh
make test          # == ./scripts/test.sh: Release build + ctest (the core suite)
make sanitize      # == ./scripts/sanitize.sh: whole suite under ASan+UBSan (+leaks), then TSan
make bench         # ./build/ironforge_bench [spatial|raster|vm|alloc]
./build/ironforge_demo out.png [threads] [width] [height] [ticks]
./build/forge script.fs [--disasm]
```

Sample demo output:

```
$ ./build/ironforge_demo docs/demo.png 4 960 540 90
nodes=14 draw_items=13 tris_in=5336 tris_binned=2614 tile_entries=5378
sim ticks=90  render 960x540 threads=4: vertex 1.18 ms, setup 0.69 ms, raster 4.67 ms, total 6.54 ms
image hash fnv1a64=d39b62d49e2f3241 -> docs/demo.png
```

### What the tests cover (61 test cases, ~48k assertions)

* **math**: randomized property tests (1000s of samples) — Lagrange identity, `A*A^-1 = I`, `(AB)^-1 = B^-1A^-1`,
  `det(AB) = det(A)det(B)`, `(AB)^T = B^TA^T`, quaternion composition == matrix composition, `from_mat3(to_mat3(q)) = q`,
  slerp endpoints/unit length, transform compose/inverse, Arvo AABB contains all transformed corners, ray-triangle
  barycentrics, frustum `contains` agrees with the clip-space cube test on 5000 random points.
* **memory**: pool exhaustion/reuse/ownership, frame markers, free-list coalescing back to one block for both policies,
  20k-op randomized stress with content verification and periodic `validate()`, best-fit vs first-fit choice,
  fragmentation stat on a deliberately fragmented arena, tracking leak report/double free.
* **spatial**: BVH vs brute force over 5 random dynamic scenes (insert/remove/move, AABB queries + closest-hit
  ray casts, structural `validate()` every round); sorted insertion stays shallow (height 11 for 1024 leaves);
  loose octree vs brute force including out-of-root objects.
* **scene**: signal re-entrancy (disconnect/connect during emit), node paths/names/ownership, deferred free,
  ECS vs a reference map (5000 random ops), generation reuse, fixed-step determinism under jittery frame times.
* **jobs**: every job exactly once (1/2/4/8 threads, 20k jobs), nested `parallel_for` without deadlock,
  stealing observed, 50 waves of recursive spawning, destructor drains.
* **render**: CRC32/Adler-32 reference values, PNG round trip + corruption detection, exact coverage of a
  two-triangle quad (no gaps/overlaps), depth order independence, perspective-correct interpolation checked
  against analytic ray casts, near-plane clipping, back-face/frustum culling counts, bit-identical output for
  0/1/2/3/4/8 threads, **golden image** (`tests/golden/demo_320x180.png`, <= 0.5% pixels may differ by > 8).
* **script**: 28 golden scripts in `tests/scripts/` (arithmetic, logic, strings, scope, control flow, functions,
  closures, lists, builtins and 19 error cases: runtime errors with traces, compile errors incl. multi-error
  recovery, lexer errors, stack overflow), lexer tokens, disassembler, C++ <-> script calls, error unwinding through
  natives, scene bindings, freed-node detection, script-connected `process` signal, 3000-program random-token fuzz.
* **physics**: resting contact, elastic collision momentum/velocity swap, contact normals, 20-sphere pile in a box.

Results from this machine:

| Command | Result |
|---|---|
| `./scripts/test.sh` (GCC 13, Release) | 61/61 test cases, 48415/48415 assertions passed |
| Clang 18 Release build + tests | 61/61 passed (golden image within tolerance) |
| ASan + UBSan + LeakSanitizer (Debug) | 61/61 passed, no reports |
| TSan | 61/61 passed, no reports — after fixing a real startup race it found (workers read `workers_.size()` while the constructor was still growing the vector) |

## ⏱️ Benchmarks

Measured with `./build/ironforge_bench` (Release, GCC 13) on a **shared 4-core VM** whose load average
was 3.3–4.5 from other jobs during the runs, so absolute numbers are noisy and multi-thread
scaling is understated. Re-run on an idle machine for meaningful scaling numbers.

**Spatial** (100k random boxes, 2000 queries):

| | brute force | BVH | octree |
|---|---|---|---|
| AABB overlap queries | 722 ms | 10.2 ms (**71x**) | 27.2 ms (27x) |
| closest-hit ray casts | 1588 ms | 16.7 ms (**95x**) | 74.7 ms (21x) |
| build (100k inserts) | – | 100 ms (height 26) | 55 ms |

Hit counts and ray checksums agree exactly across all three methods.

**Rasterizer**, demo scene at 960x540, median of 15 frames (two runs shown to illustrate the noise):

| scene | inline (no pool) | 1 worker | 2 workers | 4 workers |
|---|---|---|---|---|
| 5.3k tris (run A) | 9.7 ms | 6.9 ms | 7.3 ms | 5.5 ms |
| 146k tris (run A) | 27.4 ms | 29.1 ms | 33.3 ms | 31.3 ms |
| 146k tris (run B) | 38.3 ms | 32.9 ms | 38.2 ms | 39.9 ms |

Honest reading: on this saturated machine the pool gives at best ~1.8x on the light scene and no
measurable gain on the heavy one; a pure-compute `parallel_for` sanity check also only went from
95 ms (1 worker) to 78 ms (4 workers), i.e. the cores were not available. Scaling has not been
demonstrated here.

**forge script VM** (bytecode instructions executed / wall time):

| program | time | instructions | throughput |
|---|---|---|---|
| `fib(27)` recursive | 39–53 ms | 7.6 M | 143–193 M instr/s |
| 3M-iteration loop sum | 141–210 ms | 54 M | 258–384 M instr/s |
| 1M closure calls | 76–94 ms | 24 M | 255–316 M instr/s |
| 1M list push + 1M index | 272–279 ms | 39 M | 140–143 M instr/s |

**Allocators** (1M alloc/free pairs of 64 B, batches of 1000): malloc 25–28 ms, pool 3.9–6.1 ms,
free-list 9.4–10.6 ms, frame (bump + reset) 1.5 ms.

## ⚠️ Honest limits

* Script objects are reference counted; reference cycles created at runtime (self-capturing closures, lists
  containing themselves) are only reclaimed when the VM is destroyed (it breaks all cycles in its destructor).
  There is no tracing GC. No maps/dictionaries, no classes, no string formatting beyond `+`/`str()`.
* Script node handles are weak (detect freed nodes), but a signal connected from a script must not fire after its
  VM is destroyed.
* Rasterizer: no textures, no MSAA, no far-plane/side clipping (handled by depth range and bounding-box clamping),
  flat colour per vertex + Lambert only. Edge functions are evaluated per pixel (no incremental stepping/SIMD).
* The work-stealing deques are mutex-protected, not lock-free (Chase-Lev); fine for coarse jobs.
* Physics is a bonus: linear motion only (no rotation/friction), discrete collision (fast objects can tunnel).
* Transform composition is exact only for uniform scale.
* The PNG encoder uses uncompressed (stored) deflate blocks, so files are large (`docs/demo.png` is 1.5 MB);
  the decoder only reads that subset.
* Fuzzing is a light in-process random-token fuzz (3000 programs), not coverage-guided.

## Licence

MIT, see `LICENSE`. `third_party/doctest/doctest.h` is MIT-licensed (doctest authors).
