// IronForge demo scene, driven entirely by forge script.
// Builds a ring of cubes around a central sphere and animates it.

let ring = [];
let count = 10;
let t = 0;

fn hsv(h) {
    // cheap hue -> rgb ramp
    let r = abs(h * 6 - 3) - 1;
    let g = 2 - abs(h * 6 - 2);
    let b = 2 - abs(h * 6 - 4);
    return [min(max(r, 0), 1), min(max(g, 0), 1), min(max(b, 0), 1)];
}

let core = scene.spawn("sphere", "Core");
core.set_position(0, 1.6, 0);
core.scale = 2.2;
core.set_color(0.95, 0.75, 0.3);

for (let i = 0; i < count; i = i + 1) {
    let a = i / count * 6.2831853;
    let c = scene.spawn("cube", "Cube" + str(i));
    c.set_position(cos(a) * 4.5, 0.6, sin(a) * 4.5);
    let rgb = hsv(i / count);
    c.set_color(rgb[0], rgb[1], rgb[2]);
    ring.push(c);
}

// moons orbit the core; each one is a child so it inherits the core transform
let moon = core.spawn("sphere", "Moon");
moon.scale = 0.25;
moon.set_position(0.9, 0.2, 0);
moon.set_color(0.6, 0.8, 1.0);

let ticks = 0;
core.connect("process", fn(dt) { ticks = ticks + 1; });

fn update(dt) {
    t = t + dt;
    core.yaw(dt * 0.8);
    for (let i = 0; i < len(ring); i = i + 1) {
        let c = ring[i];
        let phase = t * 2 + i * 0.7;
        c.y = 0.6 + abs(sin(phase)) * 1.2;
        c.rotate(1, 1, 0, dt * (1 + i * 0.1));
    }
}
