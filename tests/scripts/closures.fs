fn make_counter() {
  let count = 0;
  return fn() { count = count + 1; return count; };
}
let c1 = make_counter();
let c2 = make_counter();
print c1();
print c1();
print c2();
print c1();

// closures share the captured variable
fn pair() {
  let v = 0;
  let get = fn() { return v; };
  let set = fn(x) { v = x; };
  return [get, set];
}
let p = pair();
p[1](42);
print p[0]();

// capture survives the enclosing scope (closed upvalue)
let saved;
{
  let local = "captured";
  saved = fn() { return local; };
}
print saved();

// nested capture through two levels
fn outer() {
  let x = "x";
  fn middle() {
    fn inner() { return x + "!"; }
    return inner;
  }
  return middle();
}
print outer()();

// each loop iteration body has its own block-scoped variable
let fns = [];
for (let i = 0; i < 3; i = i + 1) {
  let j = i;
  fns.push(fn() { return j; });
}
print fns[0]() + fns[1]() + fns[2]();

// recursive local function (self-capturing closure)
fn run() {
  fn countdown(n) { if (n == 0) return "liftoff"; return countdown(n - 1); }
  return countdown(5);
}
print run();
