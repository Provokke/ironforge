fn add(a, b) { return a + b; }
print add(2, 3);
fn noret() { }
print noret();
fn fib(n) { if (n < 2) return n; return fib(n - 1) + fib(n - 2); }
print fib(20);
fn fact(n) { if (n <= 1) return 1; return n * fact(n - 1); }
print fact(10);
print add;
print len;
let sq = fn(x) { return x * x; };
print sq(9);
fn apply(f, v) { return f(v); }
print apply(sq, 4);
print apply(fn(x) { return x + 1; }, 1);
print type(add);
// mutual recursion through globals
fn is_even(n) { if (n == 0) return true; return is_odd(n - 1); }
fn is_odd(n) { if (n == 0) return false; return is_even(n - 1); }
print is_even(10);
print is_odd(7);
