let i = 0;
while (i < 3) { print i; i = i + 1; }
for (let j = 0; j < 10; j = j + 1) {
  if (j == 2) continue;
  if (j == 5) break;
  print "j" + j;
}
let n = 0;
for (;;) { n = n + 1; if (n > 4) break; }
print n;
if (n == 5) print "five"; else print "not five";
if (false) print "no";
else if (true) print "else-if";
// nested loops with break only leaving the inner loop
for (let x = 0; x < 3; x = x + 1) {
  for (let y = 0; y < 3; y = y + 1) {
    if (y == 1) break;
    print str(x) + ":" + str(y);
  }
}
let k = 0;
while (true) {
  let tmp = k * 2;
  k = k + 1;
  if (k < 3) continue;
  print tmp;
  break;
}
