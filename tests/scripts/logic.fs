print true and false;
print true or false;
print nil or "default";
print 0 and "zero is truthy";
print !nil;
print !0;
print 1 < 2;
print 2 <= 2;
print 3 > 4;
print "abc" < "abd";
print "b" >= "a";
print 1 == 1;
print 1 == "1";
print nil == nil;
print "x" + "y" == "xy";
print [1] == [1];
let l = [1];
print l == l;
print 1 != 2;
// short circuit: the right side must not run
fn boom() { print "boom"; return true; }
print false and boom();
print true or boom();
