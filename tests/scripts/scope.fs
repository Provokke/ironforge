let a = "global";
{
  let a = "outer";
  {
    let a = "inner";
    print a;
  }
  print a;
}
print a;
let b = 1;
{
  b = 2;
  let c = b + 1;
  print c;
}
print b;
