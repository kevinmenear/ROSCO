import struct
vals = eval(open('/tmp/probe/vals.txt').read())
def b(x):
    return struct.pack('<d', x)
def prod_ref(DT,CF):
    return 2.0*(DT*DT)*(CF*CF)
def prod_mut(DT,CF):
    return (2.0*DT)*DT*(CF*CF)
hits_b1=[]; hits_a1=[]
for DT in vals:
    for CF in vals:
        try:
            r=prod_ref(DT,CF); m=prod_mut(DT,CF)
        except OverflowError:
            continue
        if b(r)!=b(m): hits_b1.append((DT,CF))
        if b(r-8.0)!=b(m-8.0): hits_a1.append((DT,CF))
print("ladder x ladder pairs:", len(vals)**2)
print("b1 form killed by", len(hits_b1), "pair(s); e.g.", hits_b1[:3])
print("a1 form killed by", len(hits_a1), "pair(s); e.g.", hits_a1[:5])
