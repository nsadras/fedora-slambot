h = []
s = []
v = []
while True:
    try:
        x = raw_input().split()
        h.append(int(x[0]))
        s.append(int(x[1]))
        v.append(int(x[2]))
    except:
        break;
print 'h min',min(h)
print 'h max',max(h)
print 'h avg',(min(h) + max(h))/2
print 'h thresh',(min(h) + max(h))/2 - min(h)

print 's min',min(s)
print 's max',max(s)
print 's avg',(min(s) + max(s))/2
print 's thresh',(min(s) + max(s))/2 - min(s)

print 'v min',min(v)
print 'v max',max(v)
print 'v avg',(min(v) + max(v))/2
print 'v thresh',(min(v) + max(v))/2 - min(v)
