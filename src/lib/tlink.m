function A = tlink(t, l, d, a)

A = rotz(t)*trans(l,0,d)*rotx(a);