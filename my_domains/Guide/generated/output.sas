begin_version
3
end_version
begin_metric
0
end_metric
12
begin_variable
var0
-1
2
Atom at(elektron, init-hum)
NegatedAtom at(elektron, init-hum)
end_variable
begin_variable
var1
-1
2
Atom at(elektron, init-rob)
NegatedAtom at(elektron, init-rob)
end_variable
begin_variable
var2
-1
2
Atom at(elektron, kuchnia)
NegatedAtom at(elektron, kuchnia)
end_variable
begin_variable
var3
-1
2
Atom at(zdzisiu, init-rob)
NegatedAtom at(zdzisiu, init-rob)
end_variable
begin_variable
var4
-1
2
Atom at(zdzisiu, kuchnia)
NegatedAtom at(zdzisiu, kuchnia)
end_variable
begin_variable
var5
-1
2
Atom close(elektron, elektron)
NegatedAtom close(elektron, elektron)
end_variable
begin_variable
var6
-1
2
Atom close(elektron, zdzisiu)
NegatedAtom close(elektron, zdzisiu)
end_variable
begin_variable
var7
-1
2
Atom followed(elektron)
NegatedAtom followed(elektron)
end_variable
begin_variable
var8
-1
2
Atom follows(elektron, elektron)
NegatedAtom follows(elektron, elektron)
end_variable
begin_variable
var9
-1
2
Atom follows(zdzisiu, elektron)
NegatedAtom follows(zdzisiu, elektron)
end_variable
begin_variable
var10
-1
2
Atom interacts(elektron, elektron)
NegatedAtom interacts(elektron, elektron)
end_variable
begin_variable
var11
-1
2
Atom interacts(elektron, zdzisiu)
NegatedAtom interacts(elektron, zdzisiu)
end_variable
0
begin_state
1
0
1
1
1
1
1
1
1
1
1
1
end_state
begin_goal
2
4 0
11 1
end_goal
26
begin_operator
approach elektron elektron init-hum
1
0 0
1
0 5 1 0
1
end_operator
begin_operator
approach elektron elektron init-rob
1
1 0
1
0 5 1 0
1
end_operator
begin_operator
approach elektron elektron kuchnia
1
2 0
1
0 5 1 0
1
end_operator
begin_operator
approach elektron zdzisiu init-hum
1
0 0
1
0 6 1 0
1
end_operator
begin_operator
approach elektron zdzisiu init-rob
2
1 0
3 0
1
0 6 1 0
1
end_operator
begin_operator
approach elektron zdzisiu kuchnia
2
2 0
4 0
1
0 6 1 0
1
end_operator
begin_operator
end_follow elektron elektron
1
10 0
2
0 7 0 1
0 8 0 1
1
end_operator
begin_operator
end_follow elektron zdzisiu
1
11 0
2
0 7 0 1
0 9 0 1
1
end_operator
begin_operator
end_interaction elektron elektron
2
5 0
8 1
1
0 10 0 1
1
end_operator
begin_operator
end_interaction elektron zdzisiu
2
6 0
9 1
1
0 11 0 1
1
end_operator
begin_operator
go elektron init-hum init-rob elektron
0
2
0 0 0 1
0 1 1 0
1
end_operator
begin_operator
go elektron init-hum init-rob zdzisiu
0
3
0 0 0 1
0 1 1 0
1 9 0 3 -1 0
1
end_operator
begin_operator
go elektron init-hum kuchnia elektron
0
2
0 0 0 1
0 2 1 0
1
end_operator
begin_operator
go elektron init-hum kuchnia zdzisiu
0
3
0 0 0 1
0 2 1 0
1 9 0 4 -1 0
1
end_operator
begin_operator
go elektron init-rob init-hum elektron
0
2
0 0 1 0
0 1 0 1
1
end_operator
begin_operator
go elektron init-rob init-hum zdzisiu
0
2
0 0 1 0
0 1 0 1
1
end_operator
begin_operator
go elektron init-rob kuchnia elektron
0
2
0 1 0 1
0 2 1 0
1
end_operator
begin_operator
go elektron init-rob kuchnia zdzisiu
0
3
0 1 0 1
0 2 1 0
1 9 0 4 -1 0
1
end_operator
begin_operator
go elektron kuchnia init-hum elektron
0
2
0 0 1 0
0 2 0 1
1
end_operator
begin_operator
go elektron kuchnia init-hum zdzisiu
0
2
0 0 1 0
0 2 0 1
1
end_operator
begin_operator
go elektron kuchnia init-rob elektron
0
2
0 1 1 0
0 2 0 1
1
end_operator
begin_operator
go elektron kuchnia init-rob zdzisiu
0
3
0 1 1 0
0 2 0 1
1 9 0 3 -1 0
1
end_operator
begin_operator
init_follow elektron elektron
1
10 0
2
0 7 -1 0
0 8 1 0
1
end_operator
begin_operator
init_follow elektron zdzisiu
1
11 0
2
0 7 -1 0
0 9 1 0
1
end_operator
begin_operator
start_interaction elektron elektron
1
5 0
1
0 10 1 0
1
end_operator
begin_operator
start_interaction elektron zdzisiu
1
6 0
1
0 11 1 0
1
end_operator
0
