#!/usr/bin/gnuplot -persist
set size square
plot "edges.dat" w l lt 3, "vertices.dat" w p lt 2 pt 7 ps 3, "path0.dat" w l lt 4 lw 2, "path.dat" w lp lt 1 lw 1
