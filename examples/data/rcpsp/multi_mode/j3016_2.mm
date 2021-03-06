************************************************************************
file with basedata            : mf16_.bas
initial value random generator: 1369083718
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  32
horizon                       :  227
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     30      0       41       25       41
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           7  15  24
   3        3          3           5  12  26
   4        3          1           8
   5        3          3           6  11  17
   6        3          3           9  13  14
   7        3          2          17  18
   8        3          3          10  14  22
   9        3          1          23
  10        3          2          16  21
  11        3          1          21
  12        3          2          18  25
  13        3          2          20  23
  14        3          2          15  16
  15        3          1          23
  16        3          2          20  30
  17        3          3          19  22  25
  18        3          3          19  22  29
  19        3          2          20  21
  20        3          1          27
  21        3          1          31
  22        3          1          27
  23        3          2          25  27
  24        3          1          28
  25        3          1          28
  26        3          2          29  30
  27        3          1          31
  28        3          3          29  30  31
  29        3          1          32
  30        3          1          32
  31        3          1          32
  32        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     2      10    8    0    6
         2     6       9    7    8    0
         3    10       8    5    6    0
  3      1     6       8    8    0    8
         2     7       8    8    5    0
         3     9       7    6    4    0
  4      1     1       7   10    8    0
         2     2       7    8    4    0
         3     4       6    4    0    5
  5      1     1       5    9    0    6
         2     2       4    9    2    0
         3     5       4    9    0    6
  6      1     6       8    8    0   10
         2     6       8    9    5    0
         3    10       8    3    0   10
  7      1     1       5    7    7    0
         2     2       4    5    6    0
         3     2       2    7    0    6
  8      1     1       8    7    7    0
         2     4       5    7    7    0
         3     9       2    6    0    4
  9      1     5       5    4    0    4
         2     7       5    3    0    1
         3     7       4    4    0    1
 10      1     7       7    5    0    5
         2    10       2    3    0    3
         3    10       1    2    0    5
 11      1     6       9    3    0    6
         2     7       7    3    0    6
         3     9       5    2    8    0
 12      1     3      10    7    9    0
         2     9      10    5    8    0
         3    10       9    4    8    0
 13      1     1       8    8    3    0
         2     1       8    6    0    6
         3     8       8    4    0    3
 14      1     1       4    8    8    0
         2     3       3    7    8    0
         3     7       2    5    7    0
 15      1     2       8    8    0    5
         2     6       5    7    5    0
         3     8       5    7    0    4
 16      1     1       5    4    7    0
         2     3       5    4    4    0
         3     5       4    3    0    5
 17      1     1       8    6    0    4
         2     1       6    7    0    7
         3     2       3    5    9    0
 18      1     1       9    7    0    8
         2     3       6    5    0    7
         3     4       5    4    8    0
 19      1     7       8    8    0    9
         2     8       4    8    0    7
         3     9       2    5    4    0
 20      1     3       2    8    0    6
         2     5       2    8    0    3
         3     6       1    7    0    2
 21      1     3       6   10    0    7
         2     3       7   10    4    0
         3     5       4    9    4    0
 22      1    10       6    8    0    9
         2    10       5    6    5    0
         3    10       7    8    2    0
 23      1     5       7    8   10    0
         2     7       7    8    0    7
         3     9       6    7   10    0
 24      1     5       8    4    0    5
         2     7       6    4    8    0
         3    10       6    4    5    0
 25      1     6       8    9    9    0
         2     6       6    9    0    4
         3     8       2    6    9    0
 26      1     8      10    8    7    0
         2     9       9    5    4    0
         3    10       6    4    4    0
 27      1     6       4    8    9    0
         2     8       2    8    6    0
         3    10       1    8    2    0
 28      1     7       7    5    0    8
         2     8       5    5    0    3
         3    10       3    4    6    0
 29      1     5       7    8    0    7
         2     8       4    5   10    0
         3    10       2    4    9    0
 30      1     3      10    5    0    7
         2     5       6    3    0    5
         3     7       5    2    7    0
 31      1     2       7   10    0    9
         2     3       7    9    0    8
         3     4       5    8    0    8
 32      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   52   44  100   92
************************************************************************
