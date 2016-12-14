************************************************************************
file with basedata            : me13_.bas
initial value random generator: 558876320
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  14
horizon                       :  100
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  0   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     12      0       17       10       17
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2          12  13
   3        3          2           5   6
   4        3          3           9  11  13
   5        3          3           7   8   9
   6        3          3           7   8   9
   7        3          3          10  11  13
   8        3          2          11  12
   9        3          1          10
  10        3          1          12
  11        3          1          14
  12        3          1          14
  13        3          1          14
  14        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2
------------------------------------------------------------------------
  1      1     0       0    0
  2      1     1       5    6
         2     5       5    3
         3     8       5    2
  3      1     6       5    7
         2     7       4    5
         3    10       2    3
  4      1     8      10    4
         2    10       6    4
         3    10       7    2
  5      1     1       8    7
         2     2       7    6
         3     8       4    5
  6      1     2       7    5
         2     2       6    6
         3     9       6    4
  7      1     1       9    9
         2     3       7    9
         3     7       6    9
  8      1     3      10    7
         2     6       8    5
         3     8       7    4
  9      1     4       6    5
         2     8       5    4
         3     8       6    1
 10      1     2      10    6
         2     3       6    5
         3     9       3    4
 11      1     5       6    9
         2     8       4    7
         3     8       2    8
 12      1     3       8    8
         2     4       8    7
         3     7       7    5
 13      1     3       8    3
         2     5       7    2
         3     8       6    1
 14      1     0       0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2
   12   12
************************************************************************