\ Model LRP_D:/Victor/Pos-Graduacao/UFV/Project/Instances/Instances_Prodhon_LRP/test04.txt
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x(0,0,0) + 70.71067810058594 x(0,1,0) + 565.7951354980469 x(0,2,0)
   + 70.71067810058594 x(1,0,0) + 0 x(1,1,0) + 505.3851647377014 x(1,2,0)
   + 65.79513549804688 x(2,0,0) + 5.385164737701416 x(2,1,0) + 2000 y(0)
   + 300 y(1) + 0 u(0,0) + 0 u(1,0) + 0 u(2,0)
Subject To
 c2(0): 13 x(0,2,0) + 13 x(1,2,0) + 13 x(2,2,0) <= 100
 c3(0,0): x(0,1,0) + x(0,2,0) - x(1,0,0) - x(2,0,0) = 0
 c3(0,1): - x(0,1,0) + x(1,0,0) + x(1,2,0) - x(2,1,0) = 0
 c3(0,2): - x(0,2,0) - x(1,2,0) + x(2,0,0) + x(2,1,0) = 0
 c4(0): x(0,2,0) + x(1,2,0) <= 1
 c6(0,0,0): - f(0,0) + 2 x(0,2,0) + x(1,2,0) <= 1
 c6(1,0,0): - f(1,0) + x(0,2,0) + 2 x(1,2,0) <= 1
 c7(0): 13 f(0,0) - 50 y(0) <= 0
 c7(1): 13 f(1,0) - 47 y(1) <= 0
 c8(0): f(0,0) + f(1,0) = 1
Bounds
Binaries
 f(0,0) f(1,0) x(0,0,0) x(0,1,0) x(0,2,0) x(1,0,0) x(1,1,0) x(1,2,0)
 x(2,0,0) x(2,1,0) x(2,2,0) y(0) y(1)
End
