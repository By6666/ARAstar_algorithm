# Anytime Repairing Astar algorithm

* Header file in the folder [include](./include).
 
* testing maps int the folder [inputs](./inputs).

* Code of reading map in the file [grid_input.cpp](./grid_input.cpp).

* Algorithm code file [ARAstar_algorithm.cpp](./ARAstar_algorithm.cpp).

* Algorithm use container of vector [ARAstar_useVector](./ARAstar_useVector).

This algorithm has control over a suboptimality bound for its current solution, which it uses to achieve the anytime property: it starts by ﬁnding a suboptimal solution quickly using a loose bound, then tightens the bound progressively as time allows. Given enough time it ﬁnds a provably optimal solution. While improving its bound, ARA* reuses previous search efforts and, as a result, is very efﬁcient. We demonstrate this claim empirically on a motion planning application involving a simulated robotic arm with several degrees of freedom.

