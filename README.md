# am_traj
Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight

__am_traj__ is a framework for generating large-scale piecewise polynomial trajectories for aggressive autonomous flights, with highlights on its superior computational efficiency and simultaneous spatial-temporal optimality. Besides, an extremely fast feasibility checker is designed for various kinds of constraints. All components in this framework leverage the algebraic convenience of polynomial trajectory optimization problem, thus our method is capable of computing spatial-temporal optimal trajectories with 60 pieces within 5ms, i.e., 150Hz at least.

The source code will be available after the publication of the related paper.

__Author__: Zhepei Wang from the [ZJU Fast Lab](http://www.kivact.com/).

__Related Paper__:
- __Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight__, Zhepei Wang, Xin Zhou, Chao Xu, Jian Chu, and Fei Gao, submitted to RA-L/IROS 2020.
- [__Detailed Proofs of Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight__](https://arxiv.org/abs/2002.09254), Zhepei Wang, Xin Zhou, Chao Xu, and Fei Gao, the supplementary material.

[__Video__](https://youtu.be/ayoQ7i1Lz5s):

<p align="center">
  <img src="misc/cp.gif" width = "533" height = "300"/>
</p>

<p align="center">
  <img src="misc/sv.gif" width = "427" height = "240"/>

  <img src="misc/fpv.gif" width = "427" height = "240"/>
</p>
