# AM-Traj

Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight

## 0. About

__AM-Traj__ is a __C++11__ [__header-only__](https://en.wikipedia.org/wiki/Header-only) library for generating large-scale piecewise polynomial trajectories for aggressive autonomous flights, with highlights on its superior computational efficiency and simultaneous spatial-temporal optimality. Besides, an extremely fast feasibility checker is designed for various kinds of constraints. All components in this framework leverage the algebraic convenience of polynomial trajectory optimization problem, thus our method is capable of computing a spatial-temporal optimal trajectory with 60 pieces within 5ms, i.e., 150Hz at least. You just need to include "am_traj.hpp" and "root_finder.hpp" in your code. __Please use the up-to-date master branch which may have a better performance than the one in our paper.__

__Author__: Zhepei Wang and [Fei Gao](https://ustfei.com/) from the [ZJU Fast Lab](http://www.kivact.com/).

__Related Papers__:

- [__Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight__](https://ieeexplore.ieee.org/document/9121729), Zhepei Wang, Xin Zhou, Chao Xu, Jian Chu, and Fei Gao, IEEE Robotics and Automation Letters (RA-L).
- [__Detailed Proofs of Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight__](https://arxiv.org/abs/2002.09254), Zhepei Wang, Xin Zhou, Chao Xu, and Fei Gao, the supplementary material.

  If you use our code for your academic research, please cite the first related paper.

__Video Links__: [youtube](https://youtu.be/H89ALyWA2NI) or [bilibili](https://www.bilibili.com/video/av91537651)

<p align="center">
  <img src="misc/cp.gif" width = "533" height = "300"/>
</p>

<p align="center">
  <img src="misc/sv.gif" width = "427" height = "240"/>
  <img src="misc/fpv.gif" width = "427" height = "240"/>
</p>

## 1. Features

- Only two headers are required for usage. The first one is "am_traj.hpp", which contains a polynomial trajectory optimizer. The second one is "root_finder.hpp", which originates from a self-developed [toolkit](https://github.com/ZJU-FAST-Lab/Root-Finder).

- No complex third-party dependencies except [STL](https://en.wikipedia.org/wiki/Standard_Template_Library) and [Eigen](https://en.wikipedia.org/wiki/Eigen_(C%2B%2B_library)).

- The library is able to generate __large-scale__ trajectories with __optimal coefficients__ and __optimal time allocations__ in __real-time__, without using general purpose [NLP](https://en.wikipedia.org/wiki/Nonlinear_programming) solvers. Both the unconstrained case as well as the constrained case are considered.

- The library provides an efficient formulation for various feasibility checkers. Only algebraic operations are involved, which make our feasbility check for some high-order (>4) constraints even __faster than traditional check by closed-form solutions__ for low-order constraints (<=4). Therefore, the scalability makes it no longer painful to do feasibility check for high-order polynomial trajectories.

- The library implements an extremely fast spatial trajectory solver when time allocation is prefixed and constraints are not considered. It uses a Banded LU Factorization which is much more efficient here than a general Sparse LU Factorization. The former has linear time complexity while the latter may get stuck in its "analyzePattern". Therefore, our lib __only requires 4 (or 18) seconds__ to generate an __Unconstrained Spatial (or Spatial-Temporal) Optimal Traj with 1000000 Segments__!!!

## 2. Interface

Only three functions below are needed.

### __Constructor__

    AmTraj(double wT, double wA, double wJ, double mVr, double mAr, int mIts, double eps)

  Inputs:

    wT: Weight for the time regularization
    wA: Weight for the integrated squared norm of acceleration
    wJ: Weight for the integrated squared norm of jerk
    mVr: Maximum velocity rate
    mAr: Maximum acceleration rate
    mIts: Maximum number of iterations in optimization
    eps: Relative tolerance

  Outputs:

    An instance of AmTraj: An object used for trajectory generation

  Example:

    AmTraj amTrajOpt(1024.0, 32.0, 1.0, 7.0, 3.5, 32, 0.02);

### __Unconstrained Spatial-Temporal Optimization__

    Trajectory AmTraj::genOptimalTrajDT(const std::vector<Eigen::Vector3d> &wayPs, Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc, Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const

  Inputs:

    wayPs: Fixed waypoints for a trajectory
    iniVel: Trajectory velocity at the first waypoint
    iniAcc: Trajectory acceleration at the first waypoint
    finVel: Trajectory velocity at the last waypoint
    finAcc: Trajectory acceleration at the last waypoint

  Outputs:

    An instance of Trajectory: A trajectory with optimal coefficient matrices and optimal durations, on which constraints of maximum velocity/acceleration rate are not guaranteed
  
  Example:

    Eigen::Vector3d iV(2.0, 1.0, 0.0), fV(0.0, 0.0, 0.0)
    Eigen::Vector3d iA(0.0, 0.0, 2.0), fA(0.0, 0.0, 0.0)
    std::vector<Eigen::Vector3d> wPs;
    wPs.emplace_back(0.0, 0.0, 0.0);
    wPs.emplace_back(4.0, 2.0, 1.0);
    wPs.emplace_back(9.0, 7.0, 5.0);
    wPs.emplace_back(1.0, 3.0, 2.0);
    Trajectory traj = amTrajOpt.genOptimalTrajDT(wPs, iV, iA, fV, fA);

### __Constrained Spatial-Temporal Optimization__

    Trajectory AmTraj::genOptimalTrajDTC(const std::vector<Eigen::Vector3d> &wayPs, Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc, Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const

  Inputs:

    wayPs: Fixed waypoints for a trajectory
    iniVel: Trajectory velocity at the first waypoint
    iniAcc: Trajectory acceleration at the first waypoint
    finVel: Trajectory velocity at the last waypoint
    finAcc: Trajectory acceleration at the last waypoint

  Outputs:

    An instance of Trajectory: A trajectory with optimal coefficient matrices and best time allocation, whose maximum velocity/acceleration rate are guaranteed to satisfy constraints
  
  Example:

    Eigen::Vector3d iV(2.0, 1.0, 0.0), fV(0.0, 0.0, 0.0)
    Eigen::Vector3d iA(0.0, 0.0, 2.0), fA(0.0, 0.0, 0.0)
    std::vector<Eigen::Vector3d> wPs;
    wPs.emplace_back(0.0, 0.0, 0.0);
    wPs.emplace_back(4.0, 2.0, 1.0);
    wPs.emplace_back(9.0, 7.0, 5.0);
    wPs.emplace_back(1.0, 3.0, 2.0);
    Trajectory traj = amTrajOpt.genOptimalTrajDTC(wPs, iV, iA, fV, fA);

### __The Others__

  There are many useful functions in the header files. Use it by following their comments.

## 3. Performance

### 3.0 Performance over different feasibility checkers

<p align="center">
  <img src="misc/comparefc.png" width = "496" height = "300"/>
</p>

We compare our feasibility check method with [Mueller's recursive bound check](https://doi.org/10.1109/TRO.2015.2479878), [Burri's analytical extrema check](https://doi.org/10.1109/IROS.2015.7353622), as well as the widely used sampling-based check. In each case, 1000 trajectory pieces are randomly generated along with maximum velocity rate constraints to estimate average time consumption. As is shown in above figure, our method outperforms all other methods in computation speed because of its resolution independence and scalability with higher polynomial orders. 

### 3.1 Performance over different trajectory optimizers

<p align="center">
  <img src="misc/compareto.png" width = "700" height = "300"/>
</p>

The benchmark is done as follows: We generate a sequence of waypoints by random walk, of which the step is uniformly distributed over [-3.0m, 8.0m] for each axis. The maximum speed and acceleration rates are set to 5.0m/s and 3.5m/s^2, respectively. The derivatives on the first and the last waypoints are set to zero. The objective function is set as wT=512.0, wA=0.0, wJ=1.0. For a given number of pieces, each method is applied to 1000 sequences of waypoints. The cost is then normalized by the cost of unconstrained case of our method. All comparisons are conducted on an Intel Core i7-8700 CPU under Linux environment.

We compare our method with [Richter's method](https://doi.org/10.1177/0278364914558129) with time allocation optimized by [NLopt](https://nlopt.readthedocs.io/en/latest/) (QP + NLOPT) and [Mellinger's method](https://doi.org/10.1109/ICRA.2011.5980409) with durations properly scaled using [Liu's method](https://ieeexplore.ieee.org/document/7839930/) (BGD + Time Scaling). As is shown in the figure above, our optimizer has the fastest speed and the lowest cost when constraints are taken into consideration.

## 4. Examples

Two examples are provided in this repository.

### 4.0 Example 0

  Example 0 contains a simple comparison between the constrained case of our method against conventional method which uses heuristic time allocation along with time scaling. The lap time, maximum velocity/acceleration rate, cost function value, and visualization are provided.

<p align="center">
  <img src="misc/visual.png" width = "463" height = "300"/>
</p>

- Building

  To build this example, first make sure you have [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) properly installed. Desktop-full install is recommended. Second, clone this repository into the __src__ directory of your ROS workspace. Rename it am_traj then __move the example1 subdirectory to somewhere else__, because we only consider the building of example0 here. Use the following command to install Eigen if they are not ready.
  
      sudo apt install libeigen3-dev
  
  Now cd into your current ROS workspace

      catkin_make

- Running

      source devel/setup.bash
      rospack profile
      roslaunch example0 example0.launch

  Now you can see the example 0 is running with visualization showing in rviz.

- Parameters Setting

  It is strongly recommended that modifying parameters after roughly going through our related papers. The parameters for objective should satisfy the corresponding assumption. For example, zero weight for time is illegal. Although our method is quite efficient when initial guess is far from optimum, it is still a first-order method with no Hessian info leveraged. Technically speaking, the best convergence rate for first-order method under smooth nonconvex consumption is sublinear, which means higher precision requires much more iterations. Therefore, we suggest that the relative tol should be set as 0.02~0.001. Pratically, time allocations [1.00, 2.00, 4.00, 0.30] and [1.01, 2.02, 4.04, 0.303] won't make much difference for real-time usage.

### 4.1 Example 1

  Example 1 contains a complete global planner simulation procedure. In this example, we adopt an RRT* based waypoints selector proposed by [Richter et al.](https://doi.org/10.1177/0278364914558129). Based on the selector, we use our library to generate a optimal feasible trajectory. The whole planner is able to do global traejctory generation in real time.

<p align="center">
  <img src="misc/example1viz.gif" width = "534" height = "300"/>
</p>

- Building

  To build this example, first make sure you have ROS and Eigen properly installed as described in previous example. Moreover, some parts of the simulation require GCC 7. Therefore, if your GCC version is below the required version, the following commands help you install it without changing your default compiler.
  
      sudo apt-get install -y gcc-7 g++-7 && sleep 0.2;
      sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 && sleep 0.2;
      sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7;
  
  Second, use the following commands to install [PyGame](https://www.pygame.org/) and [OMPL](https://ompl.kavrakilab.org/).

      sudo apt install python-pygame
      cd ~/Desktop/
      wget https://github.com/ompl/ompl/archive/1.4.2.zip
      unzip 1.4.2.zip
      cd ompl-1.4.2
      mkdir build
      cd build
      cmake ..
      make
      sudo make install
  
  Third, clone this repository into the __src__ directory of your ROS workspace. Rename it am_traj. Now cd into your current ROS workspace.

      catkin_make

- Running
  
  Type following commands to start all nodes first.

      source devel/setup.bash
      rospack profile
      roslaunch example1 example1.launch
  
  Open a new terminal in your current ROS workspace and then

      source devel/setup.bash
      python src/am_traj/example1/key2joy.py

  Now you can see an rviz window and a pygame window, which are shown below.

<p align="center">
  <img src="misc/twowindows.png" width = "533" height = "300"/>
</p>

  First, we should keep the pygame window active by click it once. Then, press N on your keyboard to activate manual mode. Press W/A/S/D and Up/Down/Left/Right on your keyboard to control the white quadrotor's height/yaw and horizontal movement respectively. Please navigate the quadrotor to a safe region. After that, press M on your keyboary to enter auto mode such that global planner is activated.

  To navigate the quadrotor to any region safely, you should click once the following button on tools bar of rviz.

<p align="center">
  <img src="misc/navgbutton.png" width = "232" height = "74"/>
</p>

  Once the button is clicked, you can move your mouse arrow to any free region you want. Click the place and hold it, then a green arrow appears as is shown in the following figure. You can change the direction of the green arrow. It is worth noting that, the angle between this arrow and positive part of x-axis (red axis) decides the relative height you want the quadrotor to keep at this region. The arrow in figure below gives 60 degrees which means the desired height is 60/180*MapHeight.
<p align="center">
  <img src="misc/click.png" width = "280" height = "210"/>
</p>

  Now unhold the click, the global planner will finds you an appropriate trajectory instantly. The quadrotor then tracks the trajectory.

<p align="center">
  <img src="misc/track.png" width = "533" height = "300"/>
</p>

- Conclusion

  In most cases, the planner in this example is far faster than sampling-based kinodynamic planner, which may take several seconds to find an relative good trajectory for quadrotors. However, the RRT* used for waypoints selection can still take a relative long time (0.05 ~ 0.10 s) in comparison with our trajectory optimizer (about 0.5 ms for unconstrained case and 2.0 ms for constrained case). What's more, the waypoint selector does not consider the dynamics. That's to say, it is possible that wierd waypoints may be selected, when initial speed is large or obstacles are too dense. We claim that this is the __inherent property__ of the adopted waypoint selector.

## 5. Misc

- The library implements a specific order of polynomial trajectory optimization. If higher or lower order is required, it is recommended to refer to our related papers, where the framework used in the library are described and analyzed more flexibly.

- The self-developed toolkit "root_finder.hpp" used in this library is an portable substitution to the suggested modern univariate polynomial real-roots solver in our related paper. It is more efficient in our scene than the wide used [__TOMS493: Jenkins–Traub Algorithm__](https://en.wikipedia.org/wiki/Jenkins%E2%80%93Traub_algorithm). For detailed performance, please refer to the corresponding [repository](https://github.com/ZJU-FAST-Lab/Root-Finder). If efficiency is much more important to you, we suggest an closed-source solver [RS-ANewDsc](http://anewdsc.mpi-inf.mpg.de/) from Max Planck Institute for Informatics.

- Currently, we only implement feasibilty checker for dynamic constraints. If only your constraints can be expressed in multivariate polynomial of trajectory or its higher derivatives. You can transform it into compatible format and utilize some functions in this library to accomplish simple yet solid check for it.

- The library will support optimizing waypoints in the future, following the general framework in our paper. However, the spatial constraints depend on specific planning method. Therefore, feel free to modify all codes in the library if you need to optimize waypoints.

- Thanks [Helen Oleynikova](https://github.com/helenol) et al. a lot for their [open-source project](https://github.com/ethz-asl/mav_trajectory_generation). In our benchmark, some modules in compared methods are modified from their projects.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

## 7. Maintaince
For any technical issues, please contact Zhepei WANG (<wangzhepei@live.com>) or Fei GAO (<fgaoaa@zju.edu.cn>).

For commercial inquiries, please contact Fei GAO (<fgaoaa@zju.edu.cn>).
