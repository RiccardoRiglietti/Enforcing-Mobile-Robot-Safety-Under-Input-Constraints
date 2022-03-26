# Enforcing Mobile Robot Safety Under Input Constraints

## Introduction

Many cyber-physical systems are safety critical, namely, they require guarantees that safety con-
straints are not violated during operation. Safety is often modeled by defining a safe subset of the
state space for a given system, within which the state trajectories must always evolve. Recently,
methods such as Control Barrier Functions (CBFs), have become increasingly popular as a means
of constructing and verifying such controllers. Quadratic Programs (QPs) enforcing CBF con-
ditions have been used for synthesizing low-level controllers to ensure that system trajectories
remain within the safe set [1].

In this project, we have addressed the issue of ensuring safety while having a good performance
for a dynamical system subjected to input constraints solving the Adaptive Cruise Control (ACC)
problem. Its formulation considers two vehicles, a leader and a (controlled) follower, where the
second one would like to reach its maximum speed while remaining safe. The latter concept refers
to the fact that the following car maintains always a certain distance from the leader avoiding
collisions. In this framework the velocity profile of the vehicle in front is known, hence it can be
included in the model formulation as a state if it’s not constant.
Input limitations affect the solution in such a way that "naive" methods are no more able to
always guarantee safety by letting the following vehicle enter in the unsafe region suggesting that
a more conservative method is needed.

For designing a succeeding control law we have used an extension of the standard Control Bar-
rier Functions theory building iteratively a suitable CBF applying the procedure reported in [2].
The result of this algorithm is an Input Constraint Control Barrier Function (ICCBF) that, by
construction, is able to maintain safety under input constraints.
The standard method works defining safety as Hard Constraints, throught the CBF, and per-
formance as Soft Constraint, managed with a Control Lyapunov Function (CLF). The results
obtained with the latter methodology do not take into account the presence of limitations in the
actuation system, since the procedure consists in clamping the control after its computation, and
this reflects in the fact that the controlled vehicle starts to decelerate too late, exceeding from
the boundary of the safe region.

Considering the same safety function h(x) for both methods, we have discovered the superiority
of ICCBF over CLF-CBF. Nevertheless [3] shows that, by changing on the fly the safety func-
tion, also CLF-CBF is able to maintain stability remaining on the boundary of the safe set. The
authors solve an optimization problem in order to define the optimal CBF to be used in each
situation and they also provide an explication of when each function must be used depending
both on the instantaneous velocity of the vehicles and on the maximal fixed deceleration that
they can apply by breaking as hard as possible.

The paper is structured as follows: section 2 presents some preliminaries on CBFs taking into
account also the stability problem, CLF-CBF, and the presence of input constraints, ICCBF.
Section 3 presents the definition of the problem of the Adaptive Cruise Control and the synthe-
sis of the respective feedback controllers for ensuring safety with the corresponding simulation
results.

## Results

In this project we have analysed the framework for the control of safety-critical systems through
the use of the Control Barrier Functions (CBF) in the context of the Adaptive Cruise Control
(ACC) problem considering different scenarios: changing the method used to derive the controller
and considering alternative velocity profiles of the leading vehicle. In the context of affine control
systems, this naturally yields control barrier functions (CBFs) with a large set of available control
inputs that renders a set C forward invariant. CBFs are expressed as affine inequality constraints,
in the control input, that imply forward invariance of the set, hence safety.

We have first looked at the problem of ACC in the case of the presence of input constraints,
which are explicitly included in the construction of control barrier functions (ICCBF) in order
to guarantee that safety is maintained with an input constrained controller.
We have analysed how the shape of the safe region changes by varying the parameters that define
the class-K functions α_i , finding out that lowering these values reflects in a smaller safe region,
while enlarging them too much corresponds to not finding a proper ICCBF accordingly to the
choices done in our simulations, i.e. the number N and the functions α_i . Furthermore, we have
tested the validity of this type of CBFs concluding that, once we tuned properly the parameters
of α_i , safety is guaranteed.

We have also changed the maximum allowed velocity for the following vehicle, that reflects in a
different desired controller u d (x), and we have shown that the ICCBF controller still guarantees
safety and the new speed limit is never violated, whereas the CLF-CBF still does not always
respect the hard constraint.

In addition, by looking at the case of a variable leading velocity, we have augmented the dynamical
system taking into account also the leading speed evolution. With the new system the acceleration
of the vehicle in front shows up in the definition of the ICCBF and we have noticed that the
follower accelerates and decelerates thanks to its a-priori knowledge of the leading velocity profile,
in fact it moves in phase with the leader.

This solution has been compared with the framework in which we unify safety conditions with
control objectives (expressed as Control Lyapunov Functions), by strictly enforcing the safety
constraint and relaxing the control objective defining a proper quadratic program problem. Then,
in the case of the CLF-CBF, we have considered the QP problem defined in terms of optimal
control barrier functions, chosen based on the velocities of the two vehicles. Finally, we have
done a comparison among all the solutions obtained with the different frameworks in the ACC
problem by analysing differences in term of control effort and safety.

## Authors

Riccardo Riglietti, Lorenzo Govoni, Francesco D'Orazio

## References

[1] Aaron D Ames, Samuel Coogan, Magnus Egerstedt, Gennaro Notomista, Koushil Sreenath,
and Paulo Tabuada. Control barrier functions: Theory and applications. In 2019 18th Euro-
pean control conference (ECC), pages 3420–3431. IEEE, 2019.

[2] Devansh Agrawal and Dimitra Panagou. Safe control synthesis via input constrained control
barrier functions. arXiv preprint arXiv:2104.01704, 2021.

[3] Aaron D Ames, Xiangru Xu, Jessy W Grizzle, and Paulo Tabuada. Control barrier func-
tion based quadratic programs for safety critical systems. IEEE Transactions on Automatic
Control, 62(8):3861–3876, 2016.

[4] Katja Vogel. A comparison of headway and time to collision as safety indicators. Accident
analysis & prevention, 35(3):427–433, 2003.
