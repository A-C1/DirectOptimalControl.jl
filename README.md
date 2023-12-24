# A package to solve multiple-phase optimal control problems

An p-phase optimal control problem can be stated in the following
general form. Determine the state, ${\bf x^{(p)}}(t) \in \mathbb{R}^{n_{z}^{(p)}}$
control ${\bf u^{(p)}}(t) \in \mathbb{R}^{n_{u}^{(p)}}$, initial time
$t_{0}^{(p)}\in\mathbb{R}$, final time $t_{f}^{(p)}\in\mathbb{R}$,
integrals, ${\bf q}^{(p)}\in\mathbb{R}^{n_{q}^{(p)}}$, local parameters ${\bf k_{p}} \in \mathbb{R}^{n_{k}^{(p)}}$ 
in each phase $p\in[1,...,P]$,
and the static parameters, ${\bf k_{g}}\in\mathbb{R}^{n_{kg}}$, that
minimize the cost functional 
```math
J=\sum_{p=1}^{n}J_{p}
```
where,

```math
J_{p}=\intop_{t_{i}^{p}}^{t_{f}^{p}}L^{p}({\bf x^{(p)}}(t),{\bf u^{(p)}}(t),t^{(p)},{\bf k},{\bf ap})\ dt^{p}+\phi(x^{(p)}(t_{f}^{p}),u^{(p)}(t_{f}^{p}),t_{f}^{(p)},{\bf k},{\bf ap})
```

> Note: ${\bf ap}$ contains all the auxillary parameters used to define various functions, 
> and ${\bf k} =[{\bf k_{p}}; {\bf k_{g}}]$ is a stacked vector of phase and global
> parameters which can be optimized. 

The cost funtional $J$ must be minimized subject to the following
constraints in each phase $p$:
### Path constraints: 
Path constraints are the constraints which
the states and controls must obey at each instant $t^{(p)}$ .
```math
{\bf pf_{l}}^{(p)}\leq{\bf pathfun}^{(p)}({\bf x^{p}}(t),{\bf u^{p}}(t),t^{(p)},{\bf k},{\bf ap})\in\mathbb{R}^{n_{pf}^{k}}\leq{\bf pf_{u}}^{(p)}\ \forall\ t^{(p)}\in[t_{i}^{(p)},t_{f}^{(p)}]
```
### Integral constraints: 
Define: 
```math
{\bf IF}^{(p)}=\intop_{t_{i}^{p}}^{t_{f}^{p}}{\bf integralfun}^{(p)}({\bf x^{(p)}}(t),{\bf u^{(p)}}(t),t^{(p)},{\bf k},{\bf ap})\ dt^{(p)}
```
```math
{\bf if_{l}}^{(p)}\leq{\bf pathfun}({\bf x^{p}}(t),{\bf u^{p}}(t),t^{(p)},{\bf k},{\bf ap})\in\mathbb{R}^{n_{pf}^{k}}\leq{\bf if_{u}}^{(p)}
```
### Event constraints:
Events in time are what cause a change
of phase. Event constraints represent linkages between initial times,
states, inputs and final times, states, inputs between phases.
```math
{\bf psi_{u}}^{(p)}\leq{\bf psi}(t_{i}^{(1)},t_{f}^{(1)},{\bf x_{f}}^{(1)},{\bf u_{f}}^{(1)},t_{i}^{(2)},t_{f}^{(2)},{\bf x_{f}}^{(2)},{\bf u_{f}}^{(2)},\ldots t_{i}^{(P)},t_{f}^{(P)},{\bf x_{f}}^{(P)},{\bf u_{f}}^{(P)})\in\mathbb{R}^{n_{psi}^{(p)}}\leq{\bf psi_{u}}^{(p)}
```



## Features
1. Multiple phases
2. Scaling
3. Mesh recomputation
4. Flexibility in formulating problems

## Installation
> add https://github.com/A-C1/DirectOptimalControl.jl
