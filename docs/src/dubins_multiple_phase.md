````julia
# include("../src/DirectOptimalControl.jl")
# import .DirectOptimalControl as DOC

import DirectOptimalControl as DOC
import Ipopt
using JuMP
````

Common parameters

````julia
vm = 1.0   # Max forward speed
um = 1.0   # Max turning speed
ns = 3     # Number of states
nu = 2     # Number of inputs
n = 20    # Time steps
````

System dynamics

````julia
function dyn(x, u, t, p)
    return [u[2] * cos(x[3]), u[2] * sin(x[3]), u[1]]
end
````

Objective Function
Running cost

````julia
function L(x, u, t, p)
    return 1.0
end

function phi(xf, uf, tf, p)
    return 0.0
end

function integralfun(x, u, t, p)
    return nothing
end

function pathfun(x, u, t, p)
    return nothing
end

OC = DOC.OCP()
OC.tol = 1e-5
OC.mesh_iter_max = 15
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "linear_solver", "mumps")
set_attribute(OC.model, "print_level", 0)
# set_attribute(OC.model, "max_iter", 500)
set_attribute(OC.model, "tol", 1e-6)
````

Phase 1

````julia
ph1 = DOC.PH(OC)
ph1.L = L
ph1.phi = phi
ph1.dyn = dyn
ph1.integralfun = integralfun
ph1.collocation_method = "hermite-simpson"
ph1.scale_flag = true

ph1.n = n
ph1.ns = ns
ph1.nu = nu
ph1.nq = 0
ph1.nk = 0
````

Auxillary parameters

````julia
ph1.p = (k1 = 1.0, k2 = 2.0)

ph1.limits.ll.u = [-1.0, 0.75*vm]
ph1.limits.ul.u = [1.0, vm]
ph1.limits.ll.x = [-10.0, -10.0, -10.0]
ph1.limits.ul.x = [10.0, 10.0, 10.0]
ph1.limits.ll.xf = [5.0, 5.0, π / 2 + π / 4]
ph1.limits.ul.xf = [5.0, 5.0, π / 2 + π / 4]
ph1.limits.ll.xi = [0, 0, -π / 2]
ph1.limits.ul.xi = [0, 0, -π / 2]
ph1.limits.ll.ti = 0.0
ph1.limits.ul.ti = 0.0
ph1.limits.ll.tf = 5.0
ph1.limits.ul.tf = 20.0
ph1.limits.ll.dt = 5.0
ph1.limits.ul.dt = 20.0
ph1.limits.ll.k = []
ph1.limits.ul.k = []
````

ph.set_initial_values can take two values: "Auto" and "User"
Set initial values

````julia
ph1.set_initial_vals = "Auto"
````

Phase 2

````julia
ph2 = DOC.PH(OC)
ph2.L = L
ph2.phi = phi
ph2.dyn = dyn
ph2.integralfun = integralfun
ph2.collocation_method = "hermite-simpson"
ph2.scale_flag = true

ph2.n = n
ph2.ns = ns
ph2.nu = nu
ph2.nq = 0
ph2.nk = 0

ph2.p = (k1 = 1.0, k2 = 2.0)

ph2.limits.ll.u = [-1.0, 0.75*vm]
ph2.limits.ul.u = [1.0, vm]
ph2.limits.ll.x = [-10.0, -10.0, -10.0]
ph2.limits.ul.x = [10.0, 10.0, 10.0]
ph2.limits.ll.xf =  [-5, 5, 0.0]
ph2.limits.ul.xf =  [-5, 5, 0.0]
ph2.limits.ll.xi = ph1.limits.ll.xf
ph2.limits.ul.xi = ph1.limits.ul.xf
ph2.limits.ll.ti = ph1.limits.ll.tf
ph2.limits.ul.ti = ph1.limits.ul.tf
ph2.limits.ll.dt = 5.0
ph2.limits.ul.dt = 20.0
ph2.limits.ll.tf = ph2.limits.ll.ti + ph2.limits.ll.dt
ph2.limits.ul.tf = ph2.limits.ul.ti + ph2.limits.ul.dt
````

Phase 3

````julia
ph3 = DOC.PH(OC)
ph3.L = L
ph3.phi = phi
ph3.dyn = dyn
ph3.integralfun = integralfun

ph3.n = n
ph3.tau = range(start = 0, stop = 1, length = ph3.n)
ph3.ns = ns
ph3.nu = nu
ph3.p = (k1 = 1.0, k2 = 2.0)

ph3.limits.ll.u = [-1.0, 0.75*vm]
ph3.limits.ul.u = [1.0, vm]
ph3.limits.ll.x = [-10.0, -10.0, -10.0]
ph3.limits.ul.x = [10.0, 10.0, 10.0]
ph3.limits.ll.xf =  [-5, -5, 0.0]
ph3.limits.ul.xf =  [-5, -5, 0.0]
ph3.limits.ll.xi = ph2.limits.ll.xf
ph3.limits.ul.xi = ph2.limits.ul.xf
ph3.limits.ll.ti = ph2.limits.ll.tf
ph3.limits.ul.ti = ph2.limits.ul.tf
ph3.limits.ll.dt = 7.0
ph3.limits.ul.dt = 20.0
ph3.limits.ll.tf = ph3.limits.ll.ti + ph3.limits.ll.dt
ph3.limits.ul.tf = ph3.limits.ul.ti + ph3.limits.ul.dt
ph3.collocation_method = "hermite-simpson"
ph3.scale_flag = true
````

Phase 4

````julia
ph4 = DOC.PH(OC)
ph4.L = L
ph4.phi = phi
ph4.dyn = dyn
ph4.integralfun = integralfun

ph4.n = n
ph4.tau = range(start = 0, stop = 1, length = ph4.n)
ph4.ns = ns
ph4.nu = nu
ph4.p = (k1 = 1.0, k2 = 2.0)

ph4.limits.ll.u = [-1.0, 0.75*vm]
ph4.limits.ul.u = [1.0, vm]
ph4.limits.ll.x = [-10.0, -10.0, -10.0]
ph4.limits.ul.x = [10.0, 10.0, 10.0]
ph4.limits.ll.xf =  ph1.limits.ll.xi
ph4.limits.ul.xf =  ph1.limits.ul.xi
ph4.limits.ll.xi = ph3.limits.ll.xf
ph4.limits.ul.xi = ph3.limits.ul.xf
ph4.limits.ll.ti = ph3.limits.ll.tf
ph4.limits.ul.ti = ph3.limits.ul.tf
ph4.limits.ll.dt = 7.0
ph4.limits.ul.dt = 20.0
ph4.limits.ll.tf = ph4.limits.ll.ti + ph4.limits.ll.dt
ph4.limits.ul.tf = ph4.limits.ul.ti + ph4.limits.ul.dt
ph4.collocation_method = "hermite-simpson"
ph4.scale_flag = true
````

Add the boundary constraints

````julia
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    v4 = ph[2].ti - ph[1].tf
    v7 = ph[3].ti - ph[2].tf
    v10 = ph[4].ti - ph[3].tf

    return [v4, v7, v10]
end

OC.psi = psi
OC.npsi = 3
OC.set_obj_lim = true
OC.obj_llim = 0.0
OC.obj_ulim = 43
OC.psi_llim = [0.0, 0.0, 0.0]
OC.psi_ulim = [0.0, 0.0, 0.0]

DOC.setup_mpocp(OC)
DOC.solve_mpocp(OC)
# DOC.solve(OC)
````

Solve for the control and state

````julia
solution_summary(OC.model)
````

Display results

````julia
println("Min time: ", objective_value(OC.model))


# using GLMakie
# f1, ax1, l1 = lines(value.(ph1.x[1, :]), value.(ph1.x[2, :]))
# l2 = lines!(ax1, value.(ph2.x[1, :]), value.(ph2.x[2, :]))
# l3 = lines!(ax1, value.(ph3.x[1, :]), value.(ph3.x[2, :]))
# l4 = lines!(ax1, value.(ph4.x[1, :]), value.(ph4.x[2, :]))
# ax1.autolimitaspect = 1.0
# # f1

# f2, ax2, l21 = lines(value.(ph1.t), value.(ph1.u[1, :]))
# l22 = lines!(ax2, value.(ph2.t), value.(ph2.u[1, :]))
# l23 = lines!(ax2, value.(ph3.t), value.(ph3.u[1, :]))
# l24 = lines!(ax2, value.(ph4.t), value.(ph4.u[1, :]))
# f2

# f3, ax3, l31 = lines(value.(ph1.t), value.(ph1.u[2, :]))
# l32 = lines!(ax3, value.(ph2.t), value.(ph2.u[2, :]))
# l33 = lines!(ax3, value.(ph3.t), value.(ph3.u[2, :]))
# l34 = lines!(ax3, value.(ph4.t), value.(ph4.u[2, :]))
# f3


# f4, ax4, l4 = lines(value.(ph1.xinit[1, :]), value.(ph1.xinit[2, :]))
# l2 = lines!(ax4, value.(ph2.xinit[1, :]), value.(ph2.xinit[2, :]))
# l3 = lines!(ax4, value.(ph3.xinit[1, :]), value.(ph3.xinit[2, :]))
# l4 = lines!(ax4, value.(ph4.xinit[1, :]), value.(ph4.xinit[2, :]))
# ax4.autolimitaspect = 1.0
# f4
````

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

