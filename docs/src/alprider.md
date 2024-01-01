````julia
import DirectOptimalControl as DOC
import Ipopt
using GLMakie
using JuMP

OC = DOC.OCP()
OC.tol = 1e-4
OC.mesh_iter_max = 11
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
````

# set_attribute(OC.model, "print_level", 0)
# set_attribute(OC.model, "max_iter", 500)

````julia
set_attribute(OC.model, "tol", 1e-5)


x0 = [2, 1, 2, 1]
xf = [2, 3, 1, -2]
t0 = 0
tf = 20

p = (x0 = x0, xf = xf, t0 = t0, tf = tf)

ns = 4
nu = 2
n = 100
np = 1
````

System dynamics

````julia
pf(t, a, b) = exp(-b*(t-a)^2)
function dyn(x, u, t, p)
    x1 = -10x[1] + u[1] + u[2]
    x2 = -2x[2] + u[1] + 2u[2]
    x3 = -3x[3] + 5x[4] + u[1] - u[2]
    x4 = 5x[3] - 3x[4] + u[1] + 3u[2]

    return [x1, x2, x3, x4]
end
````

Objective Function
Running cost

````julia
function L(x, u, t, p)
    return 100*(x[1]^2 + x[2]^2 + x[3]^2 + x[4]^4) + 0.01(u[1]^2 + u[2]^2)
end

function phi(xf, uf, tf, p)
    return 0.0
end

function integralfun(x, u, t, p)
    return nothing
end
````

Path function

````julia
function pathfun(x, u, t, p)
    v1 = x[1]^2 + x[2]^2 + x[3]^2 + x[4]^2 - (3pf(t, 3.0, 12.0) + 3pf(t, 6.0, 10.0)
                                             +3pf(t, 10.0, 6.0) + 8pf(t, 15.0, 4.0) + 0.01 )
    return [v1]
end
````

Phase 1

````julia
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun
ph.pathfun = pathfun
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true

ph.n = n
ph.ns = ns
ph.nu = nu
ph.np = np
ph.nq = 0
ph.p = p

ph.limits.ll.u = -1e3*[1.0, 1.0]
ph.limits.ul.u = 1e3*[1.0, 1.0]
ph.limits.ll.x = -1e3*[1.0, 1.0, 1.0, 1.0]
ph.limits.ul.x = 1e3*[1.0, 1.0, 1.0, 1.0]
ph.limits.ll.xf = xf
ph.limits.ul.xf = xf
ph.limits.ll.xi = x0
ph.limits.ul.xi = x0
ph.limits.ll.ti = t0
ph.limits.ul.ti = t0
ph.limits.ll.tf = tf
ph.limits.ul.tf = tf
ph.limits.ll.dt = tf-t0
ph.limits.ul.dt = tf-t0
ph.limits.ll.path = [0.0]
ph.limits.ul.path = [10000]

ph.set_initial_vals = "Auto"
ph.scale_flag = false
````

Add the boundary constraints

````julia
function psi(ocp::DOC.OCP)
    (;ph) = ocp
````

return [v2;]

````julia
    return nothing
end

OC.psi = psi
OC.psi_llim = []
OC.psi_ulim = []
OC.set_obj_lim = false
OC.obj_llim = 1e6
OC.obj_ulim = -1e6
````

c = NOC.add_phase(ph1, OC)

````julia
DOC.setup_mpocp(OC)
````

DOC.solve(OC)
# Solve for the control and state
solution_summary(OC.model)

# Display results
println("Min time: ", objective_value(OC.model))

# x, u, dt, oc = NOC.solve(OC)

f1, ax1, l1 = lines(value.(ph.t), value.(ph.x[1,:]))
f2, ax2, l2 = lines(value.(ph.t), value.(ph.x[2,:]))
f3, ax3, l3 = lines(value.(ph.t), value.(ph.x[3,:]))
f4, ax4, l4 = lines(value.(ph.t), value.(ph.x[4,:]))
fu1 = lines(value.(ph.t), value.(ph.u[1,:]))
fu2 = lines(value.(ph.t), value.(ph.u[1,:]))
display(f1)
display(f2)
display(f3)

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

