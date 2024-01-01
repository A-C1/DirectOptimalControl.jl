````julia
include("../src/DirectOptimalControl.jl")
import .DirectOptimalControl as DOC

import Ipopt
using GLMakie
using JuMP


OC = DOC.OCP()
OC.tol = 1e-12
OC.mesh_iter_max = 5
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
````

set_attribute(OC.model, "print_level", 0)
set_attribute(OC.model, "max_iter", 500)
set_attribute(OC.model, "tol", 1e-4)

Parameters:

````julia
zeta = 0.084
b = 5.85
d = 0.00873
G = 0.15
mu = 0.02
a = 75
A = 15
````

````julia
pMax = ((b-mu)/d)^(3/2)
pMin = 0.1
qMax = pMax
qMin = pMin
yMax = A
yMin = 0
uMax = a
uMin = 0
t0Max = 0
t0Min = 0
tfMax = 5
tfMin = 0.1
p0 = pMax/2
q0 = qMax/4
y0 = 0

p = (zeta = zeta, b = b, d = d, G = G, mu = mu)

ns = 2
nu = 1
n = 20
nq = 1
````

System dynamics

````julia
pf(t, a, b) = exp(-b*(t-a)^2)
function dyn(x, u, t, p)
    x1 = -p.zeta*x[1]*log(x[1]/x[2])
    x2 = x[2]*(p.b - p.mu -p.d*(x[1])^(2/3) - p.G*u[1] )

    return [x1, x2]
end
````

Objective Function
Running cost

````julia
function L(x, u, t, p)
    return 0.0
end

function phi(xf, uf, tf, p)
    return xf[1]
end
````

Integration function

````julia
function integralfun(x, u, t, p)
    return [u[1]]
end
````

Phase 1

````julia
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun
ph.collocation_method = "trapezoidal"

ph.n = n
````

ph.tau = range(start = 0, stop = 1, length = ph.n)

````julia
ph.ns = ns
ph.nu = nu
ph.nq = nq
ph.np = 0
ph.p = p

ph.limits.ll.u = [uMin]
ph.limits.ul.u = [uMax]
ph.limits.ll.x = [pMin, qMin]
ph.limits.ul.x = [pMax, qMax]
ph.limits.ll.xf = [pMin, qMin]
ph.limits.ul.xf = [pMax, qMax]
ph.limits.ll.xi = [p0, q0]
ph.limits.ul.xi = [p0, q0]
ph.limits.ll.ti = t0Min
ph.limits.ul.ti = t0Max
ph.limits.ll.tf = tfMin
ph.limits.ul.tf = tfMax
ph.limits.ll.dt = tfMin-t0Min
ph.limits.ul.dt = tfMax-t0Max
ph.limits.ll.integral = [-1000.0]
ph.limits.ul.integral = [A]
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
OC.npsi = 0
````

Add an option to give intial condition
c = NOC.add_phase(ph1, OC)

````julia
DOC.setup_mpocp(OC)
DOC.solve(OC)
````

Solve for the control and state

````julia
solution_summary(OC.model)
````

Display results

````julia
println("Min time: ", objective_value(OC.model))
````

x, u, dt, oc = NOC.solve(OC)

````julia
fx1, ax1, l1 = scatter(value.(ph.t), value.(ph.x[1,:]))
fx2, ax2, l2 = scatter(value.(ph.t), value.(ph.x[2,:]))
fu1 = scatter(value.(ph.t), value.(ph.u[1,:]))
fx1
````

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

