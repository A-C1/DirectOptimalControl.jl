````julia
# include("../src/DirectOptimalControl.jl")
# import .DirectOptimalControl as DOC

import DirectOptimalControl as DOC
import Ipopt
using JuMP
using Interpolations
````

Common parameters

````julia
vm = 2.0   # Max forward speed
um = 1.0   # Max turning speed
ns = 3     # Number of states
nu = 2     # Number of inputs
n = 100    # Time steps

state_e0 = [0.0, 7.0, -π/2]
N = 200
l = 0.05
δt = 0.1
timeE = LinRange(0, N * δt, N)

se = zeros(3, length(timeE))
se[:,1] = state_e0
for i = 1:(length(timeE)-1)
    se[:, i+1] = se[:, i] + [0.05, 0.05, 0.05]
end
x_e = se[1,:]
y_e = se[2,:]
th_e = se[3,:]


function xval(tf)
    #global δt, timeE, x_e, n
    interp = LinearInterpolation(timeE, x_e, extrapolation_bc = x_e[N])
    return interp(tf)
end

function yval(tf)
    #global δt, timeE, y_e, n
    interp = LinearInterpolation(timeE, y_e, extrapolation_bc=y_e[N])
    return interp(tf)
end
````

System dynamics

````julia
function dyn(x, u, t, p)
````

vm = p.vm

````julia
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
OC.mesh_iter_max = 5
OC.objective_sense = "Min"
set_optimizer(OC.model, Ipopt.Optimizer)
set_attribute(OC.model, "linear_solver", "mumps")
set_attribute(OC.model, "print_level", 0)
# set_attribute(OC.model, "max_iter", 500)
# set_attribute(OC.model, "tol", 1e-6)

@operator(OC.model, op_xval, 1, xval)
@operator(OC.model, op_yval, 1, yval)
````

#### Phase 1

````julia
ph = DOC.PH(OC)
ph.L = L
ph.phi = phi
ph.dyn = dyn
ph.integralfun = integralfun
ph.collocation_method = "hermite-simpson"
ph.scale_flag = true

ph.n = n
ph.ns = ns
ph.nu = nu
ph.nq = 0
ph.nk = 0
````

#### Auxillary parameters

````julia
ph.p = (k1 = 1.0, k2 = 2.0)

ph.limits.ll.u = [-1.0, 0.75*vm]
ph.limits.ul.u = [1.0, vm]
ph.limits.ll.x = [-30.0, -30.0, -10.0]
ph.limits.ul.x = [30.0, 30.0, 10.0]
ph.limits.ll.xf = [-30.0, -30.0, -10.0]
ph.limits.ul.xf = [30.0, 30.0, 10.0]
ph.limits.ll.xi = [0, 0, -π / 2]
ph.limits.ul.xi = [0, 0, -π / 2]
ph.limits.ll.ti = 0.0
ph.limits.ul.ti = 0.0
ph.limits.ll.tf = 5.0
ph.limits.ul.tf = 15.0
ph.limits.ll.dt = 5.0
ph.limits.ul.dt = 15.0
ph.limits.ll.k = []
ph.limits.ul.k = []
````

ph.set_initial_values can take two values: "Auto" and "User"
Set initial values

````julia
ph.set_initial_vals = "Auto"
````

------------------------------------------------------

Add the boundary constraints

````julia
function psi(ocp::DOC.OCP)
    (;ph) = ocp

    # # Phase 1
    # v1 = ph[1].ti - 0.0
    # v2 = ph[1].xf - ph[1].p.xfref
    # v3 = ph[1].xi - ph[1].p.x0

    # # Phase 2
    v4 = ph[2].ti - ph[1].tf
    # v5 = ph[2].xi - ph[1].xf
    # v6 = ph[2].xf - [-5, 5, 0.0]

    # # Phase 3
    v7 = ph[3].ti - ph[2].tf
    # v8 = ph[3].xi - ph[2].xf
    # v9 = ph[3].xf - [-5, -5, 0.0]

    # # Phase 4
    v10 = ph[4].ti - ph[3].tf
    # v11 = ph[4].xi - ph[3].xf
    # v12 = ph[4].xf - ph[4].p.x0

    # return [v1; v2; v3; v4; v5; v6; v7; v8; v9; v10; v11; v12]#; v13; v14; v15]
    # return [v4, v7, v10]
    return nothing
end

OC.psi = psi
OC.npsi = 0
OC.set_obj_lim = true
OC.obj_llim = 5.0
OC.obj_ulim = 17.0
OC.psi_llim = []
OC.psi_ulim = []
DOC.setup_mpocp(OC)

# Final conditions
function additional_constraints(ph, OC)
    @constraint(OC.model, (ph.xf[1] - op_xval(ph.tf))^2 + (ph.xf[2] - op_yval(ph.tf))^2 <= l * l)
end
OC.additional_constraints = additional_constraints
````

This function needs to be called for single use

````julia
OC.additional_constraints(ph, OC)
DOC.solve_mpocp(OC)
````

DOC.solve(OC)
Solve for the control and state

````julia
solution_summary(OC.model)
````

Display results

````julia
println("Min time: ", objective_value(OC.model))


# using GLMakie
# f1, ax1, l1 = lines(value.(ph.x[1, :]), value.(ph.x[2, :]))
# lines!(ax1, x_e, y_e)
# ax1.autolimitaspect = 1.0
# # f1
# f2, ax2, l21 = lines(value.(ph.t), value.(ph.u[1, :]))
# f2
# f3, ax3, l31 = lines(value.(ph.t), value.(ph.u[2, :]))
# f3
````

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

