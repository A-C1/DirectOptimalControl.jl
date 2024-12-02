include("../src/DirectOptimalControl.jl")
import .DirectOptimalControl as DOC
using OSQP
using COSMO

# Create OCP
d_ocp = DOC.OcpDims(0, 0)
function psi(ph::Vector{DOC.Phase})
    return nothing
end
limits_ocp = DOC.OcpLimits()
limits_ocp.ll.kg = []
limits_ocp.ul.kg = []
limits_ocp.ll.psi = []
limits_ocp.ul.psi = []
cp_ocp = DOC.OcpConfigParams(DOC.Ipopt.Optimizer)

ocp = DOC.OCP(d_ocp, psi, limits_ocp, cp_ocp)

# Define dimensions of optimal control problem
d = DOC.Dims()
d.n = 10
d.ns = 2
d.nu = 1
d.nq = 0
d.np = 0
d.nk = 2


# Define functions for optimal control problem
function f(x, u, t, p)
    x1n = x[2]
    x2n = p.k[1]*x[1] + p.k[2]*x[2]

    return [x1n, x2n]
end

function l(x, u, t, p)
    return 1.0
end

function phi(xn, un, t, p)
    return 0.0
end

function c(x, u, t, p)
    # return [1.0]
    return nothing
end

function h(x, u, t, p) 
    # return [(x[1] - 3)^2 - 9]
    return nothing
end

funcs = DOC.Funs(l, phi, f, h, c)

# Define limits
limits = DOC.Limits()

limits.ll.ti = 0.0
limits.ul.ti = 0.0
limits.ll.dt = 0.0 
limits.ul.dt = 40.0
limits.ll.tf = 0.0
limits.ul.tf = 40.0
limits.ll.xf = [0.0, 0.0]
limits.ul.xf = [0.0, 0.0]
limits.ll.xi = [10.0, 0.0]
limits.ul.xi = [10.0, 0.0]
limits.ll.x = [-20.0, -20.0]
limits.ul.x = [20.0, 20.0]
limits.ll.u = [-1.0]
limits.ul.u = [1.0]
limits.ll.y = []
limits.ul.y = []
limits.ll.c = []
limits.ul.c = []
limits.ll.k = [-10.0, -10.0]
limits.ul.k = [10.0, 10.0]

p = (v = 1.0, umax = 1.0)

cpp = DOC.ConfigParams("trapezoidal", true, true, "nonlinear")
ph = DOC.Phase(ocp, p, funcs, d, limits, cpp)

DOC.solve(ocp)

# DOC.add_phase(ph, ocp)
# DOC.add_objective(ocp)
# DOC.optimize!(ocp.model)

# for ph in ocp.ph
#     DOC.store_results!(ph.val, ph.uvars)
#     DOC.compute_new_mesh_betts(ph)
# end

# DOC.update_ocp!(ocp)
# DOC.update_phase!(ph, ocp.model)
# push!(ocp.ph, ph)
# DOC.add_phase(ph, ocp)
# DOC.add_objective(ocp)
# DOC.optimize!(ocp.model)
# DOC.store_results!(ph.val, ph.uvars)

using GLMakie
fig = Figure()
ax = Axis(fig[1,1], autolimitaspect = 1)
lines!(ax, ocp.ph[1].val.x[1,:], ocp.ph[1].val.x[2,:], color = :blue)
# arc!(ax, Point2f(3.0, 3.0), 3.0, 0.0, 2π; color =:red)
fig
 
