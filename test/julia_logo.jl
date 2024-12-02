include("../src/DirectOptimalControl.jl")
import .DirectOptimalControl as DOC

# Create OCP
d_ocp = DOC.OcpDims(0, 8)
# Add Boundary conditions and custom constraints
function psi(ph::Vector{DOC.Phase})
    return [ph[1].uvars.tf - ph[2].uvars.ti;
            ph[1].uvars.xf - ph[2].uvars.xi;
            ph[2].uvars.tf - ph[3].uvars.ti;
            ph[2].uvars.xf - ph[3].uvars.xi;
            ]
end
limits_ocp = DOC.OcpLimits()
limits_ocp.ll.kg = []
limits_ocp.ul.kg = []
limits_ocp.ll.psi = [0.0; zeros(3); zeros(3); 0.0]
limits_ocp.ul.psi = [0.0; zeros(3); zeros(3); 0.0]
cp_ocp = DOC.OcpConfigParams(DOC.Ipopt.Optimizer)

ocp = DOC.OCP(d_ocp, psi, limits_ocp, cp_ocp)

# Define dimensions of optimal control problem
d = DOC.Dims()
d.n = 30
d.ns = 3
d.nu = 1
d.nq = 2
d.np = 3
d.nk = 0


# Define functions for optimal control problem
p = (v = 1.0, umax = 1.0)

function f(x, u, t, p)
    xn = p.v*cos(x[3])
    yn = p.v*sin(x[3])
    thetan = u[1]

    return [xn, yn, thetan]
end

function l(x, u, t, p)
    return 1.0
end

function phi(xf, uf, tf, p)
    return 0.0
end

function c(x, u, t, p)
    return [sqrt(cos(x[3])^2 + sin(x[3])^2); u[1]^2]
end

function h(x, u, t, p) 
    return [(x[1] + 5)^2 + (x[2] - 0)^2 - 9;
            (x[1] - 5)^2 + (x[2] - 0)^2 - 9;
            (x[1] - 0.0)^2 + (x[2] - 8.66)^2 - 9;]
end

funcs = DOC.Funs(l, phi, f, h, c)

# Define limits
limits1 = DOC.Limits()
limits1.ll.ti = 0.0
limits1.ul.ti = 0.0
limits1.ll.dt = 0.0 
limits1.ul.dt = 200.0
limits1.ll.tf = 0.0
limits1.ul.tf = 200.0
limits1.ll.xf = [10.0, 0.0, π/2]
limits1.ul.xf = [10.0, 0.0, π/2]
limits1.ll.xi = [-10.0, 0.0, -π/2]
limits1.ul.xi = [-10.0, 0.0, -π/2]
limits1.ll.x = [-30.0, -30.0, -3.5]
limits1.ul.x = [30.0, 30.0, 3.5]
limits1.ll.u = [-1.0]
limits1.ul.u = [1.0]
limits1.ll.c = [10.0; -20.0]
limits1.ul.c = [25.0; 20.0]
limits1.ll.y = [7.0, 7.0, 7.0]
limits1.ul.y = [1000.0, 1000.0, 1000.0]

cp = DOC.ConfigParams("trapezoidal", true, true, "nonlinear")
# Adding first phase
ph1 = DOC.Phase(ocp, p, funcs, d, limits1, cp)
DOC.add_phase(ph1, ocp)

# Adding second phase
limits2 = deepcopy(limits1)
limits2.ll.ti = 0.0
limits2.ul.ti = 100.0
limits2.ll.dt = 0.0 
limits2.ul.dt = 200.0
limits2.ll.tf = 0.0
limits2.ul.tf = 200.0
limits2.ll.xi = [-30.0, -30.0, -3.5]
limits2.ul.xi = [30.0, 30.0, 3.5]
limits2.ll.xf = [0.0, 13.66, -π]
limits2.ul.xf = [0.0, 13.66, -π]
ph2 = DOC.Phase(ocp, p, funcs, d, limits2, cp)
DOC.add_phase(ph2, ocp)

# Adding third phase
limits3 = deepcopy(limits1)
limits3.ll.ti = 0.0
limits3.ul.ti = 100.0
limits3.ll.dt = 0.0 
limits3.ul.dt = 200.0
limits3.ll.tf = 0.0
limits3.ul.tf = 200.0
limits3.ll.xf = [-10.0, 0.0, -π/2]
limits3.ul.xf = [-10.0, 0.0, -π/2]
limits3.ll.xi = [-30.0, -30.0, -3.5]
limits3.ul.xi = [30.0, 30.0, 3.5]
ph3 = DOC.Phase(ocp, p, funcs, d, limits3, cp)
DOC.add_phase(ph3, ocp)

# # Add objective and optimize
# DOC.add_objective(ocp)
# # DOC.add_events(ocp)
# DOC.add_global_params(ocp)
# DOC.optimize!(ocp.model)
# for ph in ocp.ph
#     DOC.store_results!(ph.val, ph.uvars)
#     DOC.calcerrorphase_split(ph)
#     DOC.compute_new_mesh_betts(ph)
# end
DOC.solve(ocp)

using GLMakie
fig = Figure()
ax = Axis(fig[1,1], autolimitaspect = 1)
lines!(ax, DOC.value.(ph1.uvars.x[1,:]), DOC.value.(ph1.uvars.x[2,:]), color = :blue)
lines!(ax, DOC.value.(ph2.uvars.x[1,:]), DOC.value.(ph2.uvars.x[2,:]), color = :blue)
lines!(ax, DOC.value.(ph3.uvars.x[1,:]), DOC.value.(ph3.uvars.x[2,:]), color = :blue)
arc!(ax, Point2f(-5.0, 0.0), 3.0, 0.0, 2π; color =:red)
arc!(ax, Point2f(5.0, 0.0), 3.0, 0.0, 2π; color =:red)
arc!(ax, Point2f(0.0, 8.66), 3.0, 0.0, 2π; color =:red)
fig

# DOC.update_ocp!(ocp)
# for ph in ocp.ph    
#     DOC.update_phase!(ph, ocp.model)
# end

 
