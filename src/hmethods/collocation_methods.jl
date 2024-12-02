"""
hermite_simpson(ph, model)
Add hermite_simpson collocation constraints to model of a particular phase. 
This is a fourth order implicit method.
"""
# function hermite_simpson(ph::PH, model::Model)
#     (;dyn, n, ns, nu, p, x, u, t, h) = ph
#     for j in 1:n-1
#         ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

#         # @infiltrate

#         f1 = dyn(x[1:ns, j], u[1:nu, j], t[j], p)
#         f2 = dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p)
#         xb = (x[1:ns, j] + x[1:ns, j+1])/2 + h[j]*(f1 - f2)/8
#         tb = t[j] + h[j]/2 
#         fb = dyn(xb, ub, tb, p)

#         dx = h[j] * (f1 + 4*fb + f2) / 6

#         ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + dx)
#     end
# end

function hermite_simpson(ph::Phase, model::Model)
    (; dyn, n, ns, nu, p, x, u, t, h) = ph
    ph.f1 = @variable(model, [1:ns, 1:n-1])
    ph.f2 = @variable(model, [1:ns, 1:n-1])
    ph.xb = @variable(model, [1:ns, 1:n-1])
    ph.fb = @variable(model, [1:ns, 1:n-1])
    ph.dx = @variable(model, [1:ns, 1:n-1])
    for j in 1:n-1
        ub = (u[1:nu, j] + u[1:nu, j+1]) / 2

        @constraint(model, ph.f1[1:ns, j] .== dyn(x[1:ns, j], u[1:nu, j], t[j], p))
        @constraint(model, ph.f2[1:ns, j] .== dyn(x[1:ns, j+1], u[1:nu, j+1], t[j+1], p))
        @constraint(model, ph.xb[1:ns, j] .== (x[1:ns, j] + x[1:ns, j+1]) / 2 + h[j] * (ph.f1[1:ns, j] - ph.f2[1:ns, j]) / 8)
        tb = t[j] + h[j] / 2
        @constraint(model, ph.fb[1:ns, j] .== dyn(ph.xb[1:ns, j], ub, tb, p))

        @constraint(model, ph.dx[1:ns, j] .== h[j] * (ph.f1[1:ns, j] + 4 * ph.fb[1:ns, j] + ph.f2[1:ns, j]) / 6)

        ph.collocation_constraints[j] = @constraint(model, x[1:ns, j+1] .== x[1:ns, j] + ph.dx[1:ns, j])
    end
end


"""
    trapezoidal(ph, model)

Adds trapezoidal collocation to model.
"""
function trapezoidal(ph::Phase, d::Dims, model::Model)
    for j in 1:d.n-1
        @constraint(model, ph.cvars.f1[1:d.ns, j] == ph.funs.f(ph.uvars.x[1:d.ns, j], ph.uvars.u[1:d.nu, j], ph.uvars.t[j], ph.p))
        @constraint(model, ph.cvars.f2[1:d.ns, j] == ph.funs.f(ph.uvars.x[1:d.ns, j+1], ph.uvars.u[1:d.nu, j+1], ph.uvars.t[j+1], ph.p))

        @constraint(model, ph.uvars.x[1:d.ns, j+1] == ph.uvars.x[1:d.ns, j] + ph.uvars.h[j] * (ph.cvars.f1[1:d.ns, j] + ph.cvars.f2[1:d.ns, j]) / 2)
        if d.np > 0
            ph.uvars.y[:, j] .= ph.funs.h(ph.uvars.x[1:d.ns, j], ph.uvars.u[1:d.nu, j], ph.uvars.t[j], ph.p)
        end
    end
    if d.np > 0
        ph.uvars.y[:, d.n] .= ph.funs.h(ph.uvars.x[1:d.ns, d.n], ph.uvars.u[1:d.nu, d.n], ph.uvars.t[d.n], ph.p)
    end
end


# function trapezoidal(ph::Phase, d::Dims, model::Model)
#     for j in 1:d.n-1
#         f1 = ph.funs.dyn(ph.uvars.x[1:d.ns, j], ph.uvars.u[1:d.nu, j], ph.uvars.t[j], ph.p)
#         f2 = ph.funs.dyn(ph.uvars.x[1:d.ns, j+1], ph.uvars.u[1:d.nu, j+1], ph.uvars.t[j+1], ph.p)

#         @constraint(model, ph.uvars.x[1:d.ns, j+1] == (ph.uvars.x[1:d.ns, j] + ph.uvars.h[j] * (f1 + f2) / 2))
#     end
# end

