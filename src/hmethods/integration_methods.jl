function quadrature_trapezoidal(f, d::Dims, uv::UnscaledVariables, p::NamedTuple, quad_dim::Int64)
        sum = zeros(NonlinearExpr, quad_dim)
        for i = 1:d.n-1
            f1 = [f(uv.x[1:d.ns, i], uv.u[1:d.nu, i], uv.t[i], p);]
            f2 = [f(uv.x[1:d.ns, i+1], uv.u[1:d.nu, i+1], uv.t[i+1], p);]
            dx = uv.h[i] * (f1 + f2) / 2

            sum = sum + dx
        end
        return sum
end

# function quadrature_simpson(f, dyn, nq, ph, isscalar)
#     (;x, u, h, t, p, n, ns, nu) = ph
#     sum = zeros(NonlinearExpr, nq)
#     for i = 1:n-1
#         f1 = [f(x[1:ns, i], u[1:nu, i], t[i], p);]
#         f2 = [f(x[1:ns, i+1], u[1:nu, i+1], t[i+1], p);]

#         f1dyn = dyn(x[1:ns, i], u[1:nu, i], t[i], p)
#         f2dyn = dyn(x[1:ns, i+1], u[1:nu, i+1], t[i+1], p)

#         xb = (x[1:ns, i] + x[1:ns, i+1])/2 + h[i]*(f1dyn - f2dyn)/8
#         tb = t[i] + h[i]/2 
#         ub = (u[1:nu, i] + u[1:nu, i+1]) / 2
#         fb = [f(xb, ub, tb, p);]

#         dx = h[i] * (f1 + 4*fb + f2) / 6

#         sum = sum + dx
#     end
#     return sum
# end