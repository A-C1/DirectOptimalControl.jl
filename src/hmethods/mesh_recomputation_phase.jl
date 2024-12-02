# This function combines the terms which are integrated.
# Namely the running cost function "l" and the quadrature functions "c" 
function aug_quad(x, u, t, p, nq, l, c)
    if nq > 0
        return [l(x, u, t, p); c(x, u, t, p)]
    else
        return [l(x, u, t, p)]
    end
end

# Integrate with trapezoidal integrator
function trapezoidal_integrator(d::Dims, vals::Trajectories, funs::Funs, p::NamedTuple)
    xaux = zeros(d.nq + 1, d.n)
    for i = 1:d.n-1
        f1 = aug_quad(vals.x[:, i], vals.u[:,i], vals.t[i], p, d.nq, funs.l, funs.c)
        f2 = aug_quad(vals.x[:, i+1], vals.u[:,i+1], vals.t[i+1], p, d.nq, funs.l, funs.c)
        xaux[:, i+1] = xaux[:,i] + vals.h[i]*(f1 + f2)/2
    end

    return xaux
end

# Integrate with hermite integrator
function hermite_integrator(d::Dims, vals::Trajectories, funs::Funs, p::NamedTuple)
    xaux = zeros(d.nq + 1, ph.n)
    for i = 1:d.n-1
        f1 = aug_quad(vals.x[:, i], vals.u[:,i], vals.t[i], p, d.nq, funs.l, funs.c)
        f2 = aug_quad(vals.x[:, i+1], vals.u[:,i+1], vals.t[i+1], p, nq, funs.l, funs.c)

        f1dyn = ph.dyn(vals.x[:, i], vals.u[:, i], vals.t[i], p)
        f2dyn = ph.dyn(vals.x[:, i+1], vals.u[:, i+1], vals.t[i+1], p)
        xb = (vals.x[:, i] + vals.x[:, i+1])/2 + vals.h[i]*(f1dyn - f2dyn)/8
        tb = vals.t[i] + vals.h[i]/2 
        ub = (vals.u[:, i] + vals.u[:, i+1]) / 2

        fb = aug_quad(xb, ub, tb, p, d.nq, funs.l,funs.c)

        dx = h[i] * (f1 + 4*fb + f2) / 6
        xaux[:, i+1] = xaux[:,i] + dx
    end

    return xaux
end

# Should I just do usinf statistics
function mean(x)
    mean = zero(eltype(x))
    M = length(x)
    for i in 1:M
        mean = mean + x[i]
    end

    return mean/M
end

function f(t, Xtil, Xdtil, Util, p, nq, ns, f, l, c)
    X = [x(t) for x in Xtil]
    Xd = [x(t) for x in Xdtil]
    U = [u(t) for u in Util]
    return abs.([Xd[1:ns] - f(X[1:ns], U, t, p);
        Xd[ns+1:end] - aug_quad(X[1:ns], U, t, p, nq, l, c)])
end

function calculate_interpolants(ph::Phase)
    xo = ph.val.x
    u = ph.val.u
    tf = ph.val.tf
    ti = ph.val.ti
    tau = ph.mesh.tau

    ## Calculation of pseudo states
    if ph.config_params.collocation_method == "trapezoidal"
        xaux = trapezoidal_integrator(ph.dims, ph.val, ph.funs, ph.p)
    else
        xaux = hermite_integrator(ph.dims, ph.val, ph.funs, ph.p)
    end

    x = [xo; xaux]

    nst = ph.dims.ns + ph.dims.nq + 1


    tp = ph.val.t

    S = interpolate(tp, x[1, :], BSplineOrder(4))
    Sd = diff(S, Derivative(1))
    Su = interpolate(tp, u[1, :], BSplineOrder(4))
    Stype = typeof(S)
    SDtype = typeof(Sd)
    Sutype = typeof(Su)

    Xtil = Stype[]
    Xdtil = SDtype[]
    Util = Sutype[]

    for i = 1:nst
        S = interpolate(tp, x[i, :], BSplineOrder(4))
        Sd = diff(S, Derivative(1))

        push!(Xtil, S)
        push!(Xdtil, Sd)
    end

    for i = 1:ph.dims.nu
        Su = interpolate(tp, u[i, :], BSplineOrder(4))
        push!(Util, Su)
    end

    return Xtil, Xdtil, Util, nst
end



# Need to optimize this function heavily since, this is where most of the time is spent.
function calc_gk(n, nst, tp, Xtil, Xdtil, Util, p, nq, ns, dyn, L, integralfun, we)
    integral = zeros(nst, n - 1)
    Xtil = deepcopy(Xtil)
    Xdtil = deepcopy(Xdtil)
    Util = deepcopy(Util)
    for j = 1:n-1
        integral[:, j], _ = quadgk( x -> f(x, Xtil, Xdtil, Util, p, nq, ns, dyn, L, integralfun),
                                    tp[j], tp[j+1], rtol = sqrt(eps()), atol = eps()/2)
        integral[:, j] = integral[:, j] ./ (we .+ 1)
    end

    return integral
end

function calcerrorphase_split(ph::Phase)
    println("Computing error...")
    Xtil, Xdtil, Util, nst = calculate_interpolants(ph)

    tp = ph.val.t

    # f(0.1, Xtil, Xdtil, Util, ph.p, ph.nq, ph.ns, ph.dyn,ph.L,ph.integralfun)
    # Compute weigths for each interval
    ph.mesh.we = zeros(nst)
    for i = 1:nst
        X = Xtil[i].(tp[1:ph.dims.n-1])
        Xd = Xdtil[i].(tp[1:ph.dims.n-1])

        ph.mesh.we[i] = maximum([X; Xd])
    end

    println("Integrating with QuadGK...")
    integral = calc_gk(ph.dims.n, nst, tp, deepcopy(Xtil), deepcopy(Xdtil), deepcopy(Util), ph.p, ph.dims.nq, ph.dims.ns, ph.funs.f, ph.funs.l, ph.funs.c, ph.mesh.we)

    ph.mesh.error = [maximum(integral[:, i]) for i=1:ph.dims.n-1]
    ph.mesh.error_avg = mean(ph.mesh.error)
    push!(ph.mesh.error_avg_hist, ph.mesh.error_avg)
end

function compute_new_mesh_betts(ph::Phase)
    println("Computing new Mesh for phase....")
    error = deepcopy(ph.mesh.error)
    Ivec = zeros(Int64, ph.dims.n - 1)
    Mlim = ph.dims.n
    M1lim = 5
    M = 0

    # # Compute index reduction (Does not work currently. Need to correct)
    # if it == 1
    #     r = 0
    # else
    #     err_ratio = log(ph.error_avg_hist[it-1]/ph.error_avg_hist[it])
    #     rhat = ph.order + 1 - err_ratio/ log(1+ph.Iavgh[it-1])
    #     rhatint = Int(round(rhat))
    #     r = maximum([0, minimum([rhatint, ph.order])])
    # end
    # r = 3

    terminate = false
    M1 = 0
    while !terminate
        _, imax = findmax(error)
        Ivec[imax] = Ivec[imax] + 1
        M = M + 1

        # Compute new error
        # error[imax] = ph.error[imax]*(1/(1+Ivec[imax]))^(ph.order - r + 1)
        error[imax] = ph.mesh.error[imax] * (1 / (1 + Ivec[imax]))^2

        errmax = maximum(error)

        M1 = maximum(Ivec)
        # @infiltrate
        if M >= Mlim || M1 >= M1lim || ph.mesh.tol >= errmax
            terminate = true
        end
    end
    push!(ph.mesh.Iavgh, Int(ceil(mean(Ivec))))

    # Adding the vectors
    dtaun = Float64[]
    unew = Vector{Float64}[]
    xnew = Vector{Float64}[]
    for i = 1:ph.dims.n-1
        if Ivec[i] > 0
            dh = collect(range(start=ph.mesh.tau[i], stop=ph.mesh.tau[i+1], length=(Ivec[i] + 2)))
            append!(dtaun, dh[1:end-1])
            append!(unew, [value.(ph.uvars.u[:, i]) for j = 1:length(dh)-1])
            append!(xnew, [value.(ph.uvars.x[:, i]) for j = 1:length(dh)-1])
        else
            push!(dtaun, ph.mesh.tau[i])
            push!(unew, ph.val.u[:, i])
            push!(xnew, ph.val.x[:, i])
        end
    end
    # @infiltrate
    push!(dtaun, ph.mesh.tau[end])
    push!(unew, ph.val.u[:, end])
    push!(xnew, ph.val.x[:, end])

    ph.mesh.tau = dtaun
    ph.init_traj.u = stack(unew)
    ph.init_traj.x = stack(xnew)
    ph.init_traj.ti = ph.val.ti
    ph.init_traj.tf = ph.val.tf
    if ph.dims.nk > 0
        ph.init_traj.k = ph.val.k
    end
    ph.dims.n = length(dtaun)
    # end
end

function update_ocp!(ocp::OCP)
    empty!(ocp.ph)
    ocp.model = Model(Ipopt.Optimizer) # New Model
end

function update_phase!(ph::Phase, model::Model)
    ph.svars = ScaledVariables(ph.dims, model)
    ph.uvars = UnscaledVariables(ph.dims, ph.svars, ph.limits, ph.config_params, ph.mesh, model)
    ph.cvars = CacheVars(ph.dims, model)
    ph.sval = Trajectories(ph.dims)
    ph.val = Trajectories(ph.dims)
    ph.constraints = Constraints(ph.dims)
    p = ph.p[filter(x -> x != :k, keys(ph.p))]
    ph.p = (p..., k = ph.uvars.k)
end

# function solve(OC::OCP)
#     err_avg = 1.0
#     err_avg_hist = Float64[]
#     while ((err_avg > OC.tol) && (OC.mesh_iter_no <= OC.mesh_iter_max) && (OC.infeasible_iter_no <= OC.infeasible_iter_max))
#         if OC.mesh_iter_no == 1
#             for ph in OC.ph
#                 if OC.additional_constraints_flag
#                     OC.additional_constraints(ph, OC)
#                 end
#             end
#             solve_mpocp(OC)
#         else
#             setup_mpocp(OC)
#             for ph in OC.ph
#                 if OC.additional_constraints_flag
#                     OC.additional_constraints(ph, OC)
#                 end
#             end
#             solve_mpocp(OC)
#         end
#         OC.solver_status = termination_status(OC.model)
#         if ((OC.solver_status == LOCALLY_SOLVED) || (OC.solver_status == OPTIMAL) || (OC.solver_status == ALMOST_OPTIMAL) || (OC.solver_status == ALMOST_LOCALLY_SOLVED))
#             OC.infeasible_iter_no = 1
#             error = Float64[]
#             for ph in OC.ph
#                 calcerrorphase_split(ph)
#                 append!(error, ph.error)
#                 # ph.callback_fun(ph)
#                 if OC.mesh_iter_no < OC.mesh_iter_max
#                     compute_new_mesh_betts(ph, OC)
#                 end
#             end
#             err_avg = mean(error)
#             push!(err_avg_hist, err_avg)

#             OC.mesh_iter_no = OC.mesh_iter_no + 1
#         else
#             OC.infeasible_iter_no = OC.infeasible_iter_no + 1
#         end
#     end
#     println(err_avg_hist)
# end