function aug_quad(x, u, t, p, nq, L, integralfun)
    if nq > 0
        return [L(x, u, t, p); integralfun(x, u, t, p)]
    else
        return [L(x, u, t, p)]
    end
end

function trapezoidal_integrator(ph)
    naux = ph.nq + 1
    xaux = zeros(naux, ph.n)
    for i = 1:ph.n-1
        f1 = aug_quad(ph.xval[:, i], ph.uval[:,i], value(ph.t[i]), ph.p, ph.nq, ph.L,ph.integralfun)
        f2 = aug_quad(ph.xval[:, i+1], ph.uval[:,i+1], value(ph.t[i+1]), ph.p, ph.nq, ph.L, ph.integralfun)
        xaux[:, i+1] = xaux[:,i] + value(ph.h[i])*(f1 + f2)/2
    end

    return xaux
end

function hermite_integrator(ph)
    naux = ph.nq + 1
    xaux = zeros(naux, ph.n)
    x = ph.xval
    u = ph.uval
    t = value.(ph.t)
    h = value.(ph.h)
    for i = 1:ph.n-1
        f1 = aug_quad(ph.xval[:, i], ph.uval[:,i], value(ph.t[i]), ph.p, ph.nq, ph.L,ph.integralfun)
        f2 = aug_quad(ph.xval[:, i+1], ph.uval[:,i+1], value(ph.t[i+1]), ph.p, ph.nq, ph.L, ph.integralfun)

        f1dyn = ph.dyn(x[:, i], u[:, i], t[i], ph.p)
        f2dyn = ph.dyn(x[:, i+1], u[:, i+1], t[i+1], ph.p)
        xb = (x[:, i] + x[:, i+1])/2 + h[i]*(f1dyn - f2dyn)/8
        tb = t[i] + h[i]/2 
        ub = (u[:, i] + u[:, i+1]) / 2

        fb = aug_quad(xb, ub, tb, ph.p, ph.nq, ph.L,ph.integralfun)

        dx = h[i] * (f1 + 4*fb + f2) / 6
        xaux[:, i+1] = xaux[:,i] + dx
    end

    return xaux
end

function mean(x)
    mean = zero(eltype(x))
    M = length(x)
    for i in 1:M
        mean = mean + x[i]
    end

    return mean/M
end

"""
f
This is an internal function
Need to optimize this function heavily since, this is where most of the time is spent.
"""
function f(t, Xtil, Xdtil, Util, p, nq, ns, dyn, L, integralfun)
    X = [x(t) for x in Xtil]
    Xd = [x(t) for x in Xdtil]
    U = [u(t) for u in Util]
    # @infiltrate
    return abs.([Xd[1:ns] - dyn(X[1:ns], U, t, p);
        Xd[ns+1:end] - aug_quad(X[1:ns], U, t, p, nq, L, integralfun)])
end

function calculate_interpolants(ph)
    (;x, u, tau, tf, ti) = ph

    xo = ph.xval
    u = ph.uval
    tf = ph.tfval
    ti = ph.tival

    ## Calculation of pseudo states
    if ph.collocation_method == "trapezoidal"
        xaux = trapezoidal_integrator(ph)
    else
        xaux = hermite_integrator(ph)
    end

    x = [xo; xaux]

    nst = ph.ns + ph.nq + 1


    tp = value.(ph.t)

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

    for i = 1:ph.nu
        Su = interpolate(tp, u[i, :], BSplineOrder(4))
        push!(Util, Su)
    end

    # @infiltrate
    return Xtil, Xdtil, Util, nst
end


function calc_gk(n, nst, tp, Xtil, Xdtil, Util, p, nq, ns, dyn, L, integralfun, we)
    integral = zeros(nst, n - 1)
    Xtil = deepcopy(Xtil)
    Xdtil = deepcopy(Xdtil)
    Util = deepcopy(Util)
    # @infiltrate
    for j = 1:n-1
        integral[:, j], _ = quadgk( x -> f(x, Xtil, Xdtil, Util, p, nq, ns, dyn, L, integralfun),
                                    tp[j], tp[j+1], rtol = sqrt(eps()), atol = eps()/2)
        integral[:, j] = integral[:, j] ./ (we .+ 1)
    end
    # @infiltrate

    return integral
end

function calcerrorphase_split(ph)
    println("Computing error...")
    Xtil, Xdtil, Util, nst = calculate_interpolants(ph)


    tp = value.(ph.t)
    

    # f(0.1, Xtil, Xdtil, Util, ph.p, ph.nq, ph.ns, ph.dyn,ph.L,ph.integralfun)
    # Compute weigths for each interval
    ph.we = zeros(nst)
    for i = 1:nst
        # local X, Xd
        X = Xtil[i].(tp[1:ph.n-1])
        Xd = Xdtil[i].(tp[1:ph.n-1])

        ph.we[i] = maximum([X; Xd])
    end

    # @infiltrate

    println("Integrating with QuadGK...")
    integral = calc_gk(ph.n, nst, tp, deepcopy(Xtil), deepcopy(Xdtil), deepcopy(Util), ph.p, ph.nq, ph.ns, ph.dyn,ph.L,ph.integralfun, ph.we)

    ph.error = [maximum(integral[:, i]) for i=1:ph.n-1]
    ph.error_avg = mean(ph.error)
    push!(ph.error_avg_hist, ph.error_avg)
    
    # @infiltrate
end

function compute_new_mesh_betts(ph::PH, OC::OCP)
    println("Computing new Mesh....")
    # for ph in OC.ph
        # Variables required for the algorithm
        # it = OC.mesh_iter_no
    error = deepcopy(ph.error)
        Ivec = zeros(Int64, ph.n-1)
        Mlim = ph.n
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
        r = 3

        terminate = false
        M1 = 0
        while !terminate
            _, imax = findmax(error)
            Ivec[imax] = Ivec[imax] + 1
            M = M + 1

            # Compute new error
            # error[imax] = ph.error[imax]*(1/(1+Ivec[imax]))^(ph.order - r + 1)
            error[imax] = ph.error[imax]*(1/(1+Ivec[imax]))^2

            errmax = maximum(error)

            M1 = maximum(Ivec)
            # @infiltrate
            if M >= Mlim || M1 >= M1lim || errmax <= OC.tol
                terminate = true
            end
        end
        push!(ph.Iavgh, Int(ceil(mean(Ivec))))

        # Adding the vectors
        dtaun = Float64[]
        unew = Vector{Float64}[]
        xnew = Vector{Float64}[]
        for i = 1:ph.n-1
            if Ivec[i] > 0
                dh = collect(range(start=ph.tau[i], stop=ph.tau[i+1], length=(Ivec[i]+1)))
                append!(dtaun, dh[1:end-1])
                append!(unew,[value.(ph.u[:,i]) for j = 1:length(dh)-1])
                append!(xnew,[value.(ph.x[:,i]) for j = 1:length(dh)-1])
            else
                push!(dtaun, ph.tau[i])
                push!(unew, value.(ph.u[:,i]))
                push!(xnew, value.(ph.x[:,i]))
            end
        end
        push!(dtaun, ph.tau[end])
        push!(unew, value.(ph.u[:,end]))
        push!(xnew, value.(ph.x[:,end]))

        ph.tau = dtaun
        ph.uinit = stack(unew)
        ph.xinit = stack(xnew)
        ph.tiinit = ph.tival
        ph.tfinit = ph.tfval
        if ph.nk > 0
            ph.kinit = ph.kval
        end
        ph.n = length(dtaun)
    # end
end




function solve(OC::OCP)
    err_avg = 1.0
    err_avg_hist = Float64[]
    while ((err_avg > OC.tol) && (OC.mesh_iter_no <= OC.mesh_iter_max) && (OC.infeasible_iter_no <= OC.infeasible_iter_max))
        if OC.mesh_iter_no == 1
            solve_mpocp(OC)
        else
            setup_mpocp(OC)
            solve_mpocp(OC)
        end
        OC.solver_status = termination_status(OC.model)
        if (OC.solver_status == LOCALLY_SOLVED) || (OC.solver_status == OPTIMAL) || (OC.solver_status == ALMOST_OPTIMAL) || (OC.solver_status == ALMOST_LOCALLY_SOLVED)
            OC.infeasible_iter_no = 1
            error = Float64[]
            for ph in OC.ph
                calcerrorphase_split(ph)
                ph.callback_fun(ph)
                compute_new_mesh_betts(ph, OC)
                append!(error, ph.error)
            end
            err_avg = mean(error)
            push!(err_avg_hist, err_avg)

            OC.mesh_iter_no = OC.mesh_iter_no + 1
        else
            OC.infeasible_iter_no = OC.infeasible_iter_no + 1
        end
    end
    println(err_avg_hist)
end