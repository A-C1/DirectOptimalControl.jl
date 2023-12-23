function obj_f1(ph)
    if ph.collocation_method == "trapezoidal"
        obj = quadrature_trapezoidal(ph.L, 1, ph, true)[1]
    else
        obj = quadrature_simpson(ph.L,ph.dyn, 1, ph, true)[1]
    end
    obj = obj + ph.phi(ph.x[1:ph.ns, end], ph.u[1:ph.nu, end], ph.t[end], ph.p)

    return obj
end

function add_phase(ph::PH, OC::OCP)
    println("Adding Phase $(ph.id) .....")
    (;dyn, n, ns, nu, nk, p) = ph

    model = OC.model

    if ph.set_initial_vals == "Auto" && OC.mesh_iter_no == 1
        println("tau setup automatically")
        ph.tau = range(start = 0, stop = 1, length = ph.n)
    end

    ph.tis = @variable(model, base_name = "tis")
    ph.tfs = @variable(model, base_name = "tfs")
    ph.xs = @variable(model, [1:ns,1:n],base_name = "xs")
    ph.us = @variable(model, [1:nu,1:n],base_name = "us")
    ph.xis = @view ph.xs[:,1]
    ph.xfs = @view ph.xs[:,end]

    if ph.nk > 0
        ph.ks = @variable(model, [1:nk],base_name = "ks")
    end

    unscaled_vars!(ph)
    ph.xi = @view ph.x[:,1]
    ph.xf = @view ph.x[:,end]

    # Some convinietnt expressions
    ph.Δt = ph.tf - ph.ti
    ph.t = @expression(model, [i=1:n], ph.ti + ph.tau[i]*ph.Δt)
    ph.h = @expression(model, [i=1:n-1], (ph.t[i+1] - ph.t[i]))


    # Add different constraints phasewise
    # Add constraints on variables
    add_var_constraints(ph, OC)

    # Add path constraints
    if ph.np > 0
        println("Adding path constraints")
        add_path_constraints(ph, OC)
        println("Succesfully added path constraints")
    end

    # Add quadrature constraints
    if ph.nq > 0
        println("Adding quadrature constraints")
        add_quadrature_constraints(ph, OC)
        println("Succesfully added quadrature constraints")
    end

    if ph.collocation_method == "hermite-simpson"
        ph.order = 4
        hermite_simpson(ph, model)
    elseif ph.collocation_method == "trapezoidal"
        ph.order = 2
        trapezoidal(ph, model)
    elseif ph.collocation_method == "RK4"  # Remove RK4 and Euler
        ph.order = 4
        RK4(ph, model)
    elseif ph.collocation_method =="euler"
        ph.order = 2
        euler(ph, model)
    end


    println("Phase $(ph.id) Added")
end


function add_var_constraints(ph::PH, OC::OCP)
    # Relation between inital and final time
    model = OC.model
    @constraint(model, ph.tf >= ph.ti + eps())

    # Add all the limits. Move to function maybe
    for i = 1:ph.n
        for j = 1:ph.ns
            @constraint(model, ph.x[j, i] <= ph.limits.ul.x[j])   
            @constraint(model, ph.x[j, i] >= ph.limits.ll.x[j])   
        end
        for j = 1:ph.nu
            @constraint(model, ph.u[j, i] <= ph.limits.ul.u[j])
            @constraint(model, ph.u[j, i] >= ph.limits.ll.u[j])
        end
    end

    @constraint(model, ph.tf <= ph.limits.ul.tf)
    @constraint(model, ph.tf >= ph.limits.ll.tf)
    # @constraint(model, ph.limits.ll.tf <= ph.tf <= ph.limits.ul.tf)
    @constraint(model, ph.ti <= ph.limits.ul.ti)
    @constraint(model, ph.ti >= ph.limits.ll.ti)
    @constraint(model, ph.Δt <= ph.limits.ul.dt)
    @constraint(model, ph.Δt >= ph.limits.ll.dt)

    for j = 1:ph.ns
        @constraint(model, ph.xf[j] <= ph.limits.ul.xf[j])
        @constraint(model, ph.xf[j] >= ph.limits.ll.xf[j])
        @constraint(model, ph.xi[j] <= ph.limits.ul.xi[j])
        @constraint(model, ph.xi[j] >= ph.limits.ll.xi[j])
    end

    if ph.nk > 0
        for j = 1:ph.nk
            @constraint(model, ph.k[j] <= ph.limits.ul.k[j])
            @constraint(model, ph.k[j] >= ph.limits.ll.k[j])
        end
    end
end

function add_quadrature_constraints(ph::PH, OC::OCP)
    if ph.collocation_method == "trapezoidal" || ph.collocation_method =="euler"
        q = quadrature_trapezoidal(ph.integralfun, ph.nq, ph, false)
    else
        q = quadrature_simpson(ph.integralfun, ph.dyn, ph.nq, ph, false)
    end
    @constraint(OC.model, q .<= ph.limits.ul.integral)
    @constraint(OC.model, q .>= ph.limits.ll.integral)
end

function add_path_constraints(ph::PH, OC::OCP)
    for i = 1:ph.n
        @constraint(OC.model, ph.pathfun(ph.x[:,i], ph.u[:,i], ph.t[i], ph.p) .<= ph.limits.ul.path)
        @constraint(OC.model, ph.pathfun(ph.x[:,i], ph.u[:,i], ph.t[i], ph.p) .>= ph.limits.ll.path)
    end
end

function setup_mpocp(OC::OCP)
    ph = OC.ph

    obj = 0.0
    for ph in ph
        add_phase(ph, OC)
        # obj = obj + obj_f(ph.x[1:ph.ns, 1:ph.n], ph.u[1:ph.nu, 1:ph.n], ph.t[1:ph.n], ph.h[1:ph.n-1], ph.L, ph.phi, ph.n, ph.ns, ph.nu, ph.p)
        obj = obj + obj_f1(ph)
    end

    OC.obj = obj
    if OC.set_obj_lim == true
        @constraint(OC.model, OC.obj <= OC.obj_ulim)
        @constraint(OC.model, OC.obj >= OC.obj_llim)
    end

    if OC.objective_sense == "Max"
        @objective(OC.model, Max, OC.obj)
    else
        @objective(OC.model, Min, OC.obj)
    end

    if OC.npsi > 0
            psi = OC.psi(OC)
        for i in 1:OC.npsi 
            @constraint(OC.model, psi[i] >= OC.psi_llim[i])
            @constraint(OC.model, psi[i] <= OC.psi_ulim[i])
        end
    end

    # Add constraints on global parameters
    if OC.nkg > 0
        OC.kg = @variable(OC.model,[1:OC.nkg], base_name = "kg")
        for i = 1:OC.nkg
            @constraint(OC.model, OC.kg[i] <= OC.kg_ulim[i])
            @constraint(OC.model, OC.kg[i] >= OC.kg_llim[i])
        end
    end

    for ph in ph
        # generate_initial_trajectory_linear(ph, OC)
        if OC.mesh_iter_no == 1 && ph.set_initial_vals == "Auto"
            println("Computing initial values automatically for Phase $(ph.id)....")
            auto_generate_init_values(ph, OC)
        elseif OC.mesh_iter_no ==1 && ph.set_initial_vals == "User"
            println("Setting initial values which are user defined for Phase $(ph.id)...")
        else
            println("Setting initial values from previous mesh iteration for Phase $(ph.id)...")
        end
        set_initial_values(ph)
    end

end

function solve_mpocp(OC::OCP)
    println("Solving mesh iteration no:$(OC.mesh_iter_no) in multi-phase optimal control problem....")
    optimize!(OC.model)

    for ph in OC.ph
        ph.xval = value.(ph.x)
        ph.uval = value.(ph.u)
        ph.tfval = value(ph.tf)
        ph.tival = value(ph.ti)
        if ph.nk > 0
            ph.kval = value.(ph.k)
        end

        ph.xsval = value.(ph.xs)
        ph.usval = value.(ph.us)
        ph.tfsval = value(ph.tfs)
        ph.tisval = value(ph.tis)

        if ph.nk > 0
            ph.ksval = value.(ph.ks)
        end
    end
end
