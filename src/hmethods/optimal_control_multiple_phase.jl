function obj_f1(ph::Phase)
    if ph.config_params.collocation_method == "trapezoidal"
        obj = quadrature_trapezoidal(ph.funs.l, ph.dims, ph.uvars, ph.p, 1)[1]
    else
        obj = quadrature_simpson(ph.L, ph.dyn, 1, ph, true)[1]
    end
    obj = obj + ph.funs.phi(ph.uvars.x[1:ph.dims.ns, end], ph.uvars.u[1:ph.dims.nu, end], ph.uvars.t[end], ph.p)

    return obj
end

function add_phase(ph::Phase, OC::OCP)
    println("Adding Phase $(ph.id) .....")
    (; dyn, n, ns, nu, nk, p) = ph

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
        hermite_simpson(ph, model)
    elseif ph.collocation_method == "trapezoidal"
        trapezoidal(ph, model)
    end

    println("Phase $(ph.id) Added")
end


function add_var_constraints(uv::UnscaledVariables, d::Dims, lims::Limits, model::Model)
    # Relation between inital and final time
    @constraint(model, uv.tf >= uv.ti + eps())

    # Add all the limits. Move to function maybe
    for i = 1:d.n
        for j = 1:d.ns
            @constraint(model, uv.x[j, i] <= lims.ul.x[j])
            @constraint(model, uv.x[j, i] >= lims.ll.x[j])
        end
        for j = 1:d.nu
            @constraint(model, uv.u[j, i] <= lims.ul.u[j])
            @constraint(model, uv.u[j, i] >= lims.ll.u[j])
        end
    end

    # Check why this does not work?
    # for i = 1:ph.n
    #     @constraint(model, ph.limits.ll.x .<= ph.x[:, i] .<= ph.limits.ul.x)
    #     @constraint(model, ph.limits.ll.u .<= ph.u[:, i] .<= ph.limits.ul.u)
    # end

    @constraint(model, uv.tf >= lims.ll.tf)
    @constraint(model, uv.tf <= lims.ul.tf)
    @constraint(model, uv.ti <= lims.ul.ti)
    @constraint(model, uv.ti >= lims.ll.ti)
    @constraint(model, uv.Δt <= lims.ul.dt)
    @constraint(model, uv.Δt >= lims.ll.dt)

    for j = 1:d.ns
        @constraint(model, uv.xf[j] <= lims.ul.xf[j])
        @constraint(model, uv.xf[j] >= lims.ll.xf[j])
        @constraint(model, uv.xi[j] <= lims.ul.xi[j])
        @constraint(model, uv.xi[j] >= lims.ll.xi[j])
    end

    if d.nk > 0
        for j = 1:d.nk
            @constraint(model, uv.k[j] <= lims.ul.k[j])
            @constraint(model, uv.k[j] >= lims.ll.k[j])
        end
    end
end

function add_quadrature_constraints(ph::Phase, model)
    if ph.dims.nq > 0
        if ph.config_params.collocation_method == "trapezoidal"
            q = quadrature_trapezoidal(ph.funs.dyn, ph.dims, ph.uvars, ph.p, ph.dims.nq)
        else
            # q = quadrature_simpson(ph.integralfun, ph.dyn, ph.nq, ph, false)
        end
        ph.constraints.quadrature_constraints_upper = @constraint(model, q .<= ph.limits.ul.integral)
        ph.constraints.quadrature_constraints_lower = @constraint(model, q .>= ph.limits.ll.integral)
    end
end

function add_path_constraints(ph::Phase, model)
    for i = 1:ph.dims.n
        ph.constraints.path_constraints_upper[i] = @constraint(model, ph.funs.pathfun(ph.uvars.x[:, i], ph.uvars.u[:, i], ph.uvars.t[i], ph.p) .<= ph.limits.ul.path)
        ph.constraints.path_constraints_lower[i] = @constraint(model, ph.funs.pathfun(ph.uvars.x[:, i], ph.uvars.u[:, i], ph.uvars.t[i], ph.p) .>= ph.limits.ll.path)
    end
end

function setup_mpocp(OC::OCP)
    OC.model = Model(Ipopt.Optimizer)
    set_attribute(OC.model, "tol", 1e-6)
    ph = OC.ph

    obj = 0.0
    for ph in ph
        ph.k = @variable(OC.model, [1:ph.nk])   # Assigne them to field k in struct `ph`
        OC.kg = @variable(OC.model, [1:OC.nkg]) # Assign them to field k in struct `OC`
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

    OC.event_constraints_lower = Vector{ConstraintRef}(undef, OC.npsi)
    OC.event_constraints_upper = Vector{ConstraintRef}(undef, OC.npsi)
    if OC.npsi > 0
        psi = OC.psi(OC)
        for i in 1:OC.npsi
            OC.event_constraints_lower[i] = @constraint(OC.model, psi[i] >= OC.psi_llim[i])
            OC.event_constraints_upper[i] = @constraint(OC.model, psi[i] <= OC.psi_ulim[i])
        end
    end

    # Add constraints on global parameters
    if OC.nkg > 0
        OC.kg = @variable(OC.model, [1:OC.nkg], base_name = "kg")
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
        elseif OC.mesh_iter_no == 1 && ph.set_initial_vals == "User"
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
