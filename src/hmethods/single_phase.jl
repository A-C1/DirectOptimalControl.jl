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
    @constraint(model, uv.dt <= lims.ul.dt)
    @constraint(model, uv.dt >= lims.ll.dt)

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
            q = quadrature_trapezoidal(ph.funs.c, ph.dims, ph.uvars, ph.p, ph.dims.nq)
        else
            # q = quadrature_simpson(ph.integralfun, ph.dyn, ph.nq, ph, false)
        end
        ph.constraints.quadrature_constraints_upper = @constraint(model, q .<= ph.limits.ul.c)
        ph.constraints.quadrature_constraints_lower = @constraint(model, q .>= ph.limits.ll.c)
    end
end

function add_path_constraints(ph::Phase, model)
    if ph.dims.np > 0
        for i = 1:ph.dims.n
            @constraint(model, ph.funs.h(ph.uvars.x[:, i], ph.uvars.u[:, i], ph.uvars.t[i], ph.p) <= ph.limits.ul.y)
            @constraint(model, ph.funs.h(ph.uvars.x[:, i], ph.uvars.u[:, i], ph.uvars.t[i], ph.p) >= ph.limits.ll.y)
        end
    end
end



function obj_sp(ph::Phase)
    if ph.config_params.collocation_method == "trapezoidal"
        obj = quadrature_trapezoidal(ph.funs.l, ph.dims, ph.uvars, ph.p, 1)[1]
    else
        # obj = quadrature_simpson(ph.L, ph.dyn, 1, ph, true)[1]
    end
    obj = obj + ph.funs.phi(ph.uvars.x[1:ph.dims.ns, end], ph.uvars.u[1:ph.dims.nu, end], ph.uvars.t[end], ph.p)

    return obj
end

function store_results!(vals::Trajectories, uv::UnscaledVariables)
    vals.ti = value(uv.ti)
    vals.tf = value(uv.tf)
    vals.x =  value.(uv.x)
    vals.u = value.(uv.u)
    vals.t = value.(uv.t)
    vals.k = value.(uv.k)
    vals.y = value.(uv.y)
end