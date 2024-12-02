function add_phase(ph::Phase, ocp::OCP)
    println("Adding Phase")
    add_var_constraints(ph.uvars, ph.dims, ph.limits, ocp.model)
    if ph.dims.nq > 0
        add_quadrature_constraints(ph, ocp.model)  # Change this signature
    end
    if ph.dims.np > 0
        add_path_constraints(ph, ocp.model)  # Change this signature
    end
    trapezoidal(ph, ph.dims, ocp.model)  # Change this signature

    # Initialize the generate_initial_trajectory_linear
    if ocp.config_params.mesh_iter_no == 1
        auto_generate_init_values!(ph, ph.limits.ll.xi)
    end
    set_initial_values(ph)
end

function add_events(ocp::OCP)
    if ocp.dims.npsi > 0
        @constraint(ocp.model, ocp.psi(ocp.ph) <= ocp.limits.ul.psi)
        @constraint(ocp.model, ocp.limits.ll.psi <= ocp.psi(ocp.ph))
    end
end

function add_global_params(ocp::OCP)
    if ocp.dims.nkg >= 0
        @constraint(ocp.model, ocp.kg <= ocp.limits.ul.kg)
        @constraint(ocp.model, ocp.kg >= ocp.limits.ll.kg)
    end
end

function add_objective(ocp::OCP)
    obj = 0.0
    for ph in ocp.ph
        obj = obj + obj_sp(ph)
    end

    @objective(ocp.model, Min, obj)
end

function solve(ocp::OCP)
    # while ocp.config_params.mesh_iter_no < ocp.config_params.mesh_iter_max
        for ph in ocp.ph
            add_phase(ph, ocp)
        end
        add_objective(ocp)
        add_events(ocp)
        add_global_params(ocp)
        optimize!(ocp.model)
        # for ph in ocp.ph
        #     store_results!(ph.val, ph.uvars)
        #     calcerrorphase_split(ph)
        #     compute_new_mesh_betts(ph)
        # end
        # old_ph_vec = deepcopy(ocp.ph)
        # update_ocp!(ocp)
        # for ph in old_ph_vec
        #     push!(ocp.ph, ph)
        # end
    # end
end




# function setup_mpocp(OC::OCP)
#     OC.model = Model(Ipopt.Optimizer)
#     set_attribute(OC.model, "tol", 1e-6)
#     ph = OC.ph

#     obj = 0.0
#     for ph in ph
#         ph.k = @variable(OC.model, [1:ph.nk])   # Assigne them to field k in struct `ph`
#         OC.kg = @variable(OC.model, [1:OC.nkg]) # Assign them to field k in struct `OC`
#         add_phase(ph, OC)
#         # obj = obj + obj_f(ph.x[1:ph.ns, 1:ph.n], ph.u[1:ph.nu, 1:ph.n], ph.t[1:ph.n], ph.h[1:ph.n-1], ph.L, ph.phi, ph.n, ph.ns, ph.nu, ph.p)
#         obj = obj + obj_f1(ph)
#     end

#     OC.obj = obj
#     if OC.set_obj_lim == true
#         @constraint(OC.model, OC.obj <= OC.obj_ulim)
#         @constraint(OC.model, OC.obj >= OC.obj_llim)
#     end

#     if OC.objective_sense == "Max"
#         @objective(OC.model, Max, OC.obj)
#     else
#         @objective(OC.model, Min, OC.obj)
#     end

#     OC.event_constraints_lower = Vector{ConstraintRef}(undef, OC.npsi)
#     OC.event_constraints_upper = Vector{ConstraintRef}(undef, OC.npsi)
#     if OC.npsi > 0
#         psi = OC.psi(OC)
#         for i in 1:OC.npsi
#             OC.event_constraints_lower[i] = @constraint(OC.model, psi[i] >= OC.psi_llim[i])
#             OC.event_constraints_upper[i] = @constraint(OC.model, psi[i] <= OC.psi_ulim[i])
#         end
#     end

#     # Add constraints on global parameters
#     if OC.nkg > 0
#         OC.kg = @variable(OC.model, [1:OC.nkg], base_name = "kg")
#         for i = 1:OC.nkg
#             @constraint(OC.model, OC.kg[i] <= OC.kg_ulim[i])
#             @constraint(OC.model, OC.kg[i] >= OC.kg_llim[i])
#         end
#     end

#     for ph in ph
#         # generate_initial_trajectory_linear(ph, OC)
#         if OC.mesh_iter_no == 1 && ph.set_initial_vals == "Auto"
#             println("Computing initial values automatically for Phase $(ph.id)....")
#             auto_generate_init_values(ph, OC)
#         elseif OC.mesh_iter_no == 1 && ph.set_initial_vals == "User"
#             println("Setting initial values which are user defined for Phase $(ph.id)...")
#         else
#             println("Setting initial values from previous mesh iteration for Phase $(ph.id)...")
#         end
#         set_initial_values(ph)
#     end

# end

# function solve_mpocp(OC::OCP)
#     println("Solving mesh iteration no:$(OC.mesh_iter_no) in multi-phase optimal control problem....")
#     optimize!(OC.model)

#     for ph in OC.ph
#         ph.xval = value.(ph.x)
#         ph.uval = value.(ph.u)
#         ph.tfval = value(ph.tf)
#         ph.tival = value(ph.ti)
#         if ph.nk > 0
#             ph.kval = value.(ph.k)
#         end

#         ph.xsval = value.(ph.xs)
#         ph.usval = value.(ph.us)
#         ph.tfsval = value(ph.tfs)
#         ph.tisval = value(ph.tis)

#         if ph.nk > 0
#             ph.ksval = value.(ph.ks)
#         end
#     end
# end
