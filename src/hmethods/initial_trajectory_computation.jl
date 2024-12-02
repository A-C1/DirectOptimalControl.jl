function auto_generate_init_values!(ph::Phase, x0::Vector{Float64})
    # Specify initial value
    if ph.id == 1
        ph.init_traj.x[:, 1] .= ph.limits.ll.xi
    else
        ph.init_traj.x[:, 1] .= x0
    end

    # Compute average u 
    for i = 1:ph.dims.n
        for j = 1:ph.dims.nu
                ph.init_traj.u[j, i] = (ph.limits.ul.u[j] + ph.limits.ll.u[j]) / 2
        end
    end

    # Compute initial x by interpolating linearly between initial and final values
    dx = (ph.limits.ul.xf - ph.limits.ll.xi) / (ph.dims.n - 1)
    for i = 1:ph.dims.n-1
            ph.init_traj.x[:, i+1] = ph.init_traj.x[:, i] + dx
    end

    # Compute initial values for k
    for i = 1:ph.dims.nk
        ph.init_traj.k[i] = (ph.limits.ll.k[i] + ph.limits.ul.k[i]) / 2
    end

    # Compute initial values for tf and ts
    ph.init_traj.ti = (ph.limits.ll.ti + ph.limits.ul.ti) / 2
    ph.init_traj.tf = (ph.limits.ll.tf + ph.limits.ul.tf) / 2
end

function set_initial_values(ph::Phase)
    for i = 1:ph.dims.n
        # Setting initial values of u
        for j = 1:ph.dims.nu
            usj = unscaled_to_scaled(ph.init_traj.u[j, i], ph.limits.ul.u[j], ph.limits.ll.u[j], ph.config_params.scale_flag)
            set_start_value(ph.svars.us[j, i], usj)
        end
        # Setting initial values of x
        for j = 1:ph.dims.ns
            xsj = unscaled_to_scaled(ph.init_traj.x[j, i], ph.limits.ul.x[j], ph.limits.ll.x[j], ph.config_params.scale_flag)
            set_start_value(ph.svars.xs[j, i], xsj)
        end
        # Setting initial values of k
        if ph.dims.nk > 0
            for j = 1:ph.dims.nk
                ksj = unscaled_to_scaled(ph.init_traj.k[j], ph.limits.ul.k[j], ph.limits.ll.k[j], ph.config_params.scale_flag)
                set_start_value(ph.svars.ks[j], ksj)
            end
        end
        # Setting initial value of tf
        tft = unscaled_to_scaled(ph.init_traj.tf, ph.limits.ll.tf, ph.limits.ul.tf, ph.config_params.scale_flag)
        set_start_value(ph.svars.tfs, tft)
        # Setting initial value of ti
        tit = unscaled_to_scaled(ph.init_traj.ti, ph.limits.ll.ti, ph.limits.ul.ti, ph.config_params.scale_flag)
        set_start_value(ph.svars.tis, tit)
    end
end