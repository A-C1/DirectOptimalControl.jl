function set_bounds(x, xl, xu, ns)
    for i=1:ns
        if x[i] > xu[i]
            x[i] = xu[i]
        elseif x[i] < xl[i]
            x[i] = xl[i]
        end
    end
end


function generate_initial_trajectory_linear(ph::PH, OC::OCP)
    # Initial value of states. Move this to a new function
    if OC.mesh_iter_no == 1
        ph.uinit = zeros(ph.nu, ph.n)
        ph.xinit = zeros(ph.ns, ph.n)
        if ph.id == 1
            ph.xinit[:,1] .= ph.limits.ll.xi
        else
            ph.xinit[:,1] .= OC.ph[ph.id-1].xinit[:, end]
        end
    end

    for i = 1:ph.n
        for j = 1:ph.nu
            if OC.mesh_iter_no == 1 
                ph.uinit[j, i] = ph.limits.ll.u[j] + (ph.limits.ul.u[j] - ph.limits.ll.u[j])*0.5
            end
            usj = unscaled_to_scaled(ph.uinit[j,i], ph.limits.ul.u[j], ph.limits.ll.u[j])
            set_start_value(ph.us[j,i], usj)
        end
    end
    if OC.mesh_iter_no == 1
        dx = (ph.limits.ul.xf - ph.limits.ll.xi) / ph.n
        for i = 1:ph.n-1
            if OC.mesh_iter_no == 1
                ph.xinit[:, i+1] = ph.xinit[:, i] + dx
            end
        end
    end

    for i = 1:ph.n
        # xv = @view ph.xinit[:,i]
        # setbounds(xv, ph.limits.ll.x, ph.limits.ul.x, ph.ns)
        for j = 1:ph.ns
            xsj = unscaled_to_scaled(ph.xinit[j,i], ph.limits.ul.x[j], ph.limits.ll.x[j])
            set_start_value(ph.xs[j,i], xsj)
        end
    end

    if OC.mesh_iter_no > 1
        for ph in OC.ph
            set_start_value(ph.tfs, ph.tfsval)
            set_start_value(ph.tis, ph.tfsval)
        end
    else
        for ph in OC.ph
            tft = unscaled_to_scaled(ph.limits.ll.tf, ph.limits.ll.tf, ph.limits.ul.tf)
            tit = unscaled_to_scaled(ph.limits.ll.ti, ph.limits.ll.ti, ph.limits.ul.ti)
            set_start_value(ph.tfs, tft)
            set_start_value(ph.tis, tit)
        end
    end

end

function auto_generate_init_values(ph::PH, OC::OCP)
    ph.uinit = zeros(ph.nu, ph.n)
    ph.xinit = zeros(ph.ns, ph.n)
    ph.kinit = zeros(ph.nk)

    # Specify initial value
    if ph.id == 1
        ph.xinit[:, 1] .= ph.limits.ll.xi
    else
        ph.xinit[:, 1] .= OC.ph[ph.id-1].xinit[:, end]
    end

    # Compute average u 
    for i = 1:ph.n
        for j = 1:ph.nu
            if OC.mesh_iter_no == 1
                ph.uinit[j, i] = (ph.limits.ul.u[j] + ph.limits.ll.u[j]) / 2
            end
        end
    end

    # Compute initial x by interpolating linearly between initial and final values
    dx = (ph.limits.ul.xf - ph.limits.ll.xi) / ph.n
    for i = 1:ph.n-1
        if OC.mesh_iter_no == 1
            ph.xinit[:, i+1] = ph.xinit[:, i] + dx
        end
    end

    # Compute initial values for k
    for i = 1:ph.nk
        ph.kinit[i]  = (ph.limits.ll.k[i] + ph.limits.ul.k[i])/2
    end

    # Compute initial values for tf and ts
    ph.tiinit = (ph.limits.ll.ti + ph.limits.ul.ti) / 2
    ph.tfinit = (ph.limits.ll.tf + ph.limits.ul.tf) / 2
end

function set_initial_values(ph::PH)
    for i = 1:ph.n
        # Setting initial values of u
        for j = 1:ph.nu
            usj = unscaled_to_scaled(ph.uinit[j,i], ph.limits.ul.u[j], ph.limits.ll.u[j], ph.scale_flag)
            set_start_value(ph.us[j,i], usj)
        end
        # Setting initial values of x
        for j = 1:ph.ns
            xsj = unscaled_to_scaled(ph.xinit[j,i], ph.limits.ul.x[j], ph.limits.ll.x[j], ph.scale_flag)
            set_start_value(ph.xs[j,i], xsj)
        end
        # Setting initial values of k
        for j = 1:ph.nk
            ksj = unscaled_to_scaled(ph.kinit[j], ph.limits.ul.k[j], ph.limits.ll.k[j], ph.scale_flag)
            set_start_value(ph.ks[j], ksj)
        end
        # Setting initial value of tf
        tft = unscaled_to_scaled(ph.tfinit, ph.limits.ll.tf, ph.limits.ul.tf, ph.scale_flag)
        set_start_value(ph.tfs, tft)
        # Setting initial value of ti
        tit = unscaled_to_scaled(ph.tiinit, ph.limits.ll.ti, ph.limits.ul.ti, ph.scale_flag)
        set_start_value(ph.tis, tit)
    end
end