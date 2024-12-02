"""
LimitVars

Struct which defines variables which have bounds.
Defines following variables: ti, tf, dt, xf, xi, x, u, integral, path, k
These are to be provided by the user
"""
mutable struct LimitVars
    ti::Float64             # Initial time
    tf::Float64             # Final time
    dt::Float64             # delta time
    xf::Vector{Float64}
    xi::Vector{Float64}
    x::Vector{Float64}
    u::Vector{Float64}
    integral::Vector{Float64}
    path::Vector{Float64}
    k::Vector{Float64}

    function LimitVars()
        return new()
    end
end

"""
Limits

This is the actual field which the user will create.
The LimitVars structure will be filled through this
"""
mutable struct Limits
    ul::LimitVars
    ll::LimitVars

    function Limits()
        limit = new()
        limit.ul = LimitVars()
        limit.ll = LimitVars()

        return limit
    end
end

"""
Dims

To be provided by the user
"""
mutable struct Dims
    n::Int64    # Number of finite elements
    ns::Int64   # State dim
    nu::Int64   # Input dim
    nq::Int64   # Integral constraint dim
    np::Int64   # Path constraint dim
    nk::Int64   # Free parameters dim

    function Dims()
        return new()
    end
end

abstract type AbstractFuns end
"""
Funs

Various functions required for the optimal control problem. To be provided by
the user
"""
struct Funs{L <: Function, P <: Function, D <: Function, PF <: Function, IF <: Function} <: AbstractFuns 
    l::L
    phi::P
    dyn::D
    pathfun::PF
    integralfun::IF

    # Use the default constructor
end


"""
Mesh

Stores initial mesh and constants for
mesh correction
"""
mutable struct Mesh
    # Storing error information
    tau::Vector{Float64}                # Initial mesh. Initialized to uniform values
    we::Vector{Float64}
    error::Vector{Float64}
    error_avg::Float64
    error_avg_hist::Vector{Float64}
    Iavgh::Vector{Int64}
end

function Mesh(d::Dims)
    tau = collect(range(start = 0, stop = 1, length = d.n))
    we = zeros(d.n-1)
    error = zeros(d.n-1)
    error_avg = 0.0
    error_avg_hist = Float64[]
    Iavgh = zeros(d.n-1)

    return Mesh(tau, we, error, error_avg, error_avg_hist, Iavgh)
end

"""
Scaled
"""
mutable struct ScaledVariables
    tis::VariableRef
    tfs::VariableRef
    xs::Matrix{VariableRef}
    us::Matrix{VariableRef}
    ks::Vector{VariableRef}
    xis::Vector{VariableRef}
    xfs::Vector{VariableRef}
end

function ScaledVariables(dims::Dims, model::Model)
    tis = @variable(model, base_name = "tis")
    tfs = @variable(model, base_name = "tfs")
    xs = @variable(model, [1:dims.ns,1:dims.n],base_name = "xs")
    us = @variable(model, [1:dims.nu,1:dims.n],base_name = "us")
    ks = @variable(model, [1:dims.nk], base_name = "ks")
    xis = @view xs[:,1]
    xfs = @view xs[:,end]    

    return ScaledVariables(tis, tfs, xs, us, ks, xis, xfs)
end


"""
ConfigParams

Various phase related configurations
"""
mutable struct ConfigParams
    # collocation_method
    collocation_method::String
    # Additional flags
    set_initial_vals::Bool
    scale_flag::Bool
end


"""
UnscaledVariables

Used internally to provide unscaled variables
"""
mutable struct UnscaledVariables
    tf::AffExpr
    ti::AffExpr
    x::Matrix{AffExpr}
    u::Matrix{AffExpr}
    k::Vector{AffExpr}

    xf::Vector{AffExpr}
    xi::Vector{AffExpr}
    t::Vector{AffExpr}
    h::Vector{AffExpr}
    Δt::AffExpr
end


function UnscaledVariables(d::Dims, sv::ScaledVariables, limits::Limits, cp::ConfigParams, m::Mesh, model::Model)
    # Use functions from scaling.jl to compute the un
    tf = scaled_to_unscaled(sv.tfs, limits.ul.tf, limits.ll.tf, cp.scale_flag)
    ti = scaled_to_unscaled(sv.tis, limits.ul.ti, limits.ll.ti, cp.scale_flag)
    x = Matrix{AffExpr}(undef, d.ns, d.n)
    u = Matrix{AffExpr}(undef, d.ns, d.n)
    for i = 1:d.n
        for j = 1:d.ns
            x[j, i] = scaled_to_unscaled(sv.xs[j,i], limits.ul.x[j], limits.ll.x[j], cp.scale_flag)
        end
        for j = 1:d.nu
            u[j, i] = scaled_to_unscaled(sv.us[j,i], limits.ul.u[j], limits.ll.u[j], cp.scale_flag)
        end
    end

    k = Vector{AffExpr}(undef, d.nk)
    if d.nk > 0
        for i = 1:d.nk
            k[i] = scaled_to_unscaled(sv.ks[i], limits.ul.k[i], limits.ll.k[i], sp.scale_flag)
        end 
    end

    xf = @view x[:, end]
    xi = @view x[:, 1]    
    
    # Some convinietnt expressions
    Δt = tf - ti
    t = @expression(model, [i=1:d.n], ti + m.tau[i]*Δt)
    h = @expression(model, [i=1:d.n-1], (t[i+1] - t[i]))

    return UnscaledVariables(tf, ti, x, u, k, xf, xi, t, h, Δt)
end

"""
CachedVars

Used to avoid expression swell in JuMP when using function tracing. It will not
be required once JuMP supports caching with function tracing
"""
mutable struct CacheVars
    f1 :: Matrix{VariableRef}
    f2 :: Matrix{VariableRef}
    xb :: Matrix{VariableRef}
    fb :: Matrix{VariableRef}
    dx :: Matrix{VariableRef}
end

function CacheVars(d::Dims, model::Model)
    f1 = @variable(model, [1:d.ns, 1:d.n-1], base_name = "f1")
    f2 = @variable(model, [1:d.ns, 1:d.n-1], base_name = "f2")
    xb = @variable(model, [1:d.ns, 1:d.n-1], base_name = "xb")
    fb = @variable(model, [1:d.ns, 1:d.n-1], base_name = "fb")
    dx = @variable(model, [1:d.ns, 1:d.n-1], base_name = "dx")

    return CacheVars(f1, f2, xb, fb, dx)
end

"""
Trajectories

Store trajectories during various phases of solving optimal
control problem.
"""
mutable struct Trajectories
    xinit::Matrix{Float64}
    uinit::Matrix{Float64}
    tfinit::Float64
    tiinit::Float64
    kinit::Vector{Float64}
end

function Trajectories(d::Dims)
    x = zeros(d.ns, d.n)
    u = zeros(d.ns, d.n)
    tf = 0.0
    ti = 0.0
    k = zeros(d.nk)

    return Trajectories(x, u, tf, ti, k)
end

"""
Constraints

Provide easy way to access the constraints
"""
mutable struct Constraints
        collocation_constraints::Vector{Vector{ConstraintRef}}
        quadrature_constraints_upper::Vector{ConstraintRef}
        quadrature_constraints_lower::Vector{ConstraintRef}
        path_constraints_upper::Vector{Vector{ConstraintRef}}
        path_constraints_lower::Vector{Vector{ConstraintRef}}
end

function Constraints(d::Dims)
    collocation_constraints = Array{Vector{ConstraintRef}, 1}(undef, d.n-1)
    path_constraints_lower = Array{Vector{ConstraintRef}, 1}(undef, d.n)
    path_constraints_upper = Array{Vector{ConstraintRef}, 1}(undef, d.n)   
    quadrature_constraints_lower = Array{ConstraintRef, 1}(undef, d.nq)
    quadrature_constraints_upper = Array{ConstraintRef, 1}(undef, d.nq)

    return Constraints(collocation_constraints, quadrature_constraints_upper, quadrature_constraints_lower, 
                        path_constraints_upper, path_constraints_lower)
end



mutable struct Phase{P <: NamedTuple, F <: AbstractFuns}
    id::Int64
    # To be taken from user   
    p::P            # Additional parameters to be stored
    funs::F         # Optimal control functions
    dims::Dims      # Dimensions of various things
    limits::Limits  # Limits on states and outputs
    config_params::ConfigParams 
    # To be validated and setup by phase constructor
    uvars:: UnscaledVariables # Optimal control unscaled variables
    svars:: ScaledVariables       # Optimal control scaled variables
    cvars:: CacheVars       # Additional variables for speeding up computation.Need to compute initial values for these variables
    init_traj::Trajectories  # Can be taken from the user. Storing the initial trajectories. Need to do this to ensure convergence 
    sval::Trajectories       # Values of scaled variables after optimization
    val::Trajectories        # Values of unscaled variables after optimization    
    mesh :: Mesh             # Mesh computation
    constraints ::Constraints
end

# Collect user info for the phase here and validate it
function Phase(id, p, funs, dims, limits, config_params, model)
   
    mesh = Mesh(dims)
    svars = ScaledVariables(dims, model)
    uvars = UnscaledVariables(dims, svars, limits, config_params, mesh, model)
    cvars = CacheVars(dims, model)
    init_traj = Trajectories(dims)
    sval = Trajectories(dims)
    val = Trajectories(dims)
    cons = Constraints(dims)

            
    return Phase(id, p, funs, dims, limits, config_params,
                 uvars, svars, cvars, init_traj, sval, val, mesh, cons)
end


mutable struct OCP
    ph::Vector{Phase}
    model::Model

    # tol::Float64
    # mesh_iter_no::Int64
    # mesh_iter_max::Int64
    # infeasible_iter_no::Int64
    # infeasible_iter_max::Int64
    # solver_status::MOI.TerminationStatusCode

    # # Variable for global parameters to be optimized
    # kg::Vector{VariableRef}
    # obj::NonlinearExpr

    # # Dimensions
    # nkg::Int64
    # npsi::Int64

    # # limits
    # psi_ulim::Vector{Float64}
    # psi_llim::Vector{Float64}
    # kg_ulim::Vector{Float64}
    # kg_llim::Vector{Float64}
    # obj_ulim::Float64
    # obj_llim::Float64

    # # Objective Sense
    # objective_sense::String
    # set_obj_lim::Bool

    # # Global constraints storage
    # event_constraints_upper::Vector{ConstraintRef}
    # event_constraints_lower::Vector{ConstraintRef}

    # # Additional constraints
    # additional_constraints_flag::Bool
    # additional_constraints::Function

    function OCP()
        x = new()
        x.model = Model(Ipopt.Optimizer)
        x.ph = Phase[]
    #     x.ph = Phase[]
    #     x.mesh_iter_no = 1
    #     x.mesh_iter_max = 5
    #     x.infeasible_iter_no = 1
    #     x.infeasible_iter_max = 5
    #     x.objective_sense = "Min"
    #     x.nkg = 0
    #     x.npsi = 0
    #     x.set_obj_lim = false
    #     x.additional_constraints_flag = false

        return x
    end
end

function Phase(ocp::OCP, p, funs, dims, limits, config_params)
    id = length(ocp.ph) + 1
    ph = Phase(id, p, funs, dims, limits, config_params, ocp.model)
    
    push!(ocp.ph, ph)

    return ph 
end