"""
VarVals0

Struct which defines variables which have bounds
"""
mutable struct VarVals0
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

    function VarVals0()
        return new()
    end
end


mutable struct Limits0 
    ul::VarVals0
    ll::VarVals0

    function Limits0()
        limit = new()
        limit.ul = VarVals0()
        limit.ll = VarVals0()

        return limit
    end
end



mutable struct PH1
    id::Int64
    # Optimal control functions
    L::Function
    phi::Function
    dyn::Function
    pathfun::Function
    integralfun::Function

    # Dimensions of various things
    n::Int64    # Number of finite elements
    ns::Int64   # State dim
    nu::Int64   # Input dim
    nq::Int64   # Integral constraint dim
    np::Int64   # Path constraint dim
    nk::Int64   # Free parameters dim

    # Limits on states and outputs
    limits::Limits0

    # Additional parameters to be stored
    p::NamedTuple

    # Optimal control unscaled variables
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


    # Optimal control scaled variables
    tfs::VariableRef
    tis::VariableRef
    xs::Matrix{VariableRef}
    us::Matrix{VariableRef}
    ks::Vector{VariableRef}

    xfs::Vector{VariableRef}
    xis::Vector{VariableRef}

    # Storing the initial trajectories
    # Need to do this to ensure convergence
    xinit::Matrix{Float64}
    uinit::Matrix{Float64}
    tfinit::Float64
    tiinit::Float64
    kinit::Vector{Float64}


    # Storing error information
    tau::Vector{Float64}  # Initial mesh. Initialized to uniform values
    we::Vector{Float64}
    error::Vector{Float64}
    error_avg::Float64
    error_avg_hist::Vector{Float64}
    Iavgh::Vector{Int64}


    # Calculated scaled Values
    xsval::Matrix{Float64}
    usval::Matrix{Float64}
    tfsval::Float64
    tisval::Float64
    ksval::Vector{Float64}

    # Calculated Values
    xval::Matrix{Float64}
    uval::Matrix{Float64}
    tfval::Float64
    tival::Float64
    kval::Vector{Float64}

    # Create container of constraints
    collocation_constraints::Vector{Vector{ConstraintRef}}
    quadrature_constraints_upper::Vector{ConstraintRef}
    quadrature_constraints_lower::Vector{ConstraintRef}
    path_constraints_upper::Vector{Vector{ConstraintRef}}
    path_constraints_lower::Vector{Vector{ConstraintRef}}

    # collocation_method
    collocation_method::String
    order::Int64

    # Additional flags
    set_initial_vals::String 
    scale_flag::Bool

    # Callback function and data
    callback_fun::Function
    callback_nt::NamedTuple

    function PH1()
        ph = new()
        ph.limits = Limits0()

        ph.n = 0
        ph.ns = 0    
        ph.nu = 0
        ph.nq = 0
        ph.np = 0
        ph.nk = 0

        ph.collocation_method = "trapezoidal"

        ph.error_avg_hist = Float64[]
        ph.Iavgh = Int64[]
        ph.set_initial_vals = "Auto"

        return ph
    end
end


mutable struct OCP1
    ph::Vector{PH1}
    model::Model
    psi::Function

    tol::Float64
    mesh_iter_no::Int64
    mesh_iter_max::Int64
    infeasible_iter_no::Int64
    infeasible_iter_max::Int64
    solver_status::MOI.TerminationStatusCode

    # Variable for global parameters to be optimized
    kg::Vector{VariableRef}
    obj::NonlinearExpr

    # Dimensions
    nkg::Int64
    npsi::Int64

    # limits
    psi_ulim::Vector{Float64}
    psi_llim::Vector{Float64}
    kg_ulim::Vector{Float64}
    kg_llim::Vector{Float64}
    obj_ulim::Float64
    obj_llim::Float64

    # Objective Sense
    objective_sense::String
    set_obj_lim::Bool

    # Global constraints storage
    event_constraints_upper::Vector{ConstraintRef}
    event_constraints_lower::Vector{ConstraintRef}

    function OCP1()
        x = new()
        x.model = Model()
        x.ph = PH1[]
        x.mesh_iter_no = 1
        x.mesh_iter_max = 5
        x.infeasible_iter_no = 1
        x.infeasible_iter_max = 5
        x.objective_sense = "Min"
        x.nkg = 0
        x.npsi = 0
        x.set_obj_lim = false

        return x
    end
end

function PH1(ocp::OCP1)
    ph = PH1()
    ph.id = length(ocp.ph) + 1
    push!(ocp.ph, ph)

    return ph
end

Limits = Limits0
VarVals = VarVals0
PH = PH1
OCP = OCP1