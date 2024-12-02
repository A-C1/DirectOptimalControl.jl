module DirectOptimalControl

using JuMP
import Ipopt
using BSplineKit
using QuadGK
using Infiltrator
import MathOptInterface

# Code for h-methods
include("hmethods/scaling.jl")
include("hmethods/types.jl")
include("hmethods/collocation_methods.jl")
include("hmethods/integration_methods.jl")
include("hmethods/single_phase.jl")
include("hmethods/initial_trajectory_computation.jl")
include("hmethods/optimal_control_multiple_phase.jl")
include("hmethods/mesh_recomputation_phase.jl")
# include("hmethods/adjoint_computation.jl")

# Code for hp-methods
# Yet to be added. Not sure if it provides any significant
# advantage practically over h-methods. Can only be verified
# after the code is written

end # module DirectOptimalControl
