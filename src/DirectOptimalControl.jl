module DirectOptimalControl

using JuMP
using BSplineKit
using QuadGK

# Code for h-methods
include("hmethods/types.jl")
include("hmethods/scaling.jl")
include("hmethods/collocation_methods.jl")
include("hmethods/integration_methods.jl")
include("hmethods/initial_trajectory_computation.jl")
include("hmethods/optimal_control_multiple_phase.jl")
include("hmethods/mesh_recomputation_phase.jl")

# Code for hp-methods
# Yet to be added. Not sure if it provides any significant
# advantage practically over h-methods. Can only be verified
# after the code is written

end # module DirectOptimalControl
