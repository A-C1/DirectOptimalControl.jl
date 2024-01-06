using Test

@testset "tutorials" begin

    include("../tutorials/rocket.jl")
    @test objective_value(OC.model) ≈ 1.01282 atol = 1e-0

    include("../tutorials/alprider.jl")
    @test objective_value(OC.model) ≈ 2044.68 atol = 1e-1

    include("../tutorials/dubins_interception.jl")
    @test objective_value(OC.model) ≈ 8.490252005787678 atol = 1e-1

    include("../tutorials/dubins_multiple_phase.jl")
    @test objective_value(OC.model) ≈ 42.82990048690122 atol = 1e-1

    include("../tutorials/hyper_sensetive_problem.jl")
    @test objective_value(OC.model) ≈ 2.3308080176403445 atol = 1e-6

    include("../tutorials/tumor_antiogenesis.jl")
    @test objective_value(OC.model) ≈ 7520.22 atol = 1e-1

    include("../tutorials/van_der_pol_oscillator.jl")
    @test objective_value(OC.model) ≈ 2.9552 atol = 1e-2

    include("../tutorials/space_craft.jl")
    @test objective_value(OC.model) ≈ 0.5962 atol = 1e-2
end