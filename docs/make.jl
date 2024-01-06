push!(LOAD_PATH, "../src/")

using Documenter
import DirectOptimalControl as DOC
using Literate


pages = [
    "Introduction" => "README.md",
    "Rocket Control" => "rocket.md",
    "Alp Rider" => "alprider.md",
    "Dubins multiple phase" => "dubins_multiple_phase.md",
    "Dubins Interception" => "dubins_interception.md",
    "Hyper sensetive problem" => "hyper_sensetive_problem.md",
    "Space craft" => "spacecraft.md",
    "Tumor antiogenesis" => "tumor_antiogenesis.md",
    "Van Der Pol Oscillator" => "van_der_pol_oscillator.md"
]

makedocs(sitename = "Direct Optimal Control", 
         pages=pages,
         format = Documenter.HTML(prettyurls = false)
         )

deploydocs(
            repo = "https://github.com/A-C1/DirectOptimalControl.jl.git"
        )
