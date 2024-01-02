push!(LOAD_PATH, "../src/")

using Documenter
import DirectOptimalControl as DOC
using Literate


pages = [
    "Home" => "index.md",
    "Introduction" => "README.md",
    "Rocket Control" => "rocket.md",
    "Alp Rider" => "alprider.md",
    "Dubins multiple phase" => "dubins_multiple_phase.md",
    "Hyper sensetive problem" => "hyper_sensetive_problem.md"
    "Space craft" => "spacecraft.md"
    "Tumor antiogenesis" => "tumor_antiogenesis.md"
]

makedocs(sitename = "Direct Optimal Control", 
         pages=pages,
         format = Documenter.HTML(prettyurls = false)
         )

deploydocs(
            repo = "https://github.com/A-C1/DirectOptimalControl.jl.git"
        )
