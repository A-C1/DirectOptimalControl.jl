push!(LOAD_PATH, "../src/")

using Documenter
import DirectOptimalControl as DOC
using Literate


pages = [
    "Introduction" => "README.md"
    "Rocket Control" => "rocket.md"
]
makedocs(sitename = "Direct Optimal Control", 
         pages=pages,
         format = Documenter.HTML(prettyurls = false)
         )

deploydocs(
            repo = "https://github.com/A-C1/DirectOptimalControl.jl.git"
        )
