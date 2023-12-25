push!(LOAD_PATH, "../src/")

using Documenter
import DirectOptimalControl as DOC
using Literate


# Literate.markdown("../tutorials/rocket.jl", "src/", flavor = Literate.CommonMarkFlavor())
# Literate.markdown("src/README.jl", "src/manual/", documenter = true)
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
