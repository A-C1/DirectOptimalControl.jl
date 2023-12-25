push!(LOAD_PATH, "../src/")

using Documenter
import DirectOptimalControl as DOC
# using Literate


# Literate.markdown("src/tutorials/rocket.jl", "src/manual/", documenter = true)
# Literate.markdown("src/README.jl", "src/manual/", documenter = true)
pages = [
    "Introduction" => "README.md"
]
makedocs(sitename = "Direct Optimal Control", 
         pages=pages,
         format = Documenter.HTML(prettyurls = false)
         )
