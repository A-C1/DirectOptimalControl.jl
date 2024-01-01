using Literate

files = readdir("./")

ignored_files = ["Manifest.toml", "Project.toml", "generate_literate_docs.jl"]

# Generate literate markdown files for tutorials and move them to src folder of docs
for file in files
    if !(file ∈ ignored_files) 
        file_path = "./" * file
        Literate.markdown(file_path, "../docs/src/", flavor = Literate.CommonMarkFlavor())
    end
end


# mv("../README.md","../docs/src/")

# Literate.markdown("src/README.jl", "src/manual/", documenter = true)