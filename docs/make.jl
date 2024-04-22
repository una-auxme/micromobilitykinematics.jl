using Documenter
using docs

makedocs(
    sitename = "docs",
    format = Documenter.HTML(),
    modules = [docs],
    checkdocs=:exports,
    linkcheck=true,
    pages = Any[
            "Overview" => "index.md",
    ]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
#=deploydocs(
    repo = "<repository url>"
)=#
