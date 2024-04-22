using Documenter
using docs

makedocs(
    sitename = "docs",
    format = Documenter.HTML(),
    modules = [docs]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
#=deploydocs(
    repo = "<repository url>"
)=#
