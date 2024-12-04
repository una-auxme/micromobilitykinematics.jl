import Pkg; Pkg.develop(path=joinpath(@__DIR__,"../../micromobilitykinematics.jl"))
using Documenter, micromobilitykinematics
using Documenter
using Documenter: GitHubActions

makedocs(
    format = Documenter.HTML(
        collapselevel = 1,
        sidebar_sitename = false,
        edit_link = nothing,
        size_threshold = 512000,
     ),
    checkdocs=:exports,
    linkcheck=true,
    pages = Any[
            "Overview" => "index.md",
            "Examples" => Any[
                            "Changing steering position" => "Examples/changing_steering_position.md",
                            "Optimising the steering" => "Examples/optimising_the_steering.md",
                            ],

            "Function Library" => "library.md"
    ]
)

function deployConfig()
    github_repository = get(ENV, "GITHUB_REPOSITORY", "")
    github_event_name = get(ENV, "GITHUB_EVENT_NAME", "")
    if github_event_name == "workflow_run"
        github_event_name = "push"
    end
    github_ref = get(ENV, "GITHUB_REF", "")
    return GitHubActions(github_repository, github_event_name, github_ref)
end

deploydocs(repo = "github.com/adribrune/micromobilitykinematics.jl.git", devbranch = "main" , deploy_config = deployConfig())