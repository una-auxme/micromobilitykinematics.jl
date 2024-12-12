import Pkg; Pkg.develop(path=joinpath(@__DIR__,"../../micromobilitykinematics.jl"))
using Documenter, micromobilitykinematics
using Documenter
using Documenter: GitHubActions


makedocs(sitename="micromobilitykinematics.jl",
    format = Documenter.HTML(
        collapselevel = 1,
        sidebar_sitename = false,
        edit_link = nothing,
        size_threshold = 512000,
     ),
    checkdocs=:exports,
    linkcheck=true,
    pages = [
            "Introduction" => "index.md",
            "About the Vehicle" => [
                                    "Steering geometry of the vehicle" => ["Kinematic of the steering geometry" => "kinematic.md",
                                                                            "Optimisation problem of the steering geometry " => "optimisation_problem.md"],

            ],
            "Examples" => [
                            "Changing steering position" => joinpath("examples", "changing_steering_position.md"),
                            "Optimizing the steering" => joinpath("examples", "optimizing_the_steering.md"),
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

deploydocs(repo = "https://github.com/una-auxme/micromobilitykinematics.jl", devbranch = "main" , deploy_config = deployConfig())

