
"""
    get_insights(steering::Steering)


"""
function get_insights(steering::Steering)
    # Extrahiere die benötigten Werte aus dem Steering-Objekt
    θx = steering.θx
    θz = steering.θz
    δi = steering.δi
    δo = steering.δo


    x_rotational_radius = steering.rotational_component.x_rotational_radius
    z_rotational_radius = steering.rotational_component.z_rotational_radius

    track_lever_length = steering.track_lever.length
    tie_rod_length = steering.tie_rod.length

    # Erstelle eine Tabelle (DataFrame)
    df = DataFrame(
        Parameter = ["θx", "θz", "δi", "δo", "x_rotational_radius", "z_rotational_radius", "track_lever.length", "tie_rod.length"],
        Value = [θx, θz, δi, δo, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length]
    )

    return df
end 



function get_components_lengths(steering::Steering)

    x_rotational_radius = steering.rotational_component.x_rotational_radius
    z_rotational_radius = steering.rotational_component.z_rotational_radius

    track_lever_length = steering.track_lever.length
    tie_rod_length = steering.tie_rod.length

    return x_rotational_radius,z_rotational_radius,track_lever_length,tie_rod_length

end



"""
    grid_data_obj(θ_max::Tuple{I,I}, steering::Steering, suspension::Suspension, chassi::Chassis; step_size = 1) where {I <: Any}

calculates the objective of the given 

"""
function grid_data_obj(θ_max::Tuple{Any,Any}, steering::Steering, suspension::Suspension, chassi::Chassis; step_size = 1) 
    θx_max, θz_max = θ_max
    
    
    θ_tuple = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    objective = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]

    for θ in θ_tuple
        if θ == (0,0)
            continue
        end
        θx, θz  = θ
        objective[θx+1,θz+1] = steering_objective(((θx, θz)), chassi, steering, suspension)

    end

    return objective
end


"""
    grid_data_δ(θ_max::Tuple{I,I}, steering::Steering; step_size = 1) where {I <: Any}

Calculates the turning angles of the vehicle for each turning angles of the rotation component (θx, θz) of the steering system.

# Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.

# Keywords
-`step_size`: The step size of the grid

# Retruns
-`δi, δo`: Tuple of turning angles Matrix 


"""
function grid_data_δ(θ_max::Tuple{Any,Any}, steering::Steering, suspension::Suspension; step_size = 1) 
    θx_max, θz_max = θ_max
    
    
    θ_tuple = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    δi  = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    δo  = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]

    for θ in θ_tuple
        if θ == (0,0)
            continue
        end
        θx, θz  = θ
        temp_steering = update(((θx, θz)), steering, suspension)
        δi[θx+1,θz+1] = temp_steering.δi 
        δo[θx+1,θz+1] = temp_steering.δo 
    end
    return δi, δo
end





"""
    plot_optda_series(optda_dict::Dict{Int64,Any})
    

"""
function plot_optda_series(optda_dict::Dict{Int64,Any})

    #
    keys = []
    param = []
    objective = []
    δo = []


    for (key,optda) in optda_dict 
        push!(δo,optda.steering.δo)
        push!(objective,optda.objective)
        push!(param, [get_components_lengths(optda.steering)...])
        push!(keys,key)
    end 

    # 
    println(keys)
    plt_param = Plots.plot(title = "components lengths ", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))
    plt_objective = Plots.plot(title = "objective", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))  
    plt_δo = Plots.plot(title = "turning angle", xlabel = "set ID", ylabel = "[°]", legend = true, background_color_legend = RGBA(255,255,255, .01))

    
    Plots.scatter!(plt_objective, keys, objective)
    Plots.scatter!(plt_δo , keys, δo)

    repeat_keys = [[i for _ in 1:length(param[i])] for i in 1:length(param)]
    Plots.scatter!(plt_param,repeat_keys,param )
    plt_merge = Plots.plot(plt_objective,plt_δo,plt_param ,layout = (3, 1),size = (500, 900))


    display(plt_merge)
    nothing
end


"""
    plot_optda_gird_obj(θ_max::Tuple{I,I}, steering::Steering, suspension::Suspension, chassi::Chassis) where {I <: Any}

Create a plot in which the objective (Ackermann ratio) are mapped to the turning angles of the rotation component (θx, θz) of the steering system.   

# Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.
-`suspension::Suspension`: Instance of a specific suspension in which the kinematics were previously calculated.
-`chassi::Chassis`: Instance of a Chassis

# Returns

"""
function plot_optda_gird_obj(args...)
    θx_max, θz_max = args[1]
    θx = 0:1:θx_max
    θz = 0:1:θz_max

    objective = grid_data_obj(args...)

    sur_obj = PlotlyJS.surface(;x=θx, y=θz, z=objective)
    #title="Lenkabweichung in Abhängigkeit der Stellwinkel (θx, θz) der Rotationskomponente der Lenkgeometrie"
    layout = Layout(autosize=true, margin=attr(l=10, r=10, b=10, t=10),
    scene=attr(
        xaxis=attr(title="θx in [°]"),
        yaxis=attr(title="θz in [°]"),
        zaxis=attr(title="objective in [mm]"),
        scene = attr(aspectmode="cube") 
    ),
    width=600,
    height=600,
    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4)
    )

    PlotlyJS.plot(sur_obj, layout)
end



"""


"""
function plotSuspImpact(θ_max::Tuple{Any,Any}, steering::Steering, suspension::Suspension, chassis::Chassis; step_size = 1, suspensionNEUTRAL = Suspension((30,30)))
    θx_max, θz_max = θ_max
    θx = 0:1:θx_max
    θz = 0:1:θz_max

    objectiveNEUTRAL = grid_data_obj(θ_max, steering, suspensionNEUTRAL, chassis)
    objective = grid_data_obj(θ_max, steering, suspension, chassis)

    δi_NEUTRAL, δo_NEUTRAL  = grid_data_δ(θ_max, steering, suspensionNEUTRAL)
    δi, δo  = grid_data_δ(θ_max, steering, suspension)

    #############################################################################
    sur_objNEUTRAL = PlotlyJS.surface(;
                                    x=θx, 
                                    y=θz, 
                                    z=objectiveNEUTRAL, 
                                    name="Neutral",
                                    colorscale="Greys", 
                                    opacity=0.8,
                                    colorbar=attr(
                                                title="Neutral",   # Titel der Farbskala
                                                x=1.05,            # Position der Farbskala (rechts neben dem Plot)
                                                y=0.5,             # Vertikale Position
                                                thickness=20,      # Dicke der Farbskala
                                                len=0.7            # Länge der Farbskala
                                                    ))
    sur_obj = PlotlyJS.surface(;
                            x=θx, 
                            y=θz, 
                            z=objective, 
                            name="Current",
                            colorscale="YlOrRd", 
                            opacity=0.5,
                            colorbar=attr(
                                        title="Current",  # Titel der Farbskala
                                        x=1.2,           # Position der Farbskala (weiter rechts)
                                        y=0.5,            # Vertikale Position
                                        thickness=20,     # Dicke der Farbskala
                                        len=0.7           # Länge der Farbskala
                                            ))

#############################################################################
    sur_δi_NEUTRAL = PlotlyJS.surface(;
                                    x=θx, 
                                    y=θz, 
                                    z=δi_NEUTRAL, 
                                    name="Neutral",
                                    colorscale="Greys", 
                                    opacity=0.8,
                                    colorbar=attr(
                                                title="Neutral",   # Titel der Farbskala
                                                x=1.05,            # Position der Farbskala (rechts neben dem Plot)
                                                y=0.5,             # Vertikale Position
                                                thickness=20,      # Dicke der Farbskala
                                                len=0.7            # Länge der Farbskala
                                                    ))

    sur_δi = PlotlyJS.surface(;
                            x=θx, 
                            y=θz, 
                            z=δi, 
                            name="Current",
                            colorscale="YlOrRd", 
                            opacity=0.5,
                            colorbar=attr(
                                        title="Current",  # Titel der Farbskala
                                        x=1.2,           # Position der Farbskala (weiter rechts)
                                        y=0.5,            # Vertikale Position
                                        thickness=20,     # Dicke der Farbskala
                                        len=0.7           # Länge der Farbskala
                                            ))


#############################################################################
    #title="Lenkabweichung in Abhängigkeit der Stellwinkel (θx, θz) der Rotationskomponente der Lenkgeometrie"
    layout1 = Layout(autosize=true, 
                    margin=attr(l=10, r=10, b=10, t=10),
                    scene=attr(
                        xaxis=attr(title="θx in [°]"),
                        yaxis=attr(title="θz in [°]"),
                        zaxis=attr(title="objective in [mm]"),
                        scene = attr(aspectmode="cube")),
                    width=600,
                    height=600,
                    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4),
                    )

    layout2 = Layout(autosize=true, 
                    margin=attr(l=10, r=10, b=10, t=10),
                    scene=attr(
                        xaxis=attr(title="θx in [°]"),
                        yaxis=attr(title="θz in [°]"),
                        zaxis=attr(title="δ in [°]"),
                        scene = attr(aspectmode="cube")),
                    width=600,
                    height=600,
                    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4),
                    )

    plot_objective = PlotlyJS.plot([sur_objNEUTRAL, sur_obj], layout1)
    plot_δi = PlotlyJS.plot([sur_δi_NEUTRAL, sur_δi], layout2)

    display(plot_objective)
    display(plot_δi)
end


function plotSuspImpact2(θ_max::Tuple{Any,Any}, steering::Steering, suspension::Suspension, chassis::Chassis; step_size = 1, suspensionNEUTRAL = Suspension((30,30)))
    θx_max, θz_max = θ_max
    θx = 0:step_size:θx_max
    θz = 0:step_size:θz_max

    # Meshgrid erzeugen für x und y
    θx_grid = repeat(θx', length(θz), 1)
    θz_grid = repeat(θz, 1, length(θx))

    # Daten berechnen
    objectiveNEUTRAL = grid_data_obj(θ_max, steering, suspensionNEUTRAL, chassis)
    objective = grid_data_obj(θ_max, steering, suspension, chassis)

    δi_NEUTRAL, δo_NEUTRAL  = grid_data_δ(θ_max, steering, suspensionNEUTRAL)
    δi, δo  = grid_data_δ(θ_max, steering, suspension)

    # Plot 1: Objective Comparison
    fig1 = Figure(resolution = (800, 600))
    ax1 = Axis3(fig1[1, 1], title = "Objective", xlabel = "θx [°]", ylabel = "θz [°]", zlabel = "Objective [mm]")

    surface!(ax1, θx, θz, objectiveNEUTRAL; colormap = :greys, transparency = true, alpha = 0.8)
    surface!(ax1, θx, θz, objective; colormap = :ylorrd, transparency = true, alpha = 0.5)

    fig1[1, 1] = ax1

    # Plot 2: δi Comparison
    fig2 = Figure(resolution = (800, 600))
    ax2 = Axis3(fig2[1, 1], title = "Lenkabweichung δi", xlabel = "θx [°]", ylabel = "θz [°]", zlabel = "δ [°]")

    surface!(ax2, θx, θz, δi_NEUTRAL; colormap = :greys, transparency = true, alpha = 0.8)
    surface!(ax2, θx, θz, δi; colormap = :ylorrd, transparency = true, alpha = 0.5)

    fig2[1, 1] = ax2

    display(fig1)
    display(fig2)
end





"""
    plot_optda_gird_δi(θ_max::Tuple{I,I}, steering::Steering) where {T <: Any}
    	
Create a plot in which the turning angles of the vehicle are mapped to the turning angles of the rotation component (θx, θz) of the steering system.

# Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.

# Returns


"""
function plot_optda_gird_δ(args...)
    θx_max, θz_max = args[1]
    θx = 0:1:θx_max
    θz = 0:1:θz_max


    δi, δo  = grid_data_δ(args...)

    sur_δi = PlotlyJS.surface(;x=θx, y=θz, z=δi)
    sur_δo = PlotlyJS.surface(;x=θx, y=θz, z=δo)

    #title="Lenkabweichung in Abhängigkeit der Stellwinkel (θx, θz) der Rotationskomponente der Lenkgeometrie"
    layout = Layout(autosize=true, margin=attr(l=10, r=10, b=10, t=10),
    scene=attr(
        xaxis=attr(title="θx in [°]"),
        yaxis=attr(title="θz in [°]"),
        zaxis=attr(title="objective in [mm]"),
        scene = attr(aspectmode="cube") 
    ),
    width=600,
    height=600,
    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4)
    )

    PlotlyJS.plot([sur_δi, sur_δo], layout)
end

"""
    plot_steering(steering::Steering)

creates a 3D vector plot showing the kinematic chain of the steering system

# Arguments
- `steering::Steering`: instance of a steering system for which the kinematics have already been calculated


"""
function plot_steering(steering::Steering)


    ######## RotationalComponent

    # Steering UCS position
    rotcomp1 = PlotlyJS.scatter3d(
        x=[0],  # x-Koordinaten
        y=[0],  # y-Koordinaten
        z=[0],  # z-Koordinaten
        mode="markers+text",  # Linie, Punkte und Text
        text="Steering UCS",         # Beschriftung für beide Punkte
        marker=attr(size=6, color="black"),
        showlegend=false
    )


    ### x rotation 
    # point
    rotcomp2 = PlotlyJS.scatter3d(
        x=[steering.vec_x_rotational[1]],  
        y=[steering.vec_x_rotational[2]],  
        z=[steering.vec_x_rotational[3]],  
        mode="markers+text",  
        text="rotational component",         
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    rotcomp3 = PlotlyJS.scatter3d(
        x=[0, steering.vec_x_rotational[1]],  
        y=[0, steering.vec_x_rotational[2]],  
        z=[0, steering.vec_x_rotational[3]],  
        mode="lines",    
        line=attr(width=4, color="red"),  
        name= "x rotation"
    )
    
    ### z rotaion
    # point
     rotcomp4 = PlotlyJS.scatter3d(
        x=[steering.vec_z_rotational[1]],  # x-Koordinaten
        y=[steering.vec_z_rotational[2]],  # y-Koordinaten
        z=[steering.vec_z_rotational[3]],  # z-Koordinaten
        mode="markers+text",  
        text="rotational component",        
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    rotcomp5 = PlotlyJS.scatter3d(
        x=[steering.vec_x_rotational[1], steering.vec_z_rotational[1]],  # x-Koordinaten
        y=[steering.vec_x_rotational[2], steering.vec_z_rotational[2]],  # y-Koordinaten
        z=[steering.vec_x_rotational[3], steering.vec_z_rotational[3]],  # z-Koordinaten
        mode="lines",  # Linie, Punkte und Text
        line=attr(width=4, color="blue"),  
        name= "z rotaion"
    )

    ### spher joints left
    # point
     rotcomp6 = PlotlyJS.scatter3d(
        x=[steering.sphere_joints[1][1]],  # x-Koordinaten
        y=[steering.sphere_joints[1][2]],  # y-Koordinaten
        z=[steering.sphere_joints[1][3]],  # z-Koordinaten
        mode="markers+text",  
        text="sphere joint",        
        marker=attr(size=6, color="red"),  
        showlegend=false
    )
    # vector
    rotcomp7 = PlotlyJS.scatter3d(
        x=[steering.vec_z_rotational[1], steering.sphere_joints[1][1]],  # x-Koordinaten
        y=[steering.vec_z_rotational[2], steering.sphere_joints[1][2]],  # y-Koordinaten
        z=[steering.vec_z_rotational[3], steering.sphere_joints[1][3]],  # z-Koordinaten
        mode="lines",  
        line=attr(width=4, color="green"), 
        name= "spher joints"
    )


    ### spher joints right
    # point
     rotcomp8 = PlotlyJS.scatter3d(
        x=[steering.sphere_joints[2][1]],  # x-Koordinaten
        y=[steering.sphere_joints[2][2]],  # y-Koordinaten
        z=[steering.sphere_joints[2][3]],  # z-Koordinaten
        mode="markers+text",  
        text="sphere joint",
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    rotcomp9 = PlotlyJS.scatter3d(
        x=[steering.vec_z_rotational[1], steering.sphere_joints[2][1]],  # x-Koordinaten
        y=[steering.vec_z_rotational[2], steering.sphere_joints[2][2]],  # y-Koordinaten
        z=[steering.vec_z_rotational[3], steering.sphere_joints[2][3]],  # z-Koordinaten
        mode="lines",       
        line=attr(width=4, color="green"), 
        name= "spher joints ",
        showlegend=false
    )

    rotcomp = [rotcomp1, rotcomp2, rotcomp3, rotcomp4, rotcomp5, rotcomp6, rotcomp7, rotcomp8, rotcomp9]

    ######## Tie Rod

    ### left
    tierod1 = PlotlyJS.scatter3d(
        x=[steering.sphere_joints[1][1], steering.circle_joints[1][1]],  # x-Koordinaten
        y=[steering.sphere_joints[1][2], steering.circle_joints[1][2]],  # y-Koordinaten
        z=[steering.sphere_joints[1][3], steering.circle_joints[1][3]],  # z-Koordinaten
        mode="lines",  
        line=attr(width=4, color="purple"),  
        name= "tie rod"
    )

    ### right 
    tierod2 = PlotlyJS.scatter3d(
        x=[steering.sphere_joints[2][1], steering.circle_joints[2][1]],  # x-Koordinaten
        y=[steering.sphere_joints[2][2], steering.circle_joints[2][2]],  # y-Koordinaten
        z=[steering.sphere_joints[2][3], steering.circle_joints[2][3]],  # z-Koordinaten
        mode="lines",  
        line=attr(width=4, color="purple"),  
        name= "tie rod",
        showlegend=false
    )


    tierod = [tierod1, tierod2]
    ######## Suspension

    ### wishbone left
    # point
    wishbone1 = PlotlyJS.scatter3d(
        x=[steering.wishbone_ucs_position[1][1]],  # x-Koordinaten
        y=[steering.wishbone_ucs_position[1][2]],  # y-Koordinaten
        z=[steering.wishbone_ucs_position[1][3]],  # z-Koordinaten
        mode="markers+text",  # Linie, Punkte und Text
        text="wishbone UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )

    ### wishbone right
    # point
    wishbone2 = PlotlyJS.scatter3d(
        x=[steering.wishbone_ucs_position[2][1]],  # x-Koordinaten
        y=[steering.wishbone_ucs_position[2][2]],  # y-Koordinaten
        z=[steering.wishbone_ucs_position[2][3]],  # z-Koordinaten
        mode="markers+text",  # Linie, Punkte und Text
        text="wishbone UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )

    wishbone = [wishbone1, wishbone2]


    ######## Wheel Mount

    ### wheel UCS left
    # point
    vec_left = steering.wishbone_ucs_position[1] + steering.wheel_ucs_position[1]
    wheel1 = PlotlyJS.scatter3d(
        x=[vec_left[1]],  # x-Koordinaten
        y=[vec_left[2]],  # y-Koordinaten
        z=[vec_left[3]],  # z-Koordinaten
        mode="markers+text",  
        text="wheel UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )

    ### wheel UCS right
    # point
    println(steering.wheel_ucs_position[2])
    vec_right = steering.wishbone_ucs_position[2] + steering.wheel_ucs_position[2] .* [1,-1,1]
    wheel2 = PlotlyJS.scatter3d(
        x=[vec_right[1]],  # x-Koordinaten
        y=[vec_right[2]],  # y-Koordinaten
        z=[vec_right[3]],  # z-Koordinaten
        mode="markers+text",  
        text="wheel UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )

    wheel = [wheel1, wheel2]

    ######## Track lever mounting

    ###### LEFT
    ### track_lever_mounting_points_ucs
    # point
    tracklever1 = PlotlyJS.scatter3d(
        x=[steering.track_lever_mounting_points_ucs[1][1]],  # x-Koordinaten
        y=[steering.track_lever_mounting_points_ucs[1][2]],  # y-Koordinaten
        z=[steering.track_lever_mounting_points_ucs[1][3]],  # z-Koordinaten
        mode="markers+text",  
        text="track lever mounting UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    tracklever2 = PlotlyJS.scatter3d(
        x=[vec_left[1], steering.track_lever_mounting_points_ucs[1][1]],  # x-Koordinaten
        y=[vec_left[2], steering.track_lever_mounting_points_ucs[1][2]],  # y-Koordinaten
        z=[vec_left[3], steering.track_lever_mounting_points_ucs[1][3]],  # z-Koordinaten
        mode="lines",       
        line=attr(width=4, color="gray"), 
        name= "offset"
    )

    ### circle_joints
    # point 
     tracklever3 = PlotlyJS.scatter3d(
        x=[steering.circle_joints[1][1]],  # x-Koordinaten
        y=[steering.circle_joints[1][2]],  # y-Koordinaten
        z=[steering.circle_joints[1][3]],  # z-Koordinaten
        mode="markers+text",  
        text="circle joint",
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    tracklever4 = PlotlyJS.scatter3d(
        x=[steering.track_lever_mounting_points_ucs[1][1], steering.circle_joints[1][1]],  # x-Koordinaten
        y=[steering.track_lever_mounting_points_ucs[1][2], steering.circle_joints[1][2]],  # y-Koordinaten
        z=[steering.track_lever_mounting_points_ucs[1][3], steering.circle_joints[1][3]],  # z-Koordinaten
        mode="lines",       
        line=attr(width=4, color="orange"), 
        name= "track lever"
    )

    ###### RIGHT
    ### track_lever_mounting_points_ucs
    # point
    tracklever5 = PlotlyJS.scatter3d(
        x=[steering.track_lever_mounting_points_ucs[2][1]],  # x-Koordinaten
        y=[steering.track_lever_mounting_points_ucs[2][2]],  # y-Koordinaten
        z=[steering.track_lever_mounting_points_ucs[2][3]],  # z-Koordinaten
        mode="markers+text",  
        text="track lever mounting UCS",
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    tracklever6 = PlotlyJS.scatter3d(
        x=[vec_right[1], steering.track_lever_mounting_points_ucs[2][1]],  # x-Koordinaten
        y=[vec_right[2], steering.track_lever_mounting_points_ucs[2][2]],  # y-Koordinaten
        z=[vec_right[3], steering.track_lever_mounting_points_ucs[2][3]],  # z-Koordinaten
        mode="lines",       
        line=attr(width=4, color="gray"), 
        name= "offset",
        showlegend=false
    )

    ### circle_joints
    # point 
     tracklever7 = PlotlyJS.scatter3d(
        x=[steering.circle_joints[2][1]],  # x-Koordinaten
        y=[steering.circle_joints[2][2]],  # y-Koordinaten
        z=[steering.circle_joints[2][3]],  # z-Koordinaten
        mode="markers+text",  
        text="circle joint",
        marker=attr(size=6, color="red"),
        showlegend=false
    )
    # vector
    tracklever8 = PlotlyJS.scatter3d(
        x=[steering.track_lever_mounting_points_ucs[2][1], steering.circle_joints[2][1]],  # x-Koordinaten
        y=[steering.track_lever_mounting_points_ucs[2][2], steering.circle_joints[2][2]],  # y-Koordinaten
        z=[steering.track_lever_mounting_points_ucs[2][3], steering.circle_joints[2][3]],  # z-Koordinaten
        mode="lines",       
        line=attr(width=4, color="orange"), 
        name= "track lever",
        showlegend=false
    )
    tracklever = [tracklever1, tracklever2, tracklever3, tracklever4, tracklever5, tracklever6, tracklever7, tracklever8]
    
    # Erstelle das Layout für den 3D-Plot
    layout = Layout(
        title="3D-Vektor Plot",
        scene=attr(
            xaxis=attr(
                title="X-Achse",
                range=[100, -200]  # Bereich der X-Achse
            ),
            yaxis=attr(
                title="Y-Achse",
                range=[-400, 400]   # Bereich der Y-Achse
            ),
            zaxis=attr(
                title="Z-Achse",
                range=[-300, 50]    # Bereich der Z-Achse
            )
        )
    )
    
    # Kombiniere den Trace und das Layout und zeige den Plot an
    return PlotlyJS.plot([rotcomp...,tierod...,wishbone...,wheel...,tracklever...], layout)
end