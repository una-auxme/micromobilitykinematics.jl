





function GUI_steering(steering)

    
    # Figure
    fig = GLMakie.Figure(resolution = (1200, 1200))



    #######################################################| Layout 
    ############| Layout 3D Axis
    ax = GLMakie.Axis3(fig[1:2, 1:3])
    ax.aspect = :data

    # Limits
    GLMakie.xlims!(ax, -200, 50)
    GLMakie.ylims!(ax, -300, 300)
    GLMakie.zlims!(ax, -200, 50)

    #############| Layout Angles 
    angle_layout = GridLayout(tellheight = false)
    fig[3,1] = angle_layout

    titel = Label(angle_layout[1,1], "Rotation angle configuration", fontsize = 15)


    #sl_θz = Slider(angle_layout[2, 1], range = -30:1:30, startvalue = 0)
    sl_θ = SliderGrid(
                    angle_layout[2, 1],
                    (label = "θx", range = 0:1:15, format = "{:.1f}°", startvalue = 0),
                    (label = "θy", range = 0:1:15, format = "{:.1f}°", startvalue = 0),
                    (label = "θz", range = 0:1:30, format = "{:.1f}°", startvalue = 0),
                    width = 350,
                    tellheight = false)

    rowgap!(angle_layout, 1, -200)
    


    ############# Parameter Layout 
    #param_layout = GridLayout(tellheight = false)
    #fig[3,2] = param_layout

    #titel = Label(param_layout[1,1], "Components configuration", fontsize = 15)

    #sg2 = SliderGrid(
    #                param_layout[2, 1],
    #                (label = "θx radius", range = 50:1:100, format = "{:.1f}mm", startvalue = 0),
    #                (label = "θz radius", range = 50:1:200, format = "{:.1f}mm", startvalue = 0),
    #                (label = "track lever", range = 70:1:200, format = "{:.1f}mm", startvalue = 0),
    #                (label = "tie rod", range = 195:1:260, format = "{:.1f}mm", startvalue = 0),
    #                width = 350,
    #                tellheight = false)


    #rowgap!(param_layout, 1, -165)

    #############| Layout Actions            
    action_layout = GridLayout(tellheight = false)
    fig[3, 2] = action_layout  # Sub-Layout einfügen

    titel1 = Label(action_layout[1,1], "Actions", fontsize = 15)

    # Füge mehrere Buttons in das verschachtelte Layout ein:

    button_layout = GridLayout(tellheight = false)
    action_layout[2,1] = button_layout

    button1 = Button(button_layout[1, 1], label = "Reset")
    button2 = Button(button_layout[2, 1], label = "")
    button3 = Button(button_layout[3, 1], label = "")

    rowgap!(action_layout, 1, -165)
    

    #############| Layout Infos  
    info_layout = GridLayout(tellheight = false)
    fig[3, 3] = info_layout  # Sub-Layout einfügen

    titel2 = Label(info_layout[1,1], "Information", fontsize = 15)

    # Füge mehrere Buttons in das verschachtelte Layout ein:

    text_layout = GridLayout(tellheight = false)
    info_layout[2,1] = text_layout

    tb_obj = Textbox(text_layout[1, 1], placeholder = "objective: ",width = 300)
    tb_ratio = Textbox(text_layout[2, 1], placeholder = "ackermannratio: ",width = 300)
    tb_δi = Textbox(text_layout[3, 1], placeholder = "δi: ",width = 200)
    tb_δo = Textbox(text_layout[4, 1], placeholder = "δo: ",width = 200)

    rowgap!(info_layout, 1, -140)




    colsize!(fig.layout, 1, Relative(0.3))  # Spalte 1 = 30%
    colsize!(fig.layout, 2, Relative(0.3))  # Spalte 2 = 30%
    colsize!(fig.layout, 3, Relative(0.3))  # Spalte 3 = 30%




    #######################################################|  Observation
    #############| Define Observable
    rotational_coponent = [Point3f([0,0,0]),
                            Point3f(steering.vec_x_rotational...),
                            Point3f(steering.vec_z_rotational...)]
    left_steering_connections = [Point3f(steering.vec_z_rotational...),
                                    Point3f(steering.sphere_joints[1]...),
                                    Point3f(steering.circle_joints[1]...),
                                    Point3f(steering.track_lever_mounting_points_ucs[1]...)]

    right_steering_connections = [Point3f(steering.vec_z_rotational...),
                                    Point3f(steering.sphere_joints[2]...),
                                    Point3f(steering.circle_joints[2]...),
                                    Point3f(steering.track_lever_mounting_points_ucs[2]...)]

    # stationary
    stationary = [Point3f(steering.wishbone_ucs_position[1]...),
                    Point3f(steering.wishbone_ucs_position[2]...)]        


    observe_rot = Observable(rotational_coponent)
    observe_left = Observable(left_steering_connections)
    observe_right = Observable(right_steering_connections)
    observe_stationary = Observable(stationary)


    #############| Ploting 
    GLMakie.scatter!(ax, observe_rot, markersize=10)
    GLMakie.scatter!(ax, observe_left, markersize=10)
    GLMakie.scatter!(ax, observe_right, markersize=10)
    GLMakie.scatter!(ax, observe_stationary, markersize=10)

    GLMakie.lines!(ax, observe_rot)
    GLMakie.lines!(ax, observe_left)
    GLMakie.lines!(ax, observe_right)



    #######################################################|  Updating Functions
    #############| Updating Geometry
    function update_geometry!(θ)
        steeringkinematicsMOVED!(θ, steering, suspension)
        # 
        rotational_coponent = [Point3f([0,0,0]),
                                Point3f(steering.vec_x_rotational...),
                                Point3f(steering.vec_z_rotational...)]

        left_steering_connections = [Point3f(steering.vec_z_rotational...),
                                        Point3f(steering.sphere_joints[1]...),
                                        Point3f(steering.circle_joints[1]...),
                                        Point3f(steering.track_lever_mounting_points_ucs[1]...)]

        right_steering_connections = [Point3f(steering.vec_z_rotational...),
                                        Point3f(steering.sphere_joints[2]...),
                                        Point3f(steering.circle_joints[2]...),
                                        Point3f(steering.track_lever_mounting_points_ucs[2]...)]

        # stationary
        stationary = [Point3f(steering.wishbone_ucs_position[1]...),
                        Point3f(steering.wishbone_ucs_position[2]...)]        

        observe_rot[] = rotational_coponent
        observe_left[] = left_steering_connections
        observe_right[] = right_steering_connections
        observe_stationary[] = stationary
    end 





    #######################################################|  Aktion    
    #############| Layout Angles 
    on(sl_θ.sliders[1].value) do val
        θx = val
        θy = sl_θ.sliders[2].value.val
        θz = sl_θ.sliders[3].value.val
        update_geometry!((θx,θy,θz))
        obj = steering_objective((θx,θy,θz),chassis,steering,suspension)
        w = 1300
        L = obj + w 
        ratio = (w / L) * 100
        tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        tb_δo.displayed_string = "δi: $(round(steering.δo, digits=2))°"
    end

    on(sl_θ.sliders[2].value) do val
        θx = sl_θ.sliders[1].value.val
        θy = val
        θz = sl_θ.sliders[3].value.val
        update_geometry!((θx,θy,θz))
        obj = steering_objective((θx,θy,θz),chassis,steering,suspension)
        w = 1300
        L = obj + w 
        ratio = (w / L) * 100
        tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        tb_δo.displayed_string = "δi: $(round(steering.δo, digits=2))°"
    end

    on(sl_θ.sliders[3].value) do val
        θx = sl_θ.sliders[1].value.val
        θy = sl_θ.sliders[2].value.val
        θz = val
        update_geometry!((θx,θy,θz))
        obj = steering_objective((θx,θy,θz),chassis,steering,suspension)
        w = 1300
        L = obj + w 
        ratio = (w / L) * 100
        tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        tb_δo.displayed_string = "δi: $(round(steering.δo, digits=2))°"
    end


    display(fig)

end










steering = Steering(61.0, 74.0, 95.0, 228.0)


# initialisation of the susfpension
suspension = Suspension((30,30))


# steering setting
angleConfig = (0,0,0)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, steering, suspension)




GUI_steering(steering)