


lower_bourder = (50.0,
50.0, 
70.0, 
195.0)
upper_bourder = (100.0,
100.0,
200.0,
260.0
)
max_angleConfig = (15.0,1.0,35.0)

set = random_search(upper_bourder, lower_bourder, max_angleConfig)
opt = optim_over_range(set...,max_angleConfig)

checkConstraints°(73.68312302748815, 99.99954088264283, 111.06518883155972, 224.05255235101745)

opt = optim02(upper_bourder, lower_bourder, max_angleConfig)

max_angleConfig = (15.0,1.0,35.0)
opt2 = optim_at_pose((1.0,1.0,20.0),upper_bourder, lower_bourder, max_angleConfig)





grid_optim(upper_bourder, lower_bourder, max_angleConfig)

sol_dict = optim_series(2, (1,1), upper_bourder, upper_bourder, max_angleConfig)

optda = sol_dict[2]

opt2.objective

save2 = opt2.steering

save = opt.steering

p = getValue(save)
# optimal solution (58.93776580661926, 100.00000099999852, 113.81284143399382, 228.618912533653)
# optimal solution (63.771311101944455, 100.00000099999696, 110.76251978448404, 227.2229191420079)

checkConstraints°(p...)

st = opt.steering


#(56.0, 86.0, 111.0, 215.0)


suspension = Suspension((30,30))


# steering setting
angleConfig = (0.0,1.0,0.0)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, st, suspension)

θ_max = (15.0,1.0,35.0)
GUI_steering(θ_max, chassis , st, suspension)



param = getValue(st)

checkConstraints°((71.72351165297609, 99.99961733848545, 116.65242673951506, 225.9899084736011))



θ_max = (15.0,0.0,35.0)
GUI_steering(θ_max, chassis , st, suspension)



grid_optim(upper_bourder,lower_bourder, max_angleConfig)




(57.44215025102158, 141.46879245569016, 169.82983068573606, 228.15747858550213)

#EXIT: Solved To Acceptable Level.
(62.2368497962264, 114.29468297994238, 132.90122400685277, 232.44025074027434)








 θ_tuples = [(i, j) for i in 0.0:1.0:15, j in 0.0:1.0:35] # searching space of (θx, θz)
θy = 0.0
[(θ_tuple[1], θy, θ_tuple[2]) for θ_tuple in θ_tuples]






 θ_tuples = [(i,l, j) for i in 0.0:1.0:10.0, l in 0.0:0.1:5.0, j in 0.0:1.0:30.0] # searching space of (θx, θz)




using Makie 


steering = Steering(74.49225659129358, 99.99825703300766, 122.32554031130196, 223.75433904895826)

# initialisation of the susfpension
suspension = Suspension((30,30))


# steering setting
angleConfig = (15.0,0.0,35.0)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, steering, suspension)


steering_objective(angleConfig,chassis, steering, suspension)

plt = plot_steering2(steering)












fig = GLMakie.Figure()

ax = GLMakie.Axis3(fig[1, 1])

GLMakie.xlims!(ax, 0, 20)
GLMakie.ylims!(ax, 0, 20)
GLMakie.zlims!(ax, 0, 20)




fig[2, 1] = buttongrid = GridLayout(tellwidth = false)

counts = Observable([1, 4, 3, 7, 2])

buttonlabels = [lift(x -> "Count: $(x[i])", counts) for i in 1:5]

buttons = buttongrid[1, 1:5] = [Slider(fig, label = l) for l in buttonlabels]

counts[][1] += 1

for i in 1:5
    on(buttons[i].clicks) do n
        counts[][i] += 1
        notify(counts)
    end
end

xs = ys =  zeros(5)

#barplot!(counts, color = cgrad(:Spectral)[LinRange(0, 1, 5)])
meshscatter!(xs, ys, counts, markersize = 0.1)





















steering = Steering(57.44215025102158, 141.46879245569016, 169.82983068573606, 228.15747858550213)

lower_bourder = (50.0,
50.0, 
70.0, 
100.0)

upper_bourder = (100.0,
100.0,
100.0,
250.0
)
max_angleConfig = (10,0,35)


param = random_search(upper_bourder, lower_bourder, max_angleConfig)
(61.0, 74.0, 95.0, 228.0)

steering = Steering(param...)


# initialisation of the susfpension
suspension = Suspension((30,30))


# steering setting
angleConfig = (0,0,0)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, steering, suspension)

#steering_objective((10,0,35), chassis,steering,suspension)


# Figure
fig = GLMakie.Figure(resolution = (1200, 1200))

ax = GLMakie.Axis3(fig[1:2, 1:3])
ax.aspect = :data

# Limits
GLMakie.xlims!(ax, -200, 50)
GLMakie.ylims!(ax, -300, 300)
GLMakie.zlims!(ax, -200, 50)

############# Angle Layout
angle_layout = GridLayout(tellheight = false)
fig[3,1] = angle_layout

titel = Label(angle_layout[1,1], "Rotation angle configuration", fontsize = 15)


#sl_θz = Slider(angle_layout[2, 1], range = -30:1:30, startvalue = 0)
sl_θ = SliderGrid(
                angle_layout[2, 1],
                (label = "θx", range = -30:1:30, format = "{:.1f}°", startvalue = 0),
                (label = "θz", range = 0:1:15, format = "{:.1f}°", startvalue = 0),
                (label = "θz", range = -30:1:30, format = "{:.1f}°", startvalue = 0),
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
                

############## Action Layout              
action_layout = GridLayout(tellheight = false)
fig[3, 2] = action_layout  # Sub-Layout einfügen

titel = Label(action_layout[1,1], "Actions", fontsize = 15)

# Füge mehrere Buttons in das verschachtelte Layout ein:

button_layout = GridLayout(tellheight = false)
action_layout[2,1] = button_layout

button1 = Button(button_layout[1, 1], label = "")
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

   tb_obj = Textbox(text_layout[1, 1], placeholder = "objective: ",width = 200)
   tb_δi = Textbox(text_layout[2, 1], placeholder = "δi: ",width = 200)
   tb_δo = Textbox(text_layout[3, 1], placeholder = "δo: ",width = 200)

   rowgap!(info_layout, 1, -165)




   colsize!(fig.layout, 1, Relative(0.3))  # Spalte 1 = 30%
   colsize!(fig.layout, 2, Relative(0.3))  # Spalte 2 = 30%
   colsize!(fig.layout, 3, Relative(0.3))  # Spalte 3 = 30%



   tb_obj.placeholder.val  = "changed"


tb_obj.placeholder.val 
#autolimits!(ax)


###############


#sliderobservables = [s.value for s in sg1.sliders]

###############

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


observe_rot = Observable(rotational_coponent)
observe_left = Observable(left_steering_connections)
observe_right = Observable(right_steering_connections)
observe_stationary = Observable(stationary)
#points = Observable(Point3f(steering.circle_joints[1]...))


GLMakie.scatter!(ax, observe_rot, markersize=10)
GLMakie.scatter!(ax, observe_left, markersize=10)
GLMakie.scatter!(ax, observe_right, markersize=10)
GLMakie.scatter!(ax, observe_stationary, markersize=10)

GLMakie.lines!(ax, observe_rot)
GLMakie.lines!(ax, observe_left)
GLMakie.lines!(ax, observe_right)





function update_geometry!((θx,θy,θz))
    println((θx, θy, θz))
    steeringkinematicsMOVED!((θx, θy, θz), steering, suspension)

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

sl_θ.sliders[1].value.val




on(sl_θ.sliders[1].value) do val
    θx = val
    θy = sl_θ.sliders[2].value.val
    θz = sl_θ.sliders[3].value.val

    update_geometry!((θx,θy,θz))
    #autolimits!(ax)
end

on(sl_θ.sliders[2].value) do val
    θx = sl_θ.sliders[1].value.val
    θy = val
    θz = sl_θ.sliders[3].value.val

    update_geometry!((θx,θy,θz))
    #autolimits!(ax)
end

on(sl_θ.sliders[3].value) do val
    θx = sl_θ.sliders[1].value.val
    θy = sl_θ.sliders[2].value.val
    θz = val

    update_geometry!((θx,θy,θz))
    #autolimits!(ax)
end





fig = GUI_layout()

s = fig.layout[3,1].contents

s.sliders


on(button1.clicks) do n
    
    angleConfig = (10,10,30)
    println(angleConfig)
    steeringkinematics!(angleConfig, steering, suspension)

end


# GridLayout
rowsize!(fig.layout, 1, Relative(3/4))
colsize!(fig.layout, 1, Relative(2/3))

rowsize!(fig.layout, 3, Relative(1/3))
colsize!(fig.layout, 3, Relative(1/3))


on(bt.clicks) do n
    tb.displayed_string = "track_lever_length"


end
 



sliderobservables = [s.value for s in sg.sliders]


GLMakie.meshscatter!(sliderobservables[1].val,  
sliderobservables[2].val,  
sliderobservables[3].val,
             markersize = 0.01)

text!(ax, string("Label"), position = Point3f(1, 5, 8), align = (:center, :bottom), fontsize = 10)

bars = lift(sliderobservables...) do slvalues...
    [slvalues...]
end



# 



steering 




steering.θx














function distanz_3d(p1, p2)
    return sqrt((p2[1] - p1[1])^2 + (p2[2] - p1[2])^2 + (p2[3] - p1[3])^2)
end


function sind_rechtwinklig(a, b; tol=1e-10)
    skalarprodukt = dot(a, b)
    return abs(skalarprodukt)
end


function winkel_zwischen(a, b; in_grad=true)
    if length(a) != length(b)
        error("Vektoren müssen die gleiche Dimension haben.")
    end
    
    skalarprodukt = dot(a, b)
    betrag_a = norm(a)
    betrag_b = norm(b)

    if betrag_a == 0 || betrag_b == 0
        error("Einer der Vektoren hat die Länge 0 – Winkel ist undefiniert.")
    end

    cos_theta = skalarprodukt / (betrag_a * betrag_b)
    # Sicherstellen, dass cos_theta im Bereich [-1, 1] bleibt (Numerik!)
    cos_theta = clamp(cos_theta, -1.0, 1.0)

    theta = acos(cos_theta)  # in Radiant

    return in_grad ? rad2deg(theta) : theta
end

# === Makie Setup ===
fig = Figure(resolution=(800, 600))
ax = Axis3(fig[1, 1])

# Observable Slider-Werte
θx = Node(0.0)
θy = Node(0.0)
θz = Node(0.0)

# Die Observable für die Punktwolke
points = Node(Point3f0[])

# Plot der Punkte (initial leer)
sc = scatter!(ax, points, markersize=10)

# Update-Funktion
function update_geometry!()
    steeringkinematicsMOVED!((θx[], θy[], θz[]), steering, suspension)
    left, right = steering.circle_joints
    points[] = Point3f0[left... , right...]
end

# Slider erstellen
sl_θx = Slider(fig[2, 1], range = -30:1:30, startvalue = 0) 
sl_θy = Slider(fig[3, 1], range = -30:1:30, startvalue = 0)
sl_θz = Slider(fig[4, 1], range = -30:1:30, startvalue = 0)

# Slider-Callbacks setzen
on(sl_θx.value) do val
    θx[] = val
    update_geometry!()
end
on(sl_θy.value) do val
    θy[] = val
    update_geometry!()
end
on(sl_θz.value) do val
    θz[] = val
    update_geometry!()
end

fig














###################### Versuche 1

θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (54.0, 157.0, 189.0, 196.0) #(69.0, 144.0, 180.0, 200.0) (106.0, 146.0, 188.0, 200.0)# (67.0, 158.0, 195.0, 196.0)
θx_,θz_ = (10,20)
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e0, 
                                                        "acceptable_tol" => 1e0, 
                                                        "dual_inf_tol" =>1e0, 
                                                        "compl_inf_tol"=>  1e0,
                                                        "constr_viol_tol"=> 1e0,
                                                        "max_iter" => 600))
                                                        
#model = Model(NLopt.Optimizer)
#verwendung von :LD_MMA funktionier besser
# lokales Minimum bereschnen da wir mit vor random suche bereits ein guten Start wert ermitteln und dannach diesen minimieren.
# jedes werte paar hat vermutlich seinen eigenen besten wert.

#set_optimizer_attribute(model, "algorithm", :LD_MMA)
#set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 200.0)
@variable(model, 50.0 <= z_rotational_radius <= 200.0)
@variable(model, 70.0 <= track_lever_length <= 200.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

@variable(model, θx)
@variable(model, θz)


steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

register(model, :objective, 6, objective, autodiff=true)
register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)


################## objective function ##################
@NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))

################## constraints        ##################
#hier ligt der Fehler
@NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1)


set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)

JuMP.optimize!(model)


####################### Versuche 2






θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (106.0, 146.0, 188.0, 200.0) #(96.0, 134.0, 146.0, 235.0)
θx_,θz_ = (1,20)
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e-2, 
                                                        "acceptable_tol" => 1e-2, 
                                                        "dual_inf_tol" =>1e-2, 
                                                        "compl_inf_tol"=>  1e-2,
                                                        "constr_viol_tol"=> 1e-2,
                                                        "max_iter" => 600))
                                                        
#model = Model(NLopt.Optimizer)
#verwendung von :LD_MMA funktionier besser
# lokales Minimum bereschnen da wir mit vor random suche bereits ein guten Start wert ermitteln und dannach diesen minimieren.
# jedes werte paar hat vermutlich seinen eigenen besten wert.

#set_optimizer_attribute(model, "algorithm", :LD_MMA)
#set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 200.0)
@variable(model, 50.0 <= z_rotational_radius <= 200.0)
@variable(model, 70.0 <= track_lever_length <= 200.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

@variable(model, θx)
@variable(model, θz)


steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

register(model, :objective, 6, objective, autodiff=true)


register(model, :left_circsphere_plane_dependence°, 6, left_circsphere_plane_dependence°, autodiff=true)
register(model, :right_circsphere_plane_dependence°, 6, right_circsphere_plane_dependence°, autodiff=true)
register(model, :left_circcirc_min_intersec_dependence°, 6, left_circcirc_min_intersec_dependence°, autodiff=true)
register(model, :right_circcirc_min_intersec_dependence°, 6, right_circcirc_min_intersec_dependence°, autodiff=true)
register(model, :left_circcirc_max_intersec_dependence°, 6, left_circcirc_max_intersec_dependence°, autodiff=true)
register(model, :right_circcirc_max_intersec_dependence°, 6, right_circcirc_max_intersec_dependence°, autodiff=true)
register(model, :outer_sigularity_constraint°, 6, outer_sigularity_constraint°, autodiff=true)
register(model, :inner_sigularity_constraint°, 6, inner_sigularity_constraint°, autodiff=true)
register(model, :track_circle_dependence°, 6, track_circle_dependence°, autodiff=true)
register(model, :angle_dependence°, 6, angle_dependence°, autodiff=true)


step_size = 1
################## objective function ##################
@NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))

################## constraints        ##################
#hier ligt der Fehler

@NLconstraint(model, C1_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circsphere_plane_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C1_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circsphere_plane_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C2_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circcirc_min_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C2_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circcirc_min_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C3_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circcirc_max_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C3_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circcirc_max_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C4_1[i=step_size:step_size:θx_max, j=0:step_size:(θz_max-2)], outer_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01 ) 
@NLconstraint(model, C4_2[i=0, j=step_size:step_size:(θz_max-2)], outer_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01) 

@NLconstraint(model, C5_1[i=step_size:step_size:θx_max, j=0:step_size:(θz_max-2)], inner_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01 ) 
@NLconstraint(model, C5_2[i=0, j=step_size:step_size:(θz_max-2)], inner_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01) 

@NLconstraint(model, C6_1[i=step_size:step_size:θx_max, j=0:step_size:θz_max], angle_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 0.01 )
@NLconstraint(model, C6_2[i=0, j=step_size:step_size:θz_max],  angle_dependence°(θx, θz,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 0.01 )



@NLconstraint(model, C7, track_circle_dependence°(0, θz_max, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)  >= 0)



set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)

JuMP.optimize!(model)






lower_bourder = (50.0,
50.0, 
70.0, 
195.0)
upper_bourder = (100.0,
200.0,
200.0,
260.0
)
max_angleConfig = (10,35)

@thread

random_search(upper_bourder, lower_bourder, max_angleConfig)
optim((1,20), upper_bourder, lower_bourder, max_angleConfig)

grid_optim(upper_bourder, lower_bourder, max_angleConfig)

sol_dict = optim_series(2, (1,1), upper_bourder, lower_bourder, max_angleConfig)

optda = sol_dict[2]

optda.objective



@__DIR__
path = joinpath(@__DIR__, "optimization\\data\\data(7,n)\\opt_series(7,5).jld2") # [4] ALMOST_LOCALLY_SOLVED
path = joinpath(@__DIR__, "optimization\\data\\data(9,n)\\opt_series(9,11).jld2") # [2] ALMOST_LOCALLY_SOLVED
path = joinpath(@__DIR__, "optimization\\data\\data(10,n)\\opt_series(10,11).jld2")


@load path opt_series


opt_series

plot_optda_series(opt_series)

optda = opt_series[2]


optda.objective
optda.status


steering = optda.steering

get_insights(steering)


chassi = Chassis()

suspension = Suspension(30)
suspensionkinematics!(suspension)

plot_optda_gird_δ((10,35), steering)

plot_optda_gird_obj((10,35), steering, suspension, chassi)




exportXML(steering)
exportXML(suspension)

measurements = Measurements(chassi, steering)

vehicle = Vehicle(measurements, chassi, steering, (suspension, suspension))


exportXML(vehicle)

doc = XMLDocument()
root = create_root(doc, "list of parameters")
child = new_child(root, "parameter")


typeof(child)


# Hauptcode
doc = XMLDocument()
root = create_root(doc, "list of parameters")
child = new_child(root, "parameter")

for field in fieldnames(typeof(steering))
    value = getfield(steering, field)

    if typeof(value) <: Tuple
        handle_tuple(child, field, steering)
    elseif typeof(value) <: Vector
        handle_vector(child, field, steering)
    elseif typeof(value) <: AbstractSteering || typeof(value) <: AbstractSuspension || typeof(value) <: AbstractVehicle
        handle_instance(child, value)
    else 
        handle_other(child, field, steering)
    end
end

save_file(doc,"ParamList.xml")





lower_bourder = (50.0,
50.0, 
70.0, 
195.0)
upper_bourder = (100.0,
200.0,
200.0,
260.0
)
max_angleConfig = (20,10,35)

@thread

random_search(upper_bourder, lower_bourder, max_angleConfig)













steering = Steering(62.0, 176.0, 189.0, 223.0)
steering = Steering(90.0, 149.0, 192.0, 242.0)

# initialisation of the susfpension
suspension = Suspension((30,30))


# steering setting
angleConfig = (0,10,40)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, steering, suspension)


plt = plot_steering(steering)

PlotlyJS.savefig(plt,"tmplinteractiveSVGfall<==")

PlotlyJS.savefig(plt, "plot_steeringkinematics.png")

suspension.lowerwishbone[1].sphere_joint
suspension.lowerwishbone[2].sphere_joint


plotSuspImpact((10,35), steering, suspension, chassis)


steering.circle_joints[1]
steering.wheel_ucs_position[1]
steering.wishbone_ucs_position[2]

steering.wishbone_ucs_position[1] + steering.wheel_ucs_position[1]




# new steering setting 
new_angleConfig = (0,25)

# update steering kinematics 
update!(new_angleConfig, steering, suspension)















function handle_instance(child, inst)
    for field in fieldnames(typeof(inst))
        value = getfield(inst, field)
        if typeof(value) <: Tuple
            for (side, param_index) in zip(["left_", "right_"], [1,2])
                for (dim, vec_index) in zip(["x_","y_","z_"],[1,2,3])
                    properties_child = new_child(child,"properties")
                    name_child = new_child(properties_child, "name")
                    field_name = String(field)
                    name = side*dim*field_name
                    add_text(name_child, name)
                    
                    typeCode_child = new_child(properties_child, "typeCode")
                    add_text(typeCode_child, "mm")
    
                    value_child = new_child(properties_child, "value")
                    value = getfield(inst, field)[param_index][vec_index]
                    add_text(value_child, "$value mm")
    
                    comment_child = new_child(properties_child, "comment")
                    add_text(comment_child, "")
    
                    key_child = new_child(properties_child, "key")
                    add_text(key_child, "")
    
                    tolerance_child = new_child(properties_child, "tolerance")
                    add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")
    
                end
            end
        elseif typeof(value) <: Vector
            for (dim, vec_index) in zip(["x_","y_","z_"],[1,2,3])
                properties_child = new_child(child,"properties")
                name_child = new_child(properties_child, "name")
                field_name = String(field)
                name = dim*field_name
                add_text(name_child, name)
                
                typeCode_child = new_child(properties_child, "typeCode")
                add_text(typeCode_child, "mm")
    
                value_child = new_child(properties_child, "value")
                value = getfield(inst, field)[vec_index]
                add_text(value_child, "$value mm")
    
                comment_child = new_child(properties_child, "comment")
                add_text(comment_child, "")
    
                key_child = new_child(properties_child, "key")
                add_text(key_child, "")
    
                tolerance_child = new_child(properties_child, "tolerance")
                add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")
    
            end
        elseif typeof(value) <: AbstractSteering || typeof(value) <: AbstractSuspension || typeof(value) <: AbstractVehicle
            handel_class(child,value)
        else
            properties_child = new_child(child,"properties")
            name_child = new_child(properties_child, "name")
            field_name = String(field)
            add_text(name_child, field_name)
    
            unit = ""
            if field == :δi || field == :δo || field == :θx || field == :θy
                unit = "deg"
            else
                unit = "mm"
            end
            typeCode_child = new_child(properties_child, "typeCode")
            add_text(typeCode_child, unit)
    
            value_child = new_child(properties_child, "value")
            value = getfield(inst, field)
            add_text(value_child, "$value $unit")
    
            comment_child = new_child(properties_child, "comment")
            add_text(comment_child, "")
    
            key_child = new_child(properties_child, "key")
            add_text(key_child, "")
    
            tolerance_child = new_child(properties_child, "tolerance")
            add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")
        end
    end
end