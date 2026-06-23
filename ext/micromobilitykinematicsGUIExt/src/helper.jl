
function set_axis_visible!(ax, v::Bool)
    ax.blockscene.visible[] = v   # Achsdeko
    ax.scene.visible[]      = v   # Plots (lines!, scatter!, …)
end

compr_vs_delta_title() = "Compression vs. wheel angles (δi = blue, δo = orange)"
compr_vs_delta_title(θx, θy, θz) = "$(compr_vs_delta_title()) for (θx, θy, θz) = ($θx,$θy,$θz)"
theta_vs_delta_title() = "Steering vs. wheel angles (δi = blue, δo = orange)"
theta_vs_delta_title(θx_max, θy, θz_max) = "$(theta_vs_delta_title()) for (θx max, θy, θz max) = ($θx_max,$θy,$θz_max)"
ackermann_ratio_mode_label(; signed = ackermann_ratio_signed()) = signed ? "signed Ackermann ratio" : "Ackermann ratio"
ackermann_ratio_title(θx, θy, θz; signed = ackermann_ratio_signed()) = "$(ackermann_ratio_mode_label(; signed = signed)) for (θx, θy, θz) = ($θx,$θy,$θz)"
ackermann_ratio_θx_title(θy, θz; signed = ackermann_ratio_signed()) = "$(ackermann_ratio_mode_label(; signed = signed)) over θx for (θy, θz) = ($θy,$θz)"
ackermann_ratio_surface_title(; signed = ackermann_ratio_signed()) = signed ? "Signed Ackermann ratio surface plot" : "Ackermann ratio surface plot"
left_wheel_delta_title(θx, θy, right_compression, θz_max) = "Left wheel Δδ vs. compression and θz (θx, θy, right compression, θz max) = ($θx,$θy,$right_compression,$θz_max)"
wheel_center_path_title() = "Wheel center path over symmetric compression (left = blue, right = orange)"
wheel_center_surface_title(θx, θy, θz_max) = "Wheel center surface over symmetric compression and signed θz (left = blue, right = orange, θx, θy, ±θz max) = ($θx,$θy,±$θz_max)"
track_width_title() = "Track width over symmetric compression"
motion_ratio_title() = "Damper travel / wheel center vertical travel over symmetric compression"
motion_ratio_ylabel() = "Δdamper travel / Δwheel center z [mm/mm]"
roll_kinematics_title(θx, θy, θz) = "Roll kinematics, left compression / right rebound for (θx, θy, θz) = ($θx,$θy,$θz)"
roll_camber_title() = "Camber (left = blue, right = orange)"
roll_wheel_angle_title() = "Wheel angle δ (left = blue, right = orange)"
roll_track_width_title() = "Track width"
roll_ackermann_ratio_title(; signed = ackermann_ratio_signed()) = "$(ackermann_ratio_mode_label(; signed = signed))"

"""
    update_layout_visibility!(interaction_lyt::InteractionLyt; 
                                geom = false, 
                                radii = false, 
                                ratio = false, 
                                ratio_surf = false, 
                                ax_θ_vs_δ = false, 
                                deviation = false, 
                                deviation_surf = false, 
                                compr_vs_δ = false, 
                                left_wheel_delta = false,
                                sg_θx = false, 
                                sg_θy = false, 
                                sg_θz = false, 
                                comprL = false, 
                                comprR = false)

Controls the visibility of plots and sliders in the `InteractionLyt` layout.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object containing plots, sliders, and other UI sections.

# Keywords
- `geom`: Show or hide the geometry plot.
- `radii`: Show or hide the radii plot.
- `ratio`: Show or hide the Ackermann ratio plot.
- `ratio_surf`: Show or hide the Ackermann ratio surface plot.
- `ax_θ_vs_δ`: Show or hide the θ vs δ surface plot.
- `deviation`: Show or hide the deviation plot.
- `deviation_surf`: Show or hide the deviation surface plot.
- `compr_vs_δ`: Show or hide the compression vs δi plot.
- `sg_θx`: Show or hide the θx angle slider.
- `sg_θy`: Show or hide the θy angle slider.
- `sg_θz`: Show or hide the θz angle slider.
- `comprL`: Show or hide the left compression slider.
- `comprR`: Show or hide the right compression slider.

# Description
This function updates the visibility state of different plot axes and sliders 
within the interactive layout. By setting the keyword arguments to `true` or 
`false`, it toggles the display of the corresponding UI elements.

# Returns
Nothing. Updates the UI as a side effect.
"""
function update_layout_visibility!(interaction_lyt::InteractionLyt; 
                                        geom = false, 
                                        radii = false, 
                                        ratio = false, 
                                        ratio_θx = false, 
                                        ratio_surf = false, 
                                        θ_vs_δ = false, 
                                        deviation = false, 
                                        deviation_surf = false, 
                                        compr_vs_δ = false, 
                                        left_wheel_delta = false,
                                        wheel_center_path = false,
                                        wheel_center_surface = false,
                                        track_width = false,
                                        motion_ratio = false,
                                        roll_kinematics = false,
                                        sg_θx = false, 
                                        sg_θy = false, 
                                        sg_θz = false, 
                                        comprL = false, 
                                        comprR = false )


    # Layout sections for convenience                     
    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    # Toggle plot visibility
      # 2D / 3D Axes
    set_axis_visible!(section_plot.ax_geom,              geom)
    set_axis_visible!(section_plot.ax_radii,             radii)
    set_axis_visible!(section_plot.ax_ratio,             ratio)
    set_axis_visible!(section_plot.ax_ratio_θx,          ratio_θx)
    set_axis_visible!(section_plot.ax_ratio_surface,     ratio_surf)
    set_axis_visible!(section_plot.ax_θ_vs_δ_surface,    θ_vs_δ)
    set_axis_visible!(section_plot.ax_deviation,         deviation)
    set_axis_visible!(section_plot.ax_deviation_surface, deviation_surf)
    set_axis_visible!(section_plot.ax_compr_vs_δ,        compr_vs_δ)
    set_axis_visible!(section_plot.ax_left_wheel_delta,   left_wheel_delta)
    set_axis_visible!(section_plot.ax_wheel_center_path,  wheel_center_path)
    set_axis_visible!(section_plot.ax_wheel_center_surface, wheel_center_surface)
    set_axis_visible!(section_plot.ax_track_width,        track_width)
    set_axis_visible!(section_plot.ax_motion_ratio,       motion_ratio)
    set_axis_visible!(section_plot.ax_roll_camber,        roll_kinematics)
    set_axis_visible!(section_plot.ax_roll_wheel_angle,   roll_kinematics)
    set_axis_visible!(section_plot.ax_roll_track_width,   roll_kinematics)
    set_axis_visible!(section_plot.ax_roll_ackermann_deviation, roll_kinematics)



    # Toggle angle slider visibility
    section_angle.sg_θ.sliders[1].blockscene.visible[] = sg_θx
    section_angle.sg_θ.sliders[2].blockscene.visible[] = sg_θy
    section_angle.sg_θ.sliders[3].blockscene.visible[] = sg_θz

    # Toggle compression slider visibility
    section_damper.sg_compr.sliders[1].blockscene.visible[] = comprL
    section_damper.sg_compr.sliders[2].blockscene.visible[] = comprR
    nothing

end

function surface_zlimits(data_surfaces...; lower_floor = 0.0, min_span = 1.0, padding = 0.08)
    values = Float64[]

    for data in data_surfaces
        for value in data
            if value isa Real && isfinite(value)
                push!(values, Float64(value))
            end
        end
    end

    isempty(values) && return (lower_floor, lower_floor + min_span)

    zmin, zmax = extrema(values)

    if zmin == zmax
        half_span = max(min_span / 2, abs(zmin) * padding)
        zmin -= half_span
        zmax += half_span
    else
        pad = max((zmax - zmin) * padding, min_span * 0.05)
        zmin -= pad
        zmax += pad
    end

    zmin = max(lower_floor, zmin)
    zmax = max(zmax, zmin + eps(Float64))

    return (zmin, zmax)
end

function clean_tick_label(value, digits)
    rounded = round(abs(value) < eps(Float64) ? 0.0 : value; digits = digits)
    label = string(rounded)

    while occursin(".", label) && endswith(label, "0")
        label = label[1:end-1]
    end

    if endswith(label, ".")
        label = label[1:end-1]
    end

    return label == "-0" ? "0" : label
end

function nice_tick_step(span; target_count = 6)
    span <= 0 && return 1.0

    raw_step = span / max(target_count - 1, 1)
    magnitude = 10.0 ^ floor(log10(raw_step))
    fraction = raw_step / magnitude

    nice_fraction =
        fraction <= 1.0  ? 1.0 :
        fraction <= 2.0  ? 2.0 :
        fraction <= 2.5  ? 2.5 :
        fraction <= 5.0  ? 5.0 :
                            10.0

    return nice_fraction * magnitude
end

function nice_axis_ticks(zmin, zmax; target_count = 6)
    step = nice_tick_step(zmax - zmin; target_count = target_count)
    start_tick = floor(zmin / step) * step
    stop_tick = ceil(zmax / step) * step

    zmin >= 0 && start_tick < 0 && (start_tick = 0.0)

    tick_count = max(1, Int(round((stop_tick - start_tick) / step)))
    digits = max(0, Int(ceil(-log10(step))) + 1)
    ticks = [round(start_tick + i * step; digits = digits + 2) for i in 0:tick_count]
    labels = [clean_tick_label(tick, digits) for tick in ticks]

    return ticks, labels
end

function finite_plot_values(data)
    values = Float64[]

    for value in data
        if value isa Real && isfinite(value)
            push!(values, Float64(value))
        end
    end

    return values
end

function finite_minimum(data)
    values = finite_plot_values(data)
    isempty(values) && return NaN
    return minimum(values)
end

function finite_maximum(data)
    values = finite_plot_values(data)
    isempty(values) && return NaN
    return maximum(values)
end

function signed_ratio_axis_ticks(data; lower_default = 30.0, upper_default = 105.0)
    values = finite_plot_values(data)
    isempty(values) && return (lower_default:5.0:upper_default, string.(lower_default:5.0:upper_default))

    ymin, ymax = extrema([values; 100.0])
    span = max(ymax - ymin, 10.0)
    padding = max(0.08 * span, 2.0)
    ymin = min(lower_default, ymin - padding)
    ymax = max(upper_default, ymax + padding)

    return nice_axis_ticks(ymin, ymax)
end

function set_ratio_ylims!(ax, data; signed = ackermann_ratio_signed(), lower_default = 30.0, upper_default = 105.0)
    if signed
        ticks, labels = signed_ratio_axis_ticks(data; lower_default = lower_default, upper_default = upper_default)
        GLMakie.ylims!(ax, first(ticks), last(ticks))
        ax.yticks = (ticks, labels)
    else
        GLMakie.ylims!(ax, lower_default, upper_default)
        ax.yticks = lower_default:5.0:(upper_default - 5.0)
    end

    nothing
end

function set_ratio_zlims!(ax, data; signed = ackermann_ratio_signed(), lower_default = 50.0, upper_default = 105.0)
    if signed
        ticks, labels = signed_ratio_axis_ticks(data; lower_default = lower_default, upper_default = upper_default)
        GLMakie.zlims!(ax, first(ticks), last(ticks))
        ax.zticks = (ticks, labels)
    else
        GLMakie.zlims!(ax, lower_default, upper_default)
        ax.zticks = lower_default:10.0:(upper_default - 5.0)
    end

    nothing
end

function ratio_surface_colorrange(data; signed = ackermann_ratio_signed(), lower_default = 50.0, upper_default = 105.0)
    if signed
        ticks, labels = signed_ratio_axis_ticks(data; lower_default = lower_default, upper_default = upper_default)
        return (first(ticks), last(ticks))
    end

    values = finite_plot_values(data)
    isempty(values) && return (lower_default, upper_default)

    zmin, zmax = extrema(values)
    zmin == zmax && return (zmin - 1.0, zmax + 1.0)

    return (zmin, zmax)
end

function signed_ackermann_ratio_colormap(data)
    zmin, zmax = ratio_surface_colorrange(data; signed = true)
    cutoff = clamp((100.0 - zmin) / max(zmax - zmin, eps(Float64)), 0.0, 1.0)
    hard_edge = 1e-6

    if cutoff <= hard_edge
        return cgrad([:darkorange, :firebrick])
    end

    if cutoff >= 1.0 - hard_edge
        return cgrad([:royalblue, :deepskyblue])
    end

    return cgrad(
        [:royalblue, :deepskyblue, :deepskyblue, :darkorange, :firebrick],
        [0.0, cutoff - hard_edge, cutoff, cutoff + hard_edge, 1.0],
    )
end

function ackermann_ratio_surface_colormap(data; signed = ackermann_ratio_signed())
    signed ? signed_ackermann_ratio_colormap(data) : cgrad(:darkterrain)
end

function set_compr_vs_delta_zlims!(ax, delta_surfaces...)
    zmin, zmax = surface_zlimits(delta_surfaces...)
    ticks, labels = nice_axis_ticks(zmin, zmax)

    GLMakie.zlims!(ax, first(ticks), last(ticks))
    ax.zticks = (ticks, labels)
    nothing
end

function set_left_wheel_delta_zlims!(ax, delta_surface)
    zmin, zmax = surface_zlimits(delta_surface; lower_floor = -Inf)
    ticks, labels = nice_axis_ticks(zmin, zmax)

    GLMakie.zlims!(ax, first(ticks), last(ticks))
    ax.zticks = (ticks, labels)
    nothing
end

function set_line_ylims!(ax, data_series...; lower_floor = -Inf, min_span = 1.0)
    zmin, zmax = surface_zlimits(data_series...; lower_floor = lower_floor, min_span = min_span)
    ticks, labels = nice_axis_ticks(zmin, zmax)

    GLMakie.ylims!(ax, first(ticks), last(ticks))
    ax.yticks = (ticks, labels)
    nothing
end

function set_wheel_center_path_limits!(ax, path_series...)
    xs = Float64[]
    ys = Float64[]
    zs = Float64[]

    for path in path_series
        for point in path
            point_tuple = Tuple(point)
            if all(value -> value isa Real && isfinite(value), point_tuple)
                push!(xs, Float64(point_tuple[1]))
                push!(ys, Float64(point_tuple[2]))
                push!(zs, Float64(point_tuple[3]))
            end
        end
    end

    isempty(xs) && return nothing

    xlims = surface_zlimits(xs; lower_floor = -Inf, min_span = 1.0)
    ylims = surface_zlimits(ys; lower_floor = -Inf, min_span = 1.0)
    zlims = surface_zlimits(zs; lower_floor = -Inf, min_span = 1.0)

    GLMakie.xlims!(ax, xlims...)
    GLMakie.ylims!(ax, ylims...)
    GLMakie.zlims!(ax, zlims...)
    nothing
end

function set_wheel_center_surface_limits!(ax, coordinate_surfaces...)
    xs = Float64[]
    ys = Float64[]
    zs = Float64[]

    for (x_matrix, y_matrix, z_matrix) in coordinate_surfaces
        for index in eachindex(x_matrix, y_matrix, z_matrix)
            x = x_matrix[index]
            y = y_matrix[index]
            z = z_matrix[index]

            if all(value -> value isa Real && isfinite(value), (x, y, z))
                push!(xs, Float64(x))
                push!(ys, Float64(y))
                push!(zs, Float64(z))
            end
        end
    end

    isempty(xs) && return nothing

    xlims = surface_zlimits(xs; lower_floor = -Inf, min_span = 1.0)
    ylims = surface_zlimits(ys; lower_floor = -Inf, min_span = 1.0)
    zlims = surface_zlimits(zs; lower_floor = -Inf, min_span = 1.0)

    GLMakie.xlims!(ax, xlims...)
    GLMakie.ylims!(ax, ylims...)
    GLMakie.zlims!(ax, zlims...)
    nothing
end

function update_compr_vs_delta_surface!(section_plot, θ, steering, suspension)
    steering_copy = deepcopy(steering)
    suspension_copy = deepcopy(suspension)
    delta_i, delta_o = compr_vs_δ(θ, steering_copy, suspension_copy)

    section_plot.obs_compr_vs_δi[] = delta_i
    section_plot.obs_compr_vs_δo[] = delta_o
    set_compr_vs_delta_zlims!(section_plot.ax_compr_vs_δ, delta_i, delta_o)

    nothing
end

function update_left_wheel_delta_surface!(section_plot, θx, θy, θz_max, steering, suspension)
    steering_copy = deepcopy(steering)
    suspension_copy = deepcopy(suspension)
    right_compression = suspension.damper[2].compression

    delta_left = left_wheel_delta_vs_compression_θz(
        θx,
        θy,
        θz_max,
        steering_copy,
        suspension_copy;
        fixed_right_compression = right_compression,
    )

    section_plot.obs_left_wheel_delta[] = delta_left
    set_left_wheel_delta_zlims!(section_plot.ax_left_wheel_delta, delta_left)
    section_plot.ax_left_wheel_delta.title = left_wheel_delta_title(θx, θy, right_compression, θz_max)

    nothing
end

function update_wheel_center_path_plot!(section_plot, steering, suspension)
    compression_values, left_path, right_path = wheel_center_path(steering, suspension)

    section_plot.obs_wheel_center_left[] = left_path
    section_plot.obs_wheel_center_right[] = right_path
    section_plot.ax_wheel_center_path.title = wheel_center_path_title()
    set_wheel_center_path_limits!(section_plot.ax_wheel_center_path, left_path, right_path)

    nothing
end

function update_wheel_center_surface_plot!(section_plot, θx, θy, θz_max, steering, suspension)
    (
        compression_values,
        θz_values,
        left_x,
        left_y,
        left_z,
        right_x,
        right_y,
        right_z,
    ) = wheel_center_surface(θx, θy, θz_max, steering, suspension)

    section_plot.obs_wheel_center_surface_left_x[] = left_x
    section_plot.obs_wheel_center_surface_left_y[] = left_y
    section_plot.obs_wheel_center_surface_left_z[] = left_z
    section_plot.obs_wheel_center_surface_right_x[] = right_x
    section_plot.obs_wheel_center_surface_right_y[] = right_y
    section_plot.obs_wheel_center_surface_right_z[] = right_z
    section_plot.ax_wheel_center_surface.title = wheel_center_surface_title(θx, θy, θz_max)
    set_wheel_center_surface_limits!(
        section_plot.ax_wheel_center_surface,
        (left_x, left_y, left_z),
        (right_x, right_y, right_z),
    )

    nothing
end

function update_track_width_plot!(section_plot, steering, suspension)
    compression_values, track_width = track_width_over_compression(steering, suspension)

    section_plot.obs_track_width[] = track_width
    section_plot.ax_track_width.title = track_width_title()
    set_line_ylims!(section_plot.ax_track_width, track_width; lower_floor = 0.0, min_span = 1.0)

    nothing
end

function update_motion_ratio_plot!(section_plot, steering, suspension)
    compression_values, motion_ratio = damper_motion_ratio(steering, suspension)

    section_plot.obs_motion_ratio[] = motion_ratio
    section_plot.ax_motion_ratio.title = motion_ratio_title()
    set_line_ylims!(section_plot.ax_motion_ratio, motion_ratio; lower_floor = 0.0, min_span = 0.1)

    nothing
end

function update_roll_kinematics_plot!(section_plot, θ, chassis, steering, suspension; signed = ackermann_ratio_signed())
    θx, θy, θz = θ
    (
        roll_values,
        left_camber,
        right_camber,
        left_wheel_angle,
        right_wheel_angle,
        track_width,
        ackermann_ratio_values,
    ) = roll_kinematics(θ, chassis, steering, suspension; signed = signed)

    section_plot.obs_roll_left_camber[] = left_camber
    section_plot.obs_roll_right_camber[] = right_camber
    section_plot.obs_roll_left_wheel_angle[] = left_wheel_angle
    section_plot.obs_roll_right_wheel_angle[] = right_wheel_angle
    section_plot.obs_roll_track_width[] = track_width
    section_plot.obs_roll_ackermann_deviation[] = ackermann_ratio_values

    section_plot.ax_roll_camber.title = roll_camber_title()
    section_plot.ax_roll_wheel_angle.title = roll_wheel_angle_title()
    section_plot.ax_roll_track_width.title = roll_track_width_title()
    section_plot.ax_roll_ackermann_deviation.title = roll_ackermann_ratio_title(; signed = signed)
    section_plot.ax_roll_ackermann_deviation.ylabel = "Ackermann ratio [%]"

    set_line_ylims!(section_plot.ax_roll_camber, left_camber, right_camber; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(section_plot.ax_roll_wheel_angle, left_wheel_angle, right_wheel_angle; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(section_plot.ax_roll_track_width, track_width; lower_floor = 0.0, min_span = 1.0)
    set_ratio_ylims!(section_plot.ax_roll_ackermann_deviation, ackermann_ratio_values; signed = signed, lower_default = 30.0)

    nothing
end

function update_ratio_θz_plot!(section_plot, θx, θy, θz, θz_max, chassis, steering, suspension; signed = ackermann_ratio_signed())
    ratio_θz = ackermannratio_θz(θx, θy, θz_max, chassis, steering, suspension; signed = signed)

    section_plot.ax_ratio.title = ackermann_ratio_title(θx, θy, θz; signed = signed)
    section_plot.obs_ratio_θz[] = ratio_θz
    section_plot.obs_ratio_min[] = finite_minimum(ratio_θz)
    section_plot.obs_ratio_max[] = finite_maximum(ratio_θz)
    set_ratio_ylims!(section_plot.ax_ratio, ratio_θz; signed = signed, lower_default = 30.0)

    nothing
end

function update_ratio_θx_plot!(section_plot, θx_max, θy, θz, chassis, steering, suspension; signed = ackermann_ratio_signed())
    chassis_copy = deepcopy(chassis)
    steering_copy = deepcopy(steering)
    suspension_copy = deepcopy(suspension)
    ratio_θx = ackermannratio_θx(θx_max, θy, θz, chassis_copy, steering_copy, suspension_copy; signed = signed)

    section_plot.ax_ratio_θx.title = ackermann_ratio_θx_title(θy, θz; signed = signed)
    section_plot.obs_ratio_θx[] = ratio_θx
    section_plot.obs_ratio_θx_min[] = finite_minimum(ratio_θx)
    section_plot.obs_ratio_θx_max[] = finite_maximum(ratio_θx)
    set_ratio_ylims!(section_plot.ax_ratio_θx, ratio_θx; signed = signed, lower_default = 30.0)

    nothing
end

function update_ratio_surface_plot!(section_plot, θy, θ_max, chassis, steering, suspension; signed = ackermann_ratio_signed())
    θx_max, θy_max, θz_max = θ_max
    chassis_copy = deepcopy(chassis)
    steering_copy = deepcopy(steering)
    suspension_copy = deepcopy(suspension)
    ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (θx_max, θy, θz_max); signed = signed)

    section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(; signed = signed)
    section_plot.obs_ratio_surface[] = ratio_surface
    set_ratio_zlims!(section_plot.ax_ratio_surface, ratio_surface; signed = signed)

    nothing
end

function update_current_ackermann_ratio_views!(interaction_lyt, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max
    section_plot = interaction_lyt.section_plot
    section_angle = interaction_lyt.section_angle
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info = interaction_lyt.section_info

    θx = section_angle.sg_θ.sliders[1].value.val
    θy = section_angle.sg_θ.sliders[2].value.val
    θz = section_angle.sg_θ.sliders[3].value.val
    signed = ackermann_ratio_signed()
    selected_plot = section_plot_settings.menu.selection.val

    if selected_plot == "Ackermann ratio"
        update_ratio_θz_plot!(section_plot, θx, θy, θz, θz_max, chassis, steering, suspension; signed = signed)
    end

    if selected_plot == "Ackermann ratio θx sweep"
        update_ratio_θx_plot!(section_plot, θx_max, θy, θz, chassis, steering, suspension; signed = signed)
    end

    if selected_plot == "Ackermann ratio surface plot"
        update_ratio_surface_plot!(section_plot, θy, θ_max, chassis, steering, suspension; signed = signed)
    end

    if selected_plot == "Roll kinematics"
        update_roll_kinematics_plot!(section_plot, (θx, θy, θz), chassis, steering, suspension; signed = signed)
    end

    ratio = ackermannratio((θx,θy,θz), chassis, steering, suspension; signed = signed)
    section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"

    nothing
end
