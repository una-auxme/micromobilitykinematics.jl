
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
                                        ratio_surf = false, 
                                        θ_vs_δ = false, 
                                        deviation = false, 
                                        deviation_surf = false, 
                                        compr_vs_δ = false, 
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
    section_plot.ax_geom.blockscene.visible[] = geom 
    section_plot.ax_radii.blockscene.visible[] = radii
    section_plot.ax_ratio.blockscene.visible[] = ratio
    section_plot.ax_ratio_surface.blockscene.visible[] = ratio_surf
    section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = θ_vs_δ
    section_plot.ax_deviation.blockscene.visible[] = deviation
    section_plot.ax_deviation_surface.blockscene.visible[] = deviation_surf
    section_plot.ax_compr_vs_δ.blockscene.visible[] = compr_vs_δ

    # Toggle angle slider visibility
    section_angle.sg_θ.sliders[1].blockscene.visible[] = sg_θx
    section_angle.sg_θ.sliders[2].blockscene.visible[] = sg_θy
    section_angle.sg_θ.sliders[3].blockscene.visible[] = sg_θz

    # Toggle compression slider visibility
    section_damper.sg_compr.sliders[1].blockscene.visible[] = comprL
    section_damper.sg_compr.sliders[2].blockscene.visible[] = comprR
    nothing

end







"""
    safe_ui begin
        ...
    end

Führt den Block mit try/catch aus. Tritt ein Fehler auf, wird er samt Stacktrace
über `sprint(showerror, e, bt)` formatiert und in die Konsole geschrieben.
Du kannst die Fehlerbehandlung unten anpassen (z. B. in ein Observable pushen).
"""
macro safe_ui(lyt, ex)
    quote
        try
            $(esc(ex))
            $(esc(lyt)).section_error.tb_error.displayed_string = "Error: " 
            $(esc(lyt)).section_error.tb_error.boxcolor = :white
            $(esc(lyt)).section_error.tb_error.textcolor = :black
        catch err
            if err isa ErrorException              
                $(esc(lyt)).section_error.tb_error.displayed_string = "Error: $(err.msg)"
                $(esc(lyt)).section_error.tb_error.boxcolor = :lightsalmon
                $(esc(lyt)).section_error.tb_error.textcolor = (:black, 1) 
            end
        end
    end
end