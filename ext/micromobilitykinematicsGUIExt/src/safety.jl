
"""
    safe_ui begin
        ...
    end

Führt den Block mit try/catch aus. Tritt ein Fehler auf, wird er samt Stacktrace
über `sprint(showerror, e, bt)` formatiert und in die Konsole geschrieben.
Du kannst die Fehlerbehandlung unten anpassen (z. B. in ein Observable pushen).
"""
macro safe_ui(steering, suspension, lyt, ex)
    fn = gensym(:safe_ui_body)
    quote
        # 1) Body in @noinline-Funktion kapseln → sichtbarer Frame im Trace
        Base.@noinline function $(fn)()
            $(esc(ex))
        end
        try
            $(esc(ex))
            $(esc(lyt)).section_error.tb_error_id.displayed_string = "Error ID: " 
            $(esc(lyt)).section_error.tb_error_id.boxcolor = :white
            $(esc(lyt)).section_error.tb_error_id.textcolor = :black

            $(esc(lyt)).section_error.tb_error_type.displayed_string = "Error Type: " 
            $(esc(lyt)).section_error.tb_error_type.boxcolor = :white
            $(esc(lyt)).section_error.tb_error_type.textcolor = :black

            $(esc(lyt)).section_error.tb_error_msg.text = "Error Msg.: " 
            #$(esc(lyt)).section_error.tb_error_msg.backgroundcolor = :white
            #$(esc(lyt)).section_error.tb_error_msg.color = :black


        catch err
            #bt = catch_backtrace()
            #msg = sprint(showerror, err, bt)  # inklusive Stacktrace
            if $(esc(steering)).err_info.type != nothing
                $(esc(lyt)).section_error.tb_error_id.displayed_string = "Error ID: $($(esc(steering)).err_info.id )" 
                $(esc(lyt)).section_error.tb_error_id.boxcolor = :lightsalmon
                $(esc(lyt)).section_error.tb_error_id.textcolor = (:black, 1) 

                $(esc(lyt)).section_error.tb_error_type.displayed_string = "Error Type: $($(esc(steering)).err_info.type)" 
                $(esc(lyt)).section_error.tb_error_type.boxcolor = :lightsalmon
                $(esc(lyt)).section_error.tb_error_type.textcolor = (:black, 1) 

                $(esc(lyt)).section_error.tb_error_msg.text = "Error Msg.: $($(esc(steering)).err_info.description)" 
                #$(esc(lyt)).section_error.tb_error_msg.backgroundcolor = :lightsalmon
                #$(esc(lyt)).section_error.tb_error_msg.color = (:black, 1) 

                #msg_ = sprint(showerror, $(esc(steering)).err_info.type, $(esc(steering)).err_info.backtrace)
                #@error "Error with backtrace" exception = $(esc(steering)).err_info.msg



                $(esc(steering)).err_info.type = nothing
            end

            if $(esc(suspension)).err_info.type != nothing
                $(esc(lyt)).section_error.tb_error_id.displayed_string = "Error ID: $($(esc(suspension)).err_info.id )" 
                $(esc(lyt)).section_error.tb_error_id.boxcolor = :lightsalmon
                $(esc(lyt)).section_error.tb_error_id.textcolor = (:black, 1) 

                $(esc(lyt)).section_error.tb_error_type.displayed_string = "Error Type: $($(esc(suspension)).err_info.type)" 
                $(esc(lyt)).section_error.tb_error_type.boxcolor = :lightsalmon
                $(esc(lyt)).section_error.tb_error_type.textcolor = (:black, 1) 

                $(esc(lyt)).section_error.tb_error_msg.text = "Error Msg.: $($(esc(suspension)).err_info.description)" 
                #$(esc(lyt)).section_error.tb_error_msg.backgroundcolor = :lightsalmon
                #$(esc(lyt)).section_error.tb_error_msg.color = (:black, 1) 

               

                #msg_ = sprint(showerror, $(esc(suspension)).err_info.type, $(esc(suspension)).err_info.backtrace)
                #@error "Error with backtrace" exception = $(esc(steering)).err_info.msg

                $(esc(suspension)).err_info.type = nothing
            end
            rethrow()
        end
    end
end
