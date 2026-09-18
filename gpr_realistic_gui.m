function gpr_realistic_gui(model_name,cfg,feasibility,sim_out)
% GPR_REALISTIC_GUI  Minimal result viewer with valid callback ordering.
fig=uifigure('Name',['GPR Realistic: ' cfg.mode],'Position',[50 50 1400 800]);
grid=uigridlayout(fig,[2 2],'RowHeight',{'1x','2x'},'ColumnWidth',{'1x','2x'});
p1=uipanel(grid,'Title','Physics Feasibility'); p1.Layout.Row=1; p1.Layout.Column=1;
fg=uigridlayout(p1,[10 1]);
if feasibility.viable, status='PASS'; color=[0.2 0.6 0.2]; else, status='WARNING'; color=[0.8 0.2 0.2]; end
uilabel(fg,'Text',['STATUS: ' status],'FontSize',16,'FontWeight','bold','FontColor',color);
uilabel(fg,'Text',['Mode: ' cfg.mode]); uilabel(fg,'Text',sprintf('Band: %.1f-%.1f MHz',cfg.f_start/1e6,cfg.f_stop/1e6));
uilabel(fg,'Text',sprintf('Soil sigma: %.4f S/m',cfg.soil_conductivity)); uilabel(fg,'Text',sprintf('Range resolution: %.3f m',feasibility.range_res));
uilabel(fg,'Text',sprintf('Unambiguous depth: %.2f m',feasibility.unambig)); uilabel(fg,'Text',sprintf('IFFT window: %.2f m',feasibility.max_ifft_depth));
uilabel(fg,'Text',sprintf('EIRP: %.1f dBm',cfg.eirp_dBm));
if ~feasibility.viable, uilabel(fg,'Text',['Reason: ' feasibility.reason],'FontColor',color); end

p3=uipanel(grid,'Title','Results'); p3.Layout.Row=[1 2]; p3.Layout.Column=2;
tabs=uitabgroup(p3); t1=uitab(tabs,'Title','Report'); report_area=uitextarea(t1,'Editable','off');
t2=uitab(tabs,'Title','Detection Matrix'); ax=uiaxes(t2);
res_axes=struct('report',report_area,'ax',ax,'cfg',cfg);

p2=uipanel(grid,'Title','Simulation Control'); p2.Layout.Row=2; p2.Layout.Column=1;
ctrl=uigridlayout(p2,[4 1]);
status_label=uilabel(ctrl,'Text','Ready.');
uibutton(ctrl,'Text','Re-run simulation','ButtonPushedFcn',@(~,~)rerun_sim());
uibutton(ctrl,'Text','Open Simulink model','ButtonPushedFcn',@(~,~)open_system(model_name));
uibutton(ctrl,'Text','Open RF front end','ButtonPushedFcn',@(~,~)open_system([model_name '/RF_Front_End']));
plot_results(sim_out,res_axes); 

    function rerun_sim()
        try
            status_label.Text='Running...'; drawnow;
            new_out=sim(model_name); plot_results(new_out,res_axes); status_label.Text='Complete.';
        catch ME
            status_label.Text='Simulation error.'; uialert(fig,ME.message,'Simulation Error');
        end
    end
end

function plot_results(sim_out,ax)
try
    if isprop(sim_out,'gpr_logs') || isfield(sim_out,'gpr_logs')
        logs=sim_out.gpr_logs; ax.report.Value={'Simulation completed.','Inspect TP01-TP14 Scopes and tp* workspace variables.'};
        if ~isempty(logs), title(ax.ax,'Logged detection output'); end
    else
        ax.report.Value={'Simulation completed.','Inspect TP01-TP14 Scopes and tp* workspace variables.'};
    end
catch ME
    ax.report.Value={['Plot note: ' ME.message]};
end
end
