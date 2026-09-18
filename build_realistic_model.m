function model_name = build_realistic_model(cfg)
% BUILD_REALISTIC_MODEL  Build the complete algorithm as visible Simulink.
% Top level: RF_Front_End -> Digital_Processing.
% Every Bxx block has an observation Scope and To Workspace test point.

model_name = 'GPR_Realistic';
create_all_params(cfg);
if bdIsLoaded(model_name), close_system(model_name,0); end
if exist([model_name '.slx'],'file'), delete([model_name '.slx']); end
new_system(model_name); open_system(model_name);
set_param(model_name,'Solver','FixedStepDiscrete','FixedStep','1', ...
    'StartTime','0','StopTime','0','SignalLogging','on', ...
    'SignalLoggingName','gpr_logs','ReturnWorkspaceOutputs','on');

add_block('built-in/Subsystem',[model_name '/RF_Front_End'], ...
    'Position',[80 120 390 420]);
add_block('built-in/Subsystem',[model_name '/Digital_Processing'], ...
    'Position',[500 120 820 420]);
add_block('simulink/Sinks/To Workspace',[model_name '/Final_Report_Log'], ...
    'Position',[900 230 1040 260],'VariableName','gpr_final_report', ...
    'SaveFormat','Structure With Time');
add_block('simulink/Sinks/Display',[model_name '/Final_Report_Display'], ...
    'Position',[900 300 1040 330]);
add_line(model_name,'Digital_Processing/1','Final_Report_Log/1','autorouting','on');
add_line(model_name,'Digital_Processing/1','Final_Report_Display/1','autorouting','on');

build_rf([model_name '/RF_Front_End'], cfg);
build_dsp([model_name '/Digital_Processing'], cfg);
add_line(model_name,'RF_Front_End/1','Digital_Processing/1','autorouting','on');

set_param(model_name,'SimulationCommand','update');
save_system(model_name);
fprintf('Built RF_Front_End + Digital_Processing with TP01-TP14.\n');
end

function create_all_params(cfg)
items = {
 'GPR_F_START',cfg.f_start; 'GPR_F_STOP',cfg.f_stop; 'GPR_UAV_ALTITUDE',cfg.uav_altitude;
 'GPR_SCAN_LENGTH',cfg.scan_length; 'GPR_SOIL_MOISTURE',cfg.soil_moisture;
 'GPR_SOIL_SIGMA',cfg.soil_conductivity; 'GPR_PA_GAIN_DB',cfg.pa_gain_dB;
 'GPR_LNA_GAIN_DB',cfg.lna_gain_dB; 'GPR_NF_DB',cfg.system_nf_dB;
 'GPR_ADC_ENOB',cfg.adc_enob; 'GPR_N_AVG',cfg.n_averages;
 'GPR_KAISER_BETA',cfg.kaiser_beta; 'GPR_DEPTH_MIN',cfg.depth_min;
 'GPR_DEPTH_MAX',cfg.depth_max; 'GPR_PFA',cfg.cfar_pfa;
 'GPR_TGT_DEPTH',cfg.target_depths(:); 'GPR_TGT_X',cfg.target_x(:);
 'GPR_TGT_RCS',cfg.target_rcs(:); 'GPR_COUPLING_DB',cfg.antenna_coupling_dB};
for k=1:size(items,1)
    p=Simulink.Parameter(items{k,2}); p.DataType='double';
    assignin('base',items{k,1},p);
end
end

function build_rf(parent,cfg)
add_block('simulink/Ports & Subsystems/Out1',[parent '/RF_Out'], ...
    'Position',[1200 300 1230 320]);
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_ALT'], 'Position',[20 30 90 55],'Value','GPR_UAV_ALTITUDE');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_SCAN'], 'Position',[20 70 90 95],'Value','GPR_SCAN_LENGTH');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_FSTART'], 'Position',[20 120 90 145],'Value','GPR_F_START');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_FSTOP'], 'Position',[20 160 90 185],'Value','GPR_F_STOP');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_PA'], 'Position',[20 230 90 255],'Value','GPR_PA_GAIN_DB');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_MOIST'], 'Position',[20 500 90 525],'Value','GPR_SOIL_MOISTURE');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_SIGMA'], 'Position',[20 540 90 565],'Value','GPR_SOIL_SIGMA');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_TGT_D'], 'Position',[20 580 90 605],'Value','GPR_TGT_DEPTH');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_TGT_X'], 'Position',[20 620 90 645],'Value','GPR_TGT_X');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_TGT_RCS'], 'Position',[20 660 90 685],'Value','GPR_TGT_RCS');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_COUPLING'], 'Position',[20 760 90 785],'Value','GPR_COUPLING_DB');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_LNA'], 'Position',[20 900 90 925],'Value','GPR_LNA_GAIN_DB');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_NF'], 'Position',[20 940 90 965],'Value','GPR_NF_DB');
add_block('simulink/Ports & Subsystems/Constant',[parent '/P_ENOB'], 'Position',[20 1040 90 1065],'Value','GPR_ADC_ENOB');

add_mlfcn(parent,'B01_Environment',[220 30 420 80],code_environment(cfg));
add_mlfcn(parent,'B02_Waveform',[500 30 700 80],code_waveform(cfg));
add_mlfcn(parent,'B03_TX_Chain',[780 30 980 80],code_tx(cfg));
add_mlfcn(parent,'B04_Channel',[500 180 760 240],code_channel(cfg));
add_mlfcn(parent,'B05_Coupling',[820 180 1040 240],code_coupling(cfg));
add_mlfcn(parent,'B06_RX',[500 340 700 390],code_rx(cfg));
add_mlfcn(parent,'B07_ADC',[780 340 980 390],code_adc(cfg));
add_line(parent,'P_ALT/1','B01_Environment/1'); add_line(parent,'P_SCAN/1','B01_Environment/2');
add_line(parent,'P_FSTART/1','B02_Waveform/1'); add_line(parent,'P_FSTOP/1','B02_Waveform/2');
add_line(parent,'B02_Waveform/1','B03_TX_Chain/1'); add_line(parent,'P_PA/1','B03_TX_Chain/2');
add_line(parent,'B01_Environment/1','B04_Channel/1'); add_line(parent,'B03_TX_Chain/1','B04_Channel/2');
add_line(parent,'P_MOIST/1','B04_Channel/3'); add_line(parent,'P_SIGMA/1','B04_Channel/4');
add_line(parent,'P_TGT_D/1','B04_Channel/5'); add_line(parent,'P_TGT_X/1','B04_Channel/6'); add_line(parent,'P_TGT_RCS/1','B04_Channel/7');
add_line(parent,'P_FSTART/1','B04_Channel/8'); add_line(parent,'P_FSTOP/1','B04_Channel/9');
add_line(parent,'B03_TX_Chain/1','B05_Coupling/1'); add_line(parent,'P_COUPLING/1','B05_Coupling/2'); add_line(parent,'B04_Channel/1','B05_Coupling/3');
add_line(parent,'B05_Coupling/1','B06_RX/1'); add_line(parent,'P_LNA/1','B06_RX/2'); add_line(parent,'P_NF/1','B06_RX/3');
add_line(parent,'B06_RX/1','B07_ADC/1'); add_line(parent,'P_ENOB/1','B07_ADC/2'); add_line(parent,'B07_ADC/1','RF_Out/1');
add_all_test_points(parent,{'B01_Environment','B02_Waveform','B03_TX_Chain','B04_Channel','B05_Coupling','B06_RX','B07_ADC'},1);
end

function build_dsp(parent,cfg)
add_block('simulink/Ports & Subsystems/In1',[parent '/RF_In'],'Position',[20 300 50 320]);
add_block('simulink/Ports & Subsystems/Out1',[parent '/Report_Out'],'Position',[1190 300 1220 320]);
add_const(parent,'P_NAVG','GPR_N_AVG',[20 40 90 65]); add_const(parent,'P_KAISER','GPR_KAISER_BETA',[20 80 90 105]);
add_const(parent,'P_FSTART','GPR_F_START',[20 120 90 145]); add_const(parent,'P_FSTOP','GPR_F_STOP',[20 160 90 185]);
add_const(parent,'P_MOIST','GPR_SOIL_MOISTURE',[20 200 90 225]); add_const(parent,'P_SIGMA','GPR_SOIL_SIGMA',[20 240 90 265]);
add_const(parent,'P_DMIN','GPR_DEPTH_MIN',[20 520 90 545]); add_const(parent,'P_DMAX','GPR_DEPTH_MAX',[20 560 90 585]);
add_const(parent,'P_PFA','GPR_PFA',[20 600 90 625]); add_const(parent,'P_SCAN','GPR_SCAN_LENGTH',[20 640 90 665]);
add_mlfcn(parent,'B08_Averaging',[180 280 380 330],code_averaging(cfg));
add_mlfcn(parent,'B09_Calibration',[430 280 630 330],code_calibration(cfg));
add_mlfcn(parent,'B10_RangeProc',[680 280 880 330],code_rangeproc(cfg));
add_mlfcn(parent,'B11_Background',[930 280 1130 330],code_background(cfg));
add_mlfcn(parent,'B12_Migration',[430 420 690 480],code_migration(cfg));
add_mlfcn(parent,'B13_Detection',[740 420 1000 480],code_detection(cfg));
add_mlfcn(parent,'B14_Report',[1040 420 1240 480],code_report(cfg));
add_line(parent,'RF_In/1','B08_Averaging/1'); add_line(parent,'P_NAVG/1','B08_Averaging/2');
add_line(parent,'B08_Averaging/1','B09_Calibration/1'); add_line(parent,'P_FSTART/1','B09_Calibration/2'); add_line(parent,'P_FSTOP/1','B09_Calibration/3');
add_line(parent,'B09_Calibration/1','B10_RangeProc/1'); add_line(parent,'P_KAISER/1','B10_RangeProc/2');
add_line(parent,'B10_RangeProc/1','B11_Background/1'); add_line(parent,'B11_Background/1','B12_Migration/1');
add_line(parent,'P_FSTART/1','B12_Migration/2'); add_line(parent,'P_FSTOP/1','B12_Migration/3'); add_line(parent,'P_MOIST/1','B12_Migration/4'); add_line(parent,'P_SIGMA/1','B12_Migration/5'); add_line(parent,'P_DMAX/1','B12_Migration/6'); add_line(parent,'P_SCAN/1','B12_Migration/7');
add_line(parent,'B12_Migration/1','B13_Detection/1'); add_line(parent,'P_DMIN/1','B13_Detection/2'); add_line(parent,'P_DMAX/1','B13_Detection/3'); add_line(parent,'P_PFA/1','B13_Detection/4'); add_line(parent,'P_FSTART/1','B13_Detection/5'); add_line(parent,'P_FSTOP/1','B13_Detection/6'); add_line(parent,'P_MOIST/1','B13_Detection/7'); add_line(parent,'P_SIGMA/1','B13_Detection/8');
add_line(parent,'B13_Detection/1','B14_Report/1'); add_line(parent,'P_FSTART/1','B14_Report/2'); add_line(parent,'P_FSTOP/1','B14_Report/3'); add_line(parent,'P_MOIST/1','B14_Report/4'); add_line(parent,'P_SIGMA/1','B14_Report/5'); add_line(parent,'P_SCAN/1','B14_Report/6'); add_line(parent,'B14_Report/1','Report_Out/1');
add_all_test_points(parent,{'B08_Averaging','B09_Calibration','B10_RangeProc','B11_Background','B12_Migration','B13_Detection','B14_Report'},8);
end

function add_const(parent,name,value,pos)
add_block('simulink/Sources/Constant',[parent '/' name],'Position',pos,'Value',value);
end
function add_mlfcn(parent,name,pos,script)
path=[parent '/' name]; add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',pos);
set_param(bdroot(parent),'SimulationCommand','update');
chart=sfroot().find('-isa','Stateflow.EMChart','-and','Path',path);
if isempty(chart), error('Cannot find MATLAB Function chart %s.',path); end
chart.Script=script;
end
function add_all_test_points(parent,blocks,first_index)
for k=1:numel(blocks)
    b=blocks{k}; idx=first_index+k-1; base=sprintf('TP%02d_%s',idx,b);
    pos=get_param([parent '/' b],'Position'); x=pos(3)+35; y=pos(2);
    add_block('simulink/Sinks/To Workspace',[parent '/' base '_LOG'],'Position',[x y x+130 y+25], ...
        'VariableName',lower(base),'SaveFormat','Structure With Time');
    add_block('simulink/Sinks/Scope',[parent '/' base '_SCOPE'],'Position',[x y+35 x+130 y+75]);
    add_line(parent,[b '/1'],[base '_LOG/1'],'autorouting','on');
    add_line(parent,[b '/1'],[base '_SCOPE/1'],'autorouting','on');
    ph=get_param([parent '/' b],'PortHandles');
    for q=1:numel(ph.Outport)
        set_param(ph.Outport(q),'DataLogging','on','DataLoggingName',base);
    end
end
end
