%% Open-loop, deadband, and closed-loop plots

clearvars; close all; clc;

% Heatmap parameters
colorUpperLimit = 0.20;
errorUpperLimit = 0.25;

% Plot results
fig1 = figure(1); 
set(fig1,'units','normalized','outerposition',[0.1 0.1 1 0.75]);
       
% Open-loop steady-state errors
subplot(1,3,1)
load('rocPlot_000deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
h1 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
h1.ColorbarVisible = 'off'; h1.GridVisible = 'off';
ylabel('Initial Deviation (Foil Lengths): z_2(0) - r_2(0)'); title('\rm Open-Loop');
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';

% Closed-loop with deadband steady-state errors
load('rocPlot_000deg_4foils_deadband.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;

subplot(1,3,2)
h2 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
h2.ColorbarVisible = 'off'; h2.GridVisible = 'off';
xlabel('Reference Trajectory Initial Position (Foil Lengths): r_2(0)'); 
title('\rm Closed-Loop with Deadband');
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Closed-loop without deadband steady-state errors
load('rocPlot_000deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;

subplot(1,3,3)
h3 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
     'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
title('\rm Closed-Loop without Deadband');
h3.GridVisible = 'off'; 
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Reformatting
subplotSpacing = 0.27; subplotWidth = 0.25; subplotHeight = 0.6;
h1.Position(1) = 0.08; h1.Position(3) = subplotWidth;
h2.Position(1) = h1.Position(1) + subplotSpacing; h2.Position(3) = subplotWidth;
h3.Position(1) = h2.Position(1) + subplotSpacing; h3.Position(3) = subplotWidth;
h1.Position(4) = subplotHeight; h2.Position(4) = subplotHeight; h3.Position(4) = subplotHeight;

% Annotations
colorbarAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
             'Steady-State Error (Foil Lengths)','HeadStyle','none',...
             'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
             'FontSize',18,'FontName','Times New Roman','Position',[1.01 0.42 0 0]);

% Heatmap parameters
colorUpperLimit = 0.20;
errorUpperLimit = 0.25;

% Plot results
fig2 = figure(2); 
set(fig2,'units','normalized','outerposition',[0.1 0.1 1 0.75]);
       
% Open-loop steady-state errors
subplot(1,3,1)
load('rocPlot_180deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(errorReduction<0.5*abs(Z_02.')) = NaN;
h1 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
h1.ColorbarVisible = 'off'; h1.GridVisible = 'off';
ylabel('Initial Deviation (Foil Lengths): z_2(0) - r_2(0)'); title('\rm Open-Loop');
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';

% Closed-loop with deadband steady-state errors
load('rocPlot_180deg_4foils_deadband.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;

subplot(1,3,2)
h2 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
h2.ColorbarVisible = 'off'; h2.GridVisible = 'off';
xlabel('Reference Trajectory Initial Position (Foil Lengths): r_2(0)'); 
title('\rm Closed-Loop with Deadband');
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Closed-loop without deadband steady-state errors
load('rocPlot_180deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;

subplot(1,3,3)
h3 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',jet,...
     'ColorLimits',[0 colorUpperLimit],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
title('\rm Closed-Loop without Deadband');
h3.GridVisible = 'off'; 
ax = gca; ax.FontSize = 18; ax.FontName = 'Times New Roman';
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Reformatting
subplotSpacing = 0.27; subplotWidth = 0.25; subplotHeight = 0.6;
h1.Position(1) = 0.08; h1.Position(3) = subplotWidth;
h2.Position(1) = h1.Position(1) + subplotSpacing; h2.Position(3) = subplotWidth;
h3.Position(1) = h2.Position(1) + subplotSpacing; h3.Position(3) = subplotWidth;
h1.Position(4) = subplotHeight; h2.Position(4) = subplotHeight; h3.Position(4) = subplotHeight;

% Annotations
colorbarAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
             'Steady-State Error (Foil Lengths)','HeadStyle','none',...
             'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
             'FontSize',18,'FontName','Times New Roman','Position',[1.01 0.42 0 0]);
         
%% Open-loop and closed-loop results

clearvars; close all; clc;

% Heatmap parameters
colorUpperLimit = 0.20;
errorUpperLimit = 0.25;
tickSize = 16; fontSize = 22;

% Plot results
fig3 = figure(3); 
set(fig3,'units','normalized','outerposition',[0 0 0.8 1]);
       
% Load and clean data files
load('rocPlot_000deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_000_OL = errorReduction;

load('rocPlot_000deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_000_CL = errorReduction;

load('rocPlot_180deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_180_OL = errorReduction;

load('rocPlot_180deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction(errorReduction > 0.13) = NaN;
errorReduction_180_CL = errorReduction;

% Open-loop in-phase steady-state errors
subplot(2,2,1)
h11 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_000_OL),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h11.ColorbarVisible = 'off'; h11.GridVisible = 'off';
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';
ax.XDisplayLabels = nan(size(ax.XDisplayData));

% Closed-loop in-phase steady-state errors
subplot(2,2,2)
h12 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_000_CL),'colormap',jet,...
     'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h12.GridVisible = 'off'; 
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';
ax.XDisplayLabels = nan(size(ax.XDisplayData));
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Open-loop out-of-phase steady-state errors
subplot(2,2,3)
h21 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_180_OL),'colormap',jet,...
      'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h21.ColorbarVisible = 'off'; h21.GridVisible = 'off';
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';

% Closed-loop out-of-phase steady-state errors
subplot(2,2,4)
h22 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_180_CL),'colormap',jet,...
     'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h22.GridVisible = 'off'; 
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';
ax.YDisplayLabels = nan(size(ax.YDisplayData));

% Reformatting parameters
leftIndex = 0.08; bottomIndex = 0.085;
verticalSpacing = 0.44; horizontalSpacing = 0.36;
subplotWidth = 0.35; subplotHeight = 0.43;

% Apply reformatting
h11.Position(1) = leftIndex; h11.Position(2) = bottomIndex + verticalSpacing;
h11.Position(3) = subplotWidth; h11.Position(4) = subplotHeight;

h12.Position(1) = leftIndex + horizontalSpacing; h12.Position(2) = bottomIndex + verticalSpacing;
h12.Position(3) = subplotWidth; h12.Position(4) = subplotHeight;

h21.Position(1) = leftIndex; h21.Position(2) = bottomIndex;
h21.Position(3) = subplotWidth; h21.Position(4) = subplotHeight;

h22.Position(1) = leftIndex + horizontalSpacing; h22.Position(2) = bottomIndex;
h22.Position(3) = subplotWidth; h22.Position(4) = subplotHeight;

% Annotations
colorbarAN1 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[0.987 0.31 0 0]);

colorbarAN2 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[0.987 0.755 0 0]);
          
xlabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Reference Trajectory Initial Position (Foil Lengths): r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',0,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.68 0.02 0 0]);

ylabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Initial Deviation (Foil Lengths): z_2(0) - r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',90,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.20 0.54 0 0]);

olTitle =  annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Open-Loop Results','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',0,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.335 0.975 0 0]);
 
clTitle =  annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Closed-Loop Results','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',0,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.705 0.975 0 0]);
       
%% In-phase results

clearvars; close all; clc;

% Heatmap parameters
colorUpperLimit = 0.20;
errorUpperLimit = 0.25;
tickSize = 16; fontSize = 22;
       
% Load and clean data files
load('rocPlot_000deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_000_OL = errorReduction;

load('rocPlot_000deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_000_CL = errorReduction;

% Plot results
fig4 = figure(4); 
set(fig4,'units','normalized','outerposition',[0 0 0.45 1]);

% Open-loop in-phase steady-state errors
subplot(2,1,1)
h11 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_000_OL),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h11.GridVisible = 'off';
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';
ax.XDisplayLabels = nan(size(ax.XDisplayData));

% Closed-loop in-phase steady-state errors
subplot(2,1,2)
h12 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_000_CL),'colormap',jet,...
      'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h12.GridVisible = 'off'; 
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';

% Reformatting parameters
leftIndex = 0.125; bottomIndex = 0.085; verticalSpacing = 0.455;
subplotWidth = 0.68; subplotHeight = 0.445;

% Apply reformatting
h11.Position(1) = leftIndex; h11.Position(2) = bottomIndex + verticalSpacing;
h11.Position(3) = subplotWidth; h11.Position(4) = subplotHeight;

h12.Position(1) = leftIndex; h12.Position(2) = bottomIndex;
h12.Position(3) = subplotWidth; h12.Position(4) = subplotHeight;

% Annotations
colorbarAN1 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.145 0.31 0 0]);

colorbarAN2 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.145 0.765 0 0]);

closedAN =  annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Closed-Loop','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.045 0.31 0 0]);

openAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Open-Loop','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.04 0.765 0 0]);

xlabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Reference Trajectory Initial Position (Foil Lengths): r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',0,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.88 0.02 0 0]);

ylabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Initial Deviation (Foil Lengths): z_2(0) - r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',90,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.345 0.54 0 0]);       

%% Out-of-phase results

clearvars; close all; clc;

% Heatmap parameters
colorUpperLimit = 0.20;
errorUpperLimit = 0.25;
tickSize = 16; fontSize = 22;

load('rocPlot_180deg_4foils_openLoop.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction_180_OL = errorReduction;

load('rocPlot_180deg_4foils.mat');
errorReduction(errorReduction>errorUpperLimit) = NaN;
errorReduction(:,10:12) = NaN;
errorReduction(errorReduction > 0.13) = NaN;
errorReduction_180_CL = errorReduction;

% Plot results
fig4 = figure(4); 
set(fig4,'units','normalized','outerposition',[0 0 0.45 1]);

% Open-loop in-phase steady-state errors
subplot(2,1,1)
h21 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_180_OL),'colormap',jet,...
    'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h21.GridVisible = 'off';
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';
ax.XDisplayLabels = nan(size(ax.XDisplayData));

% Closed-loop in-phase steady-state errors
subplot(2,1,2)
h22 = heatmap(R_02,fliplr(Z_02),flipud(errorReduction_180_CL),'colormap',jet,...
      'ColorLimits',[0 colorUpperLimit],'CellLabelColor','none'); 
h22.GridVisible = 'off'; 
ax = gca; ax.FontSize = tickSize; ax.FontName = 'Times New Roman';

% Reformatting parameters
leftIndex = 0.125; bottomIndex = 0.085; verticalSpacing = 0.455;
subplotWidth = 0.68; subplotHeight = 0.445;

% Apply reformatting
h21.Position(1) = leftIndex; h21.Position(2) = bottomIndex + verticalSpacing;
h21.Position(3) = subplotWidth; h21.Position(4) = subplotHeight;

h22.Position(1) = leftIndex; h22.Position(2) = bottomIndex;
h22.Position(3) = subplotWidth; h22.Position(4) = subplotHeight;

% Annotations
colorbarAN1 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.145 0.31 0 0]);

colorbarAN2 = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Steady-State Error (Foil Lengths)','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.145 0.765 0 0]);

closedAN =  annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Closed-Loop','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.045 0.31 0 0]);

openAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
              'Open-Loop','HeadStyle','none',...
              'LineStyle','none','HorizontalAlignment','center','TextRotation',270,...
              'FontSize',fontSize,'FontName','Times New Roman',...
              'Position',[1.04 0.765 0 0]);

xlabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Reference Trajectory Initial Position (Foil Lengths): r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',0,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.88 0.02 0 0]);

ylabelAN = annotation('textarrow',[1,1],[0.5,0.5],'string',...
           'Initial Deviation (Foil Lengths): z_2(0) - r_2(0)','HeadStyle','none',...
           'LineStyle','none','HorizontalAlignment','center','TextRotation',90,...
           'FontSize',fontSize,'FontName','Times New Roman',...
           'Position',[0.345 0.54 0 0]);
