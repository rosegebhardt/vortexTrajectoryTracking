% Plot region of convergence data as a heatmap

% Load data
load('rocPlot_000deg_4foils.mat');

% Plot results: will need to manually edit becuase of heatmap limitations
fig1 = figure(1); 
set(fig1,'units','normalized','outerposition',[0.1 0.1 1 0.8])
heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',turbo,'ColorLimits',[0 0.2],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
colorbar;
xlabel('Reference Trajectory Initial Position: r_2(t = 0)'); 
ylabel('Initial Deviation: z_2(t = 0) - r_2(t = 0)');
ax = gca; ax.FontSize = 24; ax.FontName = 'Times New Roman';

load('rocPlot_180deg_4foils.mat');
fig2 = figure(2); 
set(fig2,'units','normalized','outerposition',[0.1 0.1 1 0.8])
heatmap(R_02,fliplr(Z_02),flipud(errorReduction),'colormap',turbo,'ColorLimits',[0 0.2],'CellLabelFormat','%.3f');%,'CellLabelColor','none'); 
colorbar;
xlabel('Reference Trajectory Initial Position: r_2(t = 0)'); 
ylabel('Initial Deviation: z_2(t = 0) - r_2(t = 0)');
ax = gca; ax.FontSize = 24; ax.FontName = 'Times New Roman';