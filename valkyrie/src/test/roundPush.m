%% test for t prediction

z0 = 1.08;
dz0 = 0.05;
zmax = 1.15;
zmin = 1.00;
ddzmax = 0.7*9.81;
ddzmin = -0.4*9.81;
gstar = -ddzmin;

a = (1/2)*(ddzmax + (ddzmax^2)/gstar);
b = dz0 + dz0*ddzmax/gstar;
c = z0-zmax + (1/2)*(dz0^2)/gstar;

tmaxhi = (-b + sqrt(b^2-4*a*c))/(2*a)
tmaxlo = (-b - sqrt(b^2-4*a*c))/(2*a)

dz0 = -0.05;

a = (1/2)*(ddzmin - (ddzmin^2)/ddzmax);
b = dz0 + dz0*ddzmin/ddzmax;
c = z0-zmin - (1/2)*(dz0^2)/ddzmax;

tminhi = (-b + sqrt(b^2-4*a*c))/(2*a)
tminlo = (-b - sqrt(b^2-4*a*c))/(2*a)

%% 360 push 
angles = table2array(angleAndPercentWeight72(:,1));
weights = table2array(angleAndPercentWeight72(:,2));

incr = length(angles)-1;

for i = 1:incr
    angle = angles(i,:);
    weight = weights(i,:);
    plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
    hold on;
    nextangle = angles(i+1,:);
    nextweight = weights(i+1,:);
    plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    hold on;
end
angle = angles(72,:);
    weight = weights(72,:);
    nextangle = angles(1,:);
    nextweight = weights(1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');

%%
angles = table2array(angleAndPercentWeight72(:,1));
weights = table2array(angleAndPercentWeight72(:,2));

incr = length(angles)-1;

for i = 1:incr
    angle = angles(i,:);
    weight = weights(i,:);
    plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
    hold on;
    nextangle = angles(i+1,:);
    nextweight = weights(i+1,:);
    plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    hold on;
end
angle = angles(72,:);
    weight = weights(72,:);
    nextangle = angles(1,:);
    nextweight = weights(1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');


angles = table2array(angleAndPercentWeight72HeightTiles(:,1));
weights = table2array(angleAndPercentWeight72HeightTiles(:,2));

incr = length(angles)-1;

for i = 1:incr
    angle = angles(i,:);
    weight = weights(i,:);
    hold on;
    nextangle = angles(i+1,:);
    nextweight = weights(i+1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    hold on;
end
angle = angles(72,:);
    weight = weights(72,:);
    nextangle = angles(1,:);
    nextweight = weights(1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');

    %%
    
 figure;
angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,1));
weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,2));

incr = length(angles)-1;

for i = 1:incr
    angle = angles(i,:);
    weight = weights(i,:);
    plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
    hold on;
    nextangle = angles(i+1,:);
    nextweight = weights(i+1,:);
    plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    hold on;
end
angle = angles(72,:);
    weight = weights(72,:);
    nextangle = angles(1,:);
    nextweight = weights(1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');


angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,1));
weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,2));

incr = length(angles)-1;

for i = 1:incr
    angle = angles(i,:);
    weight = weights(i,:);
    hold on;
    nextangle = angles(i+1,:);
    nextweight = weights(i+1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    hold on;
end
angle = angles(72,:);
    weight = weights(72,:);
    nextangle = angles(1,:);
    nextweight = weights(1,:);
    plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
    
    
    %%
        %%
    
 figure;
angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,1));
weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,2));

wHNav = mean(weights)

polarplot(angles,weights);
ax = gca;
ax.ThetaTick = [0:20:340];
ax.ThetaTickMode = 'manual';
ax.ThetaMinorGrid = 'on';
ax.RMinorGrid = 'on';
ax.RLim = [0 2.5];
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,1));
weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,2));

wHav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

%%
        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight36TilesStanding24(:,1));
weights = table2array(angleAndPercentWeight36TilesStanding24(:,2));

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(18) angles(1)];
weight = [weights(18) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight36TilesStanding24Normal(:,1));
weights = table2array(angleAndPercentWeight36TilesStanding24Normal(:,2));

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(18) angles(1)];
weight = [weights(18) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSS.eps', opts)


