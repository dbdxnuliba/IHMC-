%% test for t prediction

% z0 = 1.08;
% dz0 = 0.05;
% zmax = 1.15;
% zmin = 1.00;
% ddzmax = 0.7*9.81;
% ddzmin = -0.4*9.81;
% gstar = -ddzmin;
% 
% a = (1/2)*(ddzmax + (ddzmax^2)/gstar);
% b = dz0 + dz0*ddzmax/gstar;
% c = z0-zmax + (1/2)*(dz0^2)/gstar;
% 
% tmaxhi = (-b + sqrt(b^2-4*a*c))/(2*a)
% tmaxlo = (-b - sqrt(b^2-4*a*c))/(2*a)
% 
% dz0 = -0.05;
% 
% a = (1/2)*(ddzmin - (ddzmin^2)/ddzmax);
% b = dz0 + dz0*ddzmin/ddzmax;
% c = z0-zmin - (1/2)*(dz0^2)/ddzmax;
% 
% tminhi = (-b + sqrt(b^2-4*a*c))/(2*a)
% tminlo = (-b - sqrt(b^2-4*a*c))/(2*a)
% 
% %% 360 push 
% angles = table2array(angleAndPercentWeight72(:,1));
% weights = table2array(angleAndPercentWeight72(:,2));
% 
% incr = length(angles)-1;
% 
% for i = 1:incr
%     angle = angles(i,:);
%     weight = weights(i,:);
%     plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
%     hold on;
%     nextangle = anles(i+1,:);
%     nextweight = weights(i+1,:);
%     plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     hold on;
% end
% angle = angles(72,:);
%     weight = weights(72,:);
%     nextangle = angles(1,:);
%     nextweight = weights(1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
% 
% %%
% angles = table2array(angleAndPercentWeight72(:,1));
% weights = table2array(angleAndPercentWeight72(:,2));
% 
% incr = length(angles)-1;
% 
% for i = 1:incr
%     angle = angles(i,:);
%     weight = weights(i,:);
%     plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
%     hold on;
%     nextangle = angles(i+1,:);
%     nextweight = weights(i+1,:);
%     plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     hold on;
% end
% angle = angles(72,:);
%     weight = weights(72,:);
%     nextangle = angles(1,:);
%     nextweight = weights(1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
% 
% 
% angles = table2array(angleAndPercentWeight72HeightTiles(:,1));
% weights = table2array(angleAndPercentWeight72HeightTiles(:,2));
% 
% incr = length(angles)-1;
% 
% for i = 1:incr
%     angle = angles(i,:);
%     weight = weights(i,:);
%     hold on;
%     nextangle = angles(i+1,:);
%     nextweight = weights(i+1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     hold on;
% end
% angle = angles(72,:);
%     weight = weights(72,:);
%     nextangle = angles(1,:);
%     nextweight = weights(1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
% 
%     %%
%     
%  figure;
% angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,1));
% weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,2));
% 
% incr = length(angles)-1;
% 
% for i = 1:incr
%     angle = angles(i,:);
%     weight = weights(i,:);
%     plot([0 weight*cos(angle)], [0 weight*sin(angle)],'Color',[0.4 0.4 0.4])
%     hold on;
%     nextangle = angles(i+1,:);
%     nextweight = weights(i+1,:);
%     plot([0 nextweight*cos(nextangle)], [0 nextweight*sin(nextangle)],'Color',[0.4 0.4 0.4])
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     hold on;
% end
% angle = angles(72,:);
%     weight = weights(72,:);
%     nextangle = angles(1,:);
%     nextweight = weights(1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
% 
% 
% angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,1));
% weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,2));
% 
% incr = length(angles)-1;
% 
% for i = 1:incr
%     angle = angles(i,:);
%     weight = weights(i,:);
%     hold on;
%     nextangle = angles(i+1,:);
%     nextweight = weights(i+1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     hold on;
% end
% angle = angles(72,:);
%     weight = weights(72,:);
%     nextangle = angles(1,:);
%     nextweight = weights(1,:);
%     plot([weight*cos(angle) nextweight*cos(nextangle)],[weight*sin(angle) nextweight*sin(nextangle)],'Color','k');
%     
%     
%     %%
%         %%
%     
%  figure;
% angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,1));
% weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWayNormal(:,2));
% 
% wHNav = mean(weights)
% 
% polarplot(angles,weights);
% ax = gca;
% ax.ThetaTick = [0:20:340];
% ax.ThetaTickMode = 'manual';
% ax.ThetaMinorGrid = 'on';
% ax.RMinorGrid = 'on';
% ax.RLim = [0 2.5];
% hold on;
% angle = [angles(72) angles(1)];
% weight = [weights(72) weights(1)];
% polarplot(angle,weight);
% 
% angles = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,1));
% weights = table2array(angleAndPercentWeight72TilesV2NormalHalfWay(:,2));
% 
% wHav = mean(weights)
% 
% polarplot(angles,weights);
% angle = [angles(72) angles(1)];
% weight = [weights(72) weights(1)];
% polarplot(angle,weight);

%%

f = 9.81*0.05*0.6;
        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles10(:,1));
weights = table2array(angleAndPercentWeight72Tiles10(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);
rlim([0 0.6])

angles = table2array(angleAndPercentWeight72Tiles10Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10Ang(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngNormal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngNormal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng00.eps', opts)


        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles1001(:,1));
weights = table2array(angleAndPercentWeight72Tiles1001(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

rlim([0 0.6])

angles = table2array(angleAndPercentWeight72Tiles1001Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles1001Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10Ang01(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang01(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10Ang01Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang01Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng01.eps', opts)

        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles10Quart(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Quart(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);
rlim([0 0.6])

angles = table2array(angleAndPercentWeight72Tiles10QuartNormal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10QuartNormal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngQuart(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngQuart(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngQuartNormal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngQuartNormal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng02.eps', opts)
        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles1003(:,1));
weights = table2array(angleAndPercentWeight72Tiles1003(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles1003Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles1003Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);
rlim([0 0.6])
angles = table2array(angleAndPercentWeight72Tiles10Ang03(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang03(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngNormal1(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngNormal1(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng03.eps', opts)

        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles10Half(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Half(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);
rlim([0 0.6])

angles = table2array(angleAndPercentWeight72Tiles10HalfNormal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10HalfNormal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngHalf(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngHalf(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10AngHalfNormal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10AngHalfNormal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng04.eps', opts)

        %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(angleAndPercentWeight72Tiles1005(:,1));
weights = table2array(angleAndPercentWeight72Tiles1005(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights);
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);
rlim([0 0.6])

angles = table2array(angleAndPercentWeight72Tiles1005Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles1005Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10Ang05(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang05(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

angles = table2array(angleAndPercentWeight72Tiles10Ang05Normal(:,1));
weights = table2array(angleAndPercentWeight72Tiles10Ang05Normal(:,2))*f;

wav = mean(weights)

polarplot(angles,weights);
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight);

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAng05.eps', opts)

%% WITH ACTIONS NO ANG
     %%
     
     col = [0.9 0.5 0.2;
            0.2 0.7 0.2;
            0.3 0.6 0.9];
     f = 9.81*0.05*0.6;   
     %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;

angles = table2array(roundPushPos085Ang(:,1));
weights = table2array(roundPushPos085Ang(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos085(:,1));
weights = table2array(roundPushPos085(:,2))*f;
actions = table2array(roundPushPos085(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction00.eps', opts)


    %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(roundPush01Normal(:,1));
weights = table2array(roundPush01Normal(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos08501(:,1));
weights = table2array(roundPushPos08501(:,2))*f;
actions = table2array(roundPushPos08501(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction01.eps', opts)

    %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(roundPushQuartNormal(:,1));
weights = table2array(roundPushQuartNormal(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos085Quart(:,1));
weights = table2array(roundPushPos085Quart(:,2))*f;
actions = table2array(roundPushPos085Quart(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction02.eps', opts)

    %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(roundPush03Normal(:,1));
weights = table2array(roundPush03Normal(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos08503(:,1));
weights = table2array(roundPushPos08503(:,2))*f;
actions = table2array(roundPushPos08503(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction03.eps', opts)

    %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(roundPushHalfNormal(:,1));
weights = table2array(roundPushHalfNormal(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos085Half(:,1));
weights = table2array(roundPushPos085Half(:,2))*f;
actions = table2array(roundPushPos085Half(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction04.eps', opts)

  %%
    set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
figure;
angles = table2array(roundPushPos085Ang05(:,1));
weights = table2array(roundPushPos085Ang05(:,2))*f;

wNav = mean(weights)

polarplot(angles,weights,'Color','r');
hold on;
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','r');
rlim([0 0.5])

angles = table2array(roundPushPos08505(:,1));
weights = table2array(roundPushPos08505(:,2))*f;
actions = table2array(roundPushPos08505(:,3));

wav = mean(weights)

polarplot(angles,weights,'Color','b');
angle = [angles(72) angles(1)];
weight = [weights(72) weights(1)];
polarplot(angle,weight,'Color','b');
hold on;
for i =1:71
polarplot([angles(i) angles(i+1)], [0 weights(i+1)],'Color', col(actions(i+1)+1,:));
end
polarplot([angles(end) angles(1)], [0, weights(1)], 'Color', col(actions(1)+1,:));

ax = gca;
ax.ThetaTickMode = 'manual';
ax.ThetaTick = [0:20:340];
ax.ThetaMinorGrid = 'on';

ax.RMinorGrid = 'on';

opts.Format = 'eps';
opts.Color = 'CMYK';
opts.Resolution = 10000000;
exportfig(gcf,'roundSSAction05.eps', opts)