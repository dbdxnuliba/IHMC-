% Hidden Point Removal Test
% used to verify the Java implementation against a well-known resource.

% The Java version would be used to remove hidden points of a point cloud 
% so the result is similar to a LIDAR view of a structure/object.

% Resources:
% https://www.mathworks.com/matlabcentral/fileexchange/16581-hidden-point-removal
% https://www.isprs-ann-photogramm-remote-sens-spatial-inf-sci.net/II-5/9/2014/isprsannals-II-5-9-2014.pdf
% http://vecg.cs.ucl.ac.uk/Projects/SmartGeometry/robustPointVisibility/paper_docs/VisibilityOfNoisyPointCloud_small.pdf
% http://www.cloudcompare.org/

close all; clear all; clc
pointSize = .5;

[X,Y,Z] = HPRdata();
figure
scatter3(X,Z,Y,pointSize);

p = [X; Y; Z]';                         %NxD D dimensional point cloud.                    
C = [-10,5,5];                          %1xD  D dimensional Camera viewpoint.
param = 2.3;                            %param - parameter for the algorithm. Indirectly sets the radius.

figure('NumberTitle', 'off', 'Name', 'Random Staircase HPR - Blue: Matlab, Red: Java');
vp = HPR(p,C,param);                    %visiblePtInds - indices of p that are visible from C.
scatter3(X(vp), Z(vp), Y(vp), pointSize);
hold on;

[X,Y,Z] = HPR_JavaResult();
scatter3(X-10,Z+5,Y+5,pointSize);


