
% HPR - Using HPR ("Hidden Point Removal) method, approximates a visible subset of points 
% as viewed from a given viewpoint.
% Usage:
% visiblePtInds=HPR(p,C,param)
%
% Input:
% p - NxD D dimensional point cloud.
% C - 1xD D dimensional viewpoint.
% param - parameter for the algorithm. Indirectly sets the radius.
%
% Output:
% visiblePtInds - indices of p that are visible from C.
%
% This code was written by Sagi Katz
% sagikatz@tx.technion.ac.il
% Technion, 2006.
% For more information, see "Direct Visibility of Point Sets", Katz S., Tal
% A. and Basri R., SIGGRAPH 2007, ACM Transactions on Graphics, Volume 26, Issue 3, August 2007.
%
% This method is patent pending.

function visiblePtInds = HPR(p,C,param)
dim = size(p,2);
numPts = size(p,1);
p = p-repmat(C,[numPts 1]);                                       %Move C to the origin
normp = sqrt(dot(p,p,2));                                         %Calculate ||p||
R = repmat(max(normp)*(10^param),[numPts 1]);                     %Sphere radius
P = p+2*repmat(R-normp,[1 dim]).*p./repmat(normp,[1 dim]);        %Spherical flipping

%pp = [P;zeros(1,dim)];
%xx = convhulln([P;zeros(1,dim)]);
%yy = unique(xx);
%figure
%scatter3(xx(1), xx(2), xx(3));

% convhulln returns the indices of the three corners of triangle making up
% the outer (convex) hull
% unique returns the sorted list of unrepeated indices in the hull list 
visiblePtInds = unique(convhulln([P;zeros(1,dim)]));              %convex hull
visiblePtInds(visiblePtInds==numPts+1) = [];
end
