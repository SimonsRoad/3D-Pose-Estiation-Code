% function used to map object from 3D object coordinate system into image coordinate 
function [imagepoint valid]= poseTrans(worldpoint, rotation, translation, focalLength)

sz = size(worldpoint);
wi = ones(sz(1), 1);
homogeneousWorldPts = [worldpoint, wi];


transmat = [rotation translation'];

ip = transmat*homogeneousWorldPts';
zi = ip(3,:);
v = size(find(zi<0));
valid = 1- v(2);
zi2 = [zi;zi];
imagepoint = ip(1:2,:)*focalLength./zi2;
imagepoint = imagepoint';


end