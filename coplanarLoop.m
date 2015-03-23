% function to do the iteration in coplanar posit
function [rot, trans, error] = coplanarLoop(imagePoints, objectPoints, objectVectors, objectMatrix, u, nbPoints, focalLength, rotation, translation)

converged = 0;
count = 0;
Threshold = 2;
oldE = 999;
while ~converged
    % calculate new epsilon
    epsilon = (1 + (objectVectors * rotation(3,:)')/translation(3) );
    epsilon2 = epsilon * ones(1, 2);
    SOPImagePoints = imagePoints .* epsilon2;
    % calculate image vectors
    imageVectors = SOPImagePoints - ones(nbPoints,1) * SOPImagePoints(1,:);


	IJ = (objectMatrix * imageVectors)';
	I0 = IJ(1,:);
    J0 = IJ(2,:);
    
    % solve two vectors I1 J1, I2, J2
    [I1, J1, I2, J2] = solve2(I0, J0, u);

    %first solution
    IVect1 = I1;
    JVect1 = J1;
	ISquare1 = IVect1*IVect1';
	JSquare1 = JVect1*JVect1';
	scale11 = sqrt(ISquare1); scale21 = sqrt(JSquare1);
	row11 = IVect1/scale11; row21 = JVect1/scale21;
	row31 = cross(row11, row21);
	rotation1 = [row11; row21; row31];
	scale1 = (scale11 + scale21)/2.0;
	translation1 = [imagePoints(1,:), focalLength]/scale1;
		
    % calculate approximate image points using the pose just calculated
    [imagepoint1, valid1] = poseTrans(objectPoints, rotation1, translation1, focalLength);

    if(valid1)
        E1 = averageDistance(imagePoints, imagepoint1);
    else
        E1 = 999;
    end
    
    
    % second solution   
    IVect2 = I2;
    JVect2 = J2;
	ISquare2 = IVect2*IVect2';
	JSquare2 = JVect2*JVect2';
	scale12 = sqrt(ISquare2); scale22 = sqrt(JSquare2);
	row12 = IVect2/scale12; row22 = JVect2/scale22;
	row32 = cross(row12, row22);
	rotation2 = [row12; row22; row32];
	scale2 = (scale12 + scale22)/2.0;
	translation2 = [imagePoints(1,:), focalLength]/scale2;
		
    
    [imagepoint2, valid2] = poseTrans(objectPoints, rotation2, translation2, focalLength);
    
    if(valid2)
        E2 = averageDistance(imagePoints, imagepoint2);
    else
        E2 = 999;
    end

    if(E1 < E2)
        rotation = rotation1;
        translation = translation1;
        E = E1;
    else
        rotation = rotation2;
        translation = translation2;
        E = E2;
    end
    
    if(E < Threshold)
        converged = 1;
    else
        if(E > oldE || E == oldE)
%           disp(' not converge ');
          E = 999;
          break;
        end
    end
    
    oldE = E;

 	disp(' ================ iteration ================ ');
    count = count + 1
    rotation
    translation
    E

    
%     [imagepoint valid]= poseTrans(objectPoints, rotation, translation, focalLength);
%     imagepoint
%     E = averageDistance(imagepoint,imagePoints)

end % while
% return one solution
rot = rotation;
trans = translation;
error = E;
