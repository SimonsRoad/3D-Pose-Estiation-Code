% function to calculate average square root distance between two images
function ave = averageDistance(a, b)
d2 = (a-b).*(a-b);
d2 = sum(d2')';
d = sqrt(d2);
ave = sum(d)/length(d);
end
