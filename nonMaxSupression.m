function P = nonMaxSupression(P_in,radius)
%nonMaxSupression Performs an inefficient non maximum supression of
%cornerPoints
%   P_in: candidate corner points
%   radius: threshold for neighbourhood distance (currently using manhattan)
[~, I] = sort(P_in.Metric,'descend');
P_sort = P_in(I);
k = 1;
while k < length(P_sort)
    p_cur = P_sort(k);
    D = pdist2(p_cur.Location, P_sort.Location, 'cityblock');
    close_points = D < radius; close_points(k) = 0;
    P_sort(close_points) = [];
    k = k+1;
end
P = P_sort;
    
end

