function corners = getCornersSpread(img,existing_points, num_corners, params)
%GETCORNERSSPREAD Summary of this function goes here
%   Detailed explanation goes here
subdivisions = [4,6];
w = floor(params.image_size(2)/subdivisions(1));
h = floor(params.image_size(1)/subdivisions(2));
ul_x_vec = 1:w:params.image_size(2)-w+1;
ul_y_vec = 1:h:params.image_size(1)-h+1;

c_location = [];
c_metric = [];
for ul_x = ul_x_vec
    for ul_y = ul_y_vec
        ROI = [ul_x, ul_y, w , h];
%         polygon_x = [ul_x, ul_x+w, ul_x+w, ul_x, ul_x];
%         polygon_y = [ul_y, ul_y, ul_y+h, ul_y+h, ul_y];
%         if existing_points
% %             figure
% %             plot(polygon_x,polygon_y);
%             n_pts_in_ROI = sum(inpolygon(existing_points(1,:),...
%                     existing_points(2,:),polygon_x,polygon_y));
%         else
%             n_pts_in_ROI = 0;
%         end
        select_num = 20; %max(0, 10-n_pts_in_ROI);
        if select_num
            c = detectHarrisFeatures(img, 'ROI', ROI);
            c_location = [c_location; c.selectStrongest(select_num).Location];
            c_metric = [c_metric; c.selectStrongest(select_num).Metric];
        end
    end 
end
corners = cornerPoints(c_location, 'Metric', c_metric);

if existing_points
    D = pdist2(existing_points', corners.Location, params.proposal_test_norm, 'Smallest', 1);
    distinct_points = D > 10;
    corners = corners(distinct_points);
end
% corners = detectHarrisFeatures(img, 'MinQuality', 0.00001, 'FilterSize', 3);
% corners = detectFASTFeatures(img, 'MinContrast', 0.001);
corners = corners.selectUniform(num_corners,params.image_size);
end

