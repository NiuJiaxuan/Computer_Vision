% Problem 1

% load the checker.png
checker = double(imread('checker.png'));

% initial the viables 
sigma_I = 1;
sigma_D = 0.7;
alpha = 0.05;
T = 1000000;

% Set the mask size
ceilling = ceil(3 * sigma_D);
ax_size = 2 * ceilling + 1;
c_arr = 1:ax_size;
a_arr = -ceilling:ceilling;

% Initialize Gx and Gy
Gx = zeros(ax_size, ax_size);
Gy = zeros(ax_size, ax_size);

% fill mask values
for i = 1:numel(c_arr)
    y = a_arr(i);
    for j = 1:numel(c_arr)
        x = a_arr(j);
        Gx(i,j) = x * exp((-x * x - y * y)/(2 * sigma_D.^2)) / (2 * pi * sigma_D.^4);
        Gy(i,j) = y * exp((-x * x - y * y)/(2 * sigma_D.^2)) / (2 * pi * sigma_D.^4);
    end
end

% compute GX and Gy
Gx = Gx ./ sum(abs(Gx),'all');
Gy = Gy ./ sum(abs(Gy),'all');

% compute Ix and Iy
Ix = imfilter(checker, Gx, 'replicate');
Iy = imfilter(checker, Gy, 'replicate');

% compute Ix^2 Iy^2 IxIy
Ixx = Ix.^2;
Iyy = Iy.^2;
Ixy = Ix.*Iy;

% compute GIx^2 GIy^2 GIxIy
G = fspecial('gaussian', 2*ceil(3*sigma_I)+1, sigma_I);
gIxx = imfilter(Ixx, G, 'replicate');
gIyy = imfilter(Iyy, G, 'replicate');
gIxy = imfilter(Ixy, G, 'replicate');

% sue the cornerness function to compute R
R = gIxx .* gIyy - gIxy.^2 - alpha * (gIxx + gIyy).^2;

% display the value of R(17:23, 17:23)
disp(R(17:23, 17:23));

pause;

% remove the smaller and negative values in R
new_R = zeros(size(R,1),size(R,2));

for j = 1:1:size(R,1)
    for i = 1:1:size(R,2)
        if R(j,i) >= T
            new_R(j,i) = R(j,i);
        end
    end
end

% display the thresholded R
imagesc(new_R);
axis('image');
colormap("gray");

pause;


%do non-maximum suppression on R
[row, col] = size(new_R);
m = 1;

for y = 2:1:(row-1)
    for x = 2:1:(col-1)

         b = new_R((y-1):(y+1),(x-1):(x+1));

         max_b = max(max(b));
         is_corner = 0;

         for j = 1:1:3
             for i = 1:1:3
                 if b(j,i) == max_b
                    is_corner = is_corner + 1;
                 end
             end
         end

         if (is_corner == 1 && b(2,2) == max_b)
             nonmax_R(m,1) = y;
             nonmax_R(m,2) = x;
             m = m + 1;
         end

    end
end


fastX = nonmax_R(:,2);
fastY = nonmax_R(:,1);
imshow(uint8(checker));
hold on;
plot(fastX,fastY,'r.');
hold off;
pause;

%% 
% problem 2

% load the tower.png
img = double(imread('tower.png'));

% initial variables 
T = [10, 20, 30, 50];
n = 9;

% Display the image and overlay the FAST feature points
figure;
for i = 1:1:4
    subplot(2,2,i);
    t = T(i);
    corner = getCorners(img, t, n);
    fastX = corner(:,2);
    fastY = corner(:,1);
    imshow(uint8(img));
    hold on;
    plot(fastX,fastY,'r.');
    hold off;
    title("T = " + t);
end


%% function part 

% get the surrounding pixels
function [points] = circle(img, row, col, T)

    points(1) = img(row + 3, col);
    points(2) = img(row + 3, col + 1);
    points(3) = img(row + 2, col + 2);
    points(4) = img(row + 1, col + 3);
    points(5) = img(row, col + 3);
    points(6) = img(row - 1, col + 3);
    points(7) = img(row - 2, col + 2);
    points(8) = img(row - 3, col + 1);
    points(9) = img(row - 3, col);
    points(10) = img(row - 3, col - 1);
    points(11) = img(row - 2, col - 2);
    points(12) = img(row - 1, col - 3);
    points(13) = img(row, col - 3);
    points(14) = img(row + 1, col - 3);
    points(15) = img(row + 2, col - 2);
    points(16) = img(row + 3, col - 1);

    points = intensity(img,row,col,T,points);

end

function [points] = intensity(img, row, col, T,points)
    inten = img(row, col);
    for i = 1:16
        if points(i) < (inten - T)
            points(i) = 1;
            points(i + 16) = 1;
        elseif points(i) > (inten + T)
            points(i) = 0;
            points(i + 16) = 0;
        else
            points(i) = -1;
            points(i + 16) = -1;
        end
    end

end 

function [corner] = getCorners(img, T, n_s)

    [R, C] = size(img);
    m = 1;

    for y = 4:1:(R-3)
        for x = 4:1:(C-3)           
            points = circle(img, y, x, T);

            max_n = 0;
            if points(1) ~= -1
                n = 1;
            else
                n = 0;
            end
            current_flag = points(1); 

            for i = 2:32              
                if (current_flag == points(i))
                    if current_flag ~= -1
                        n = n + 1;
                    else
                        n = 0;
                    end
                else
                    if n > max_n
                        max_n = n;
                    end
                    if points(i) ~= -1
                        n = 1;
                    else
                        n = 0;
                    end
                    current_flag = points(i);
                end
            end

            if n > max_n
                        max_n = n;
            end

            if max_n >= n_s
                corner(m,1) = y;
                corner(m,2) = x;
                m = m + 1;
            end

        end
    end

end

