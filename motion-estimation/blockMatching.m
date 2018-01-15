clear all;
close all;
clc;


%-------------------------------------------------------------------------------------------

                                        % Motion Estimation

%-------------------------------------------------------------------------------------------
% Block matching algorithm for motion estimation
%-------------------------------------------------------------------------------------------
blockSize=8;

halfName = 'MVI_4117_frame_0';
extension = '.bmp';
startingFrame = imread('MVI_4117_frame_0669.bmp');
endingFrame = imread('MVI_4117_frame_0720.bmp');

% tracking object
imcurrent = imcomplement(startingFrame);
diff_im = imsubtract(imcurrent(:,:,3), rgb2gray(imcurrent));
diff_im = medfilt2(diff_im,[3 3]);
diff_im = im2bw(diff_im,0.1);
diff_im = bwareaopen(diff_im,50);
bw = bwlabel(diff_im,4);
stats=regionprops(logical(bw),'BoundingBox');
%startingFrame(458,288,:)=0;
imshow(startingFrame);
hold on
% extracting leaf pixel from 'pos' variable
% draw the leaf
pos = stats(9).BoundingBox;
rectangle('Position', pos,'Curvature',[1 1],'EdgeColor','r','LineWidth',2);
axis equal; axis tight; axis off;  
set(gcf, 'Color', 'White');  title('tracking object', 'FontSize', 17);
hold off

startingPixel=[458,288];

% preallocating space
pathline=zeros;

for frames=669:720
   % reading frames one-by-one
   frameNumber = frames;
   currentFrame = strcat(halfName,num2str(frameNumber),extension);
   referenceFrame = strcat(halfName,num2str(frameNumber+1),extension);
   currentImage= imread(currentFrame);
   referenceImage = imread(referenceFrame);

   % initializing the pathline
   pathline(frames,1) = startingPixel(1);
   pathline(frames,2) = startingPixel(2);
   
   % converts the truecolor images RGB to the grayscale intensity image
   grayCurr=rgb2gray(currentImage);
   grayRef=rgb2gray(referenceImage);
   
   % determining the image size
   [row, col] = size(grayCurr);
   
   min=10000;
   % 2*blockSize x 2*blockSize
   for i=-blockSize:blockSize
       for j=-blockSize:blockSize
           sumDiff=0;
           % blockSize x blockSize
           for k=1:blockSize
               for m=1:blockSize
                   if((startingPixel(1)+i<row)&&(startingPixel(2)+m+j<col)&&(startingPixel(1)+k+i<row)&&(startingPixel(2)+m<col))
                       diff = abs((grayCurr(startingPixel(1)+k,startingPixel(2)+m))-(grayRef(startingPixel(1)+k+i,startingPixel(2)+m+j)));
                       sumDiff = sum(diff(:)) + sumDiff;
                   else
                       diff = 255;
                   end
               end
           end 
           if (sumDiff < min)
               min = sumDiff;
               tempPixel(1)=startingPixel(1)+i;
               tempPixel(2)=startingPixel(2)+j;
           end
       end
   end
   startingPixel = tempPixel;
end

figure, imshow(endingFrame);
axis equal; axis tight; axis off;  
set(gcf, 'Color', 'White');  title('block matching for motion estimation', 'FontSize', 17);

% draw the pathline
hold on
for frames=669:719
    quiver(pathline(frames,2),pathline(frames,1),pathline(frames+1,2)-pathline(frames,2),pathline(frames+1,1)-pathline(frames,1),'r');
end
hold off
