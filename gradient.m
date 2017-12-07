function xyz = gradient (r,c,img,factor, region_counts, K)
% r=row vector
% c=column vector
% img=depth image
% factor for thresholding
% K=intrinsic matrix

depthimg=zeros(480,640);
binaryimg=zeros(480,640);

depval=zeros(length(r),1);

for i=1:length(r)
    depthimg(r(i),c(i))=img(r(i),c(i)); 
    binaryimg(r(i),c(i))=1;
end

Im=mat2gray(depthimg); %converting to intensity image
[~,threshold]=edge(Im,'sobel');
BW=edge(Im,'sobel',factor*threshold); %binary image - 1s=edges


[row,col]=find(BW==1); 
binaryimg(sub2ind(size(binaryimg),row,col))=0; 
se = strel('disk',2);
binaryimg=imerode(binaryimg,se);

Ledges = bwlabel(binaryimg);
countsedges=histcounts(Ledges(:));  % counts for the labels
%finding object with counts above region_counts
labelobjedges=find(countsedges(2:end)>region_counts);

xyz=cell(1,length(labelobjedges));
for i=1:length(labelobjedges)
	[r,c]=find(Ledges==labelobjedges(i));
    xyz{i}=zeros(length(r),3);%passar para 3D
    for l=1:length(r)
        xyz{i}(l,3) =img(r(l),c(l))*0.001; % Convert to meters
    end
    xyz{i}(:,1)= (xyz{i}(:,3)/K(1,1)) .*(c-K(1,3)) ;
    xyz{i}(:,2) = (xyz{i}(:,3)/K(2,2)) .*(r-K(2,3)) ;
end


end