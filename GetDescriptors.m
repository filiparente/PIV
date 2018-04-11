function [hsv_descr,keypt_descr,counts] = GetDescriptors(goodxyz, hImage1, sImage1, im1, K, R, T,peak_thresh, edge_thresh)

hsv_descr={};
keypt_descr={};
counts=[];
% lbp_features={};

for k=1:length(goodxyz)            
            
            %HSV DESCRIPTOR
            %coordinates in the rgb image corresponding to our object 
            uv_fromd=[];
            hcnt1=[];
            scnt1=[];
            uv_fromd=round(uvrgb_fromd(goodxyz{k},K,R,T)); 

            good_ind=find(uv_fromd(:,2)>0 & uv_fromd(:,1)>0 & uv_fromd(:,2)<480 & uv_fromd(:,1)<640); %it doesnt work when u or v are 0 because indexes in matlab start at 1
         
            %hsv histograms
            hcnt1=histcounts(hImage1(sub2ind(size(hImage1),uv_fromd(good_ind,2),uv_fromd(good_ind,1))),10); %rows->v, columns->u
            scnt1=histcounts(sImage1(sub2ind(size(sImage1),uv_fromd(good_ind,2),uv_fromd(good_ind,1))),10); %10 bins         
          %  hcnt1=hcnt1/sum(hcnt1);
           % scnt1=scnt1/sum(scnt1);
            hsv_descr{k}=[hcnt1'; scnt1']; %descriptor
            hsv_descr{k}=hsv_descr{k};
            
            %KEYPTS DESCRIPTOR
            imgray=rgb2gray(im1);
            imgObj=imcrop(imgray,[min(uv_fromd(:,1)) min(uv_fromd(:,2)) max(uv_fromd(:,1))-min(uv_fromd(:,1)) max(uv_fromd(:,2))-min(uv_fromd(:,2))]);
            %object keypts 
            Fnew1=[];
            [Fnew1,keypt_descr{k}]=vl_sift(single(imgObj),'PeakThresh', peak_thresh,'edgethresh', edge_thresh);
           
            counts(k)=length(goodxyz{k});
            %texture feature for each object
            %lbp_features{k}=extractLBPFeatures(imgObj);
%             

end