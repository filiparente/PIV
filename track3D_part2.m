function [objects, cam1toW, cam2toW] = track3D_part2( imgseq1, imgseq2, cam_params)

    d=length(imgseq1);

    ims1=[];
    ims2=[];
    imgs1=[];
    imgs2=[];
    for i=1:d,
        im1=rgb2gray(imread(imgseq1(i).rgb));
        ims1=[ims1 im1(:)]; 
        im2=rgb2gray(imread(imgseq2(i).rgb));
        ims2=[ims2 im2(:)];
        load(imgseq1(i).depth);
        imgs1(:,:,i) = ReplaceZeros(double(depth_array));
        load(imgseq2(i).depth);
        imgs2(:,:,i) = ReplaceZeros(double(depth_array));
    end

    %background
    medim1=quantile(double(ims1),0.75,2);
    medim2=quantile(double(ims2),0.75,2);
    imgray1=(uint8(reshape(medim1,[480 640])));
    imgray2=(uint8(reshape(medim2,[480 640])));
    bg1q=quantile(imgs1,0.75,3);
    bg2q=quantile(imgs2,0.75,3);
    
    %values for part2
    peak_thresh=0; % increase to limit ; default is 0   
    edge_thresh=20; %decrease to limit ; default is 10
    match_thresh=2.5;
    ransac_thresh=0.10;
    Ninter_ransac=100;
    
    [R T]= RT_c2toc1(peak_thresh,edge_thresh, match_thresh, ransac_thresh, Ninter_ransac,imgray1,imgray2, bg1q, bg2q,cam_params.Kdepth,cam_params.Krgb,cam_params.R,cam_params.T);

    %values for part1
    bg_threshold=100; %130 %threshold use to remove background
    region_counts=2000; %2300 %threshold used to eliminate objects with small nr of pixels
    z_cut=5000;     
    knn=50; %15 %number of nearest neighbours used in knnsearch
    edgethresh=0.1; %0.25%0.1%threshold used to remove big edges in the mst
    mst_grid=0.007; %get one point per "cube" of mst_grid mm

    peak_thresh1=0; % increase to limit ; default is 0
    edge_thresh1=10;%5 %decrease to limit ; default is 10 %5
    match_thresh=1.5; %1.2  

    xyz = {};
    row = {};   
    col={};
    goodxyz1 = {};
    goodxyz2 = {};

    goodxyz21 = {};
    
    %matching objects parameters
    alfa=0.4; %0.4
    Beta=0.6; %0.6
    gama=0.0;
    costOfNonAssignment=0.35;%5; %0.35??
    %cam1
    hsv_descr_old1={};
    keypt_descr_old1={};
    %cam2
    hsv_descr_old2={};
    keypt_descr_old2={};

    ntracks=0;
    final_limits=cell(150,1);
    for k=1:150
        final_limits{k}=zeros(6,d);
    end

    hsv_final={};
    keypt_final={};
    hsv_final_old={};
    keypt_final_old={};
    limits={};
    old_frame=[];
    points_counts_old=[];

    
    for i = 1:d
      
    fg1q = abs(imgs1(:,:,i)-bg1q)> bg_threshold; %returns 0 or 1   
    fg2q = abs(imgs2(:,:,i)-bg2q)> bg_threshold; %returns 0 or 1
    
    %removing z>3000 and erasing small objs
    bw1=RemoveSmallObjs(imgs1(:,:,i),fg1q,z_cut);
    bw2=RemoveSmallObjs(imgs2(:,:,i),fg2q,z_cut);
   
    L1 = bwlabel(bw1); 

    counts1=histcounts(L1(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj1=find(counts1(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background

    % the 3D coordinates of each pixel 
    for k=1:length(labelobj1)
        [row{k} col{k} xyz{k}]=Obj3DCoord(labelobj1,k,L1,imgs1(:,:,i),cam_params.Kdepth);

    end         
    
    %clearing unnecessary variables
    clear labelobj1;
    clear counts1;
    
    %getting the 3D coords of each object without overlapping and background
    for j=1:length(xyz)
                goodxyz1=[goodxyz1 ConnComponents(xyz{j},knn,edgethresh,region_counts,mst_grid)]; %adding cells
    end
     
    %clearing variables
    xyz=[];
    row=[];
    col=[];   
    
   % doing the same for cam2
    L2 = bwlabel(bw2);
    counts2=histcounts(L2(:));  % counts for the labels
    %finding object with counts above region_counts
    labelobj2=find(counts2(2:end)>region_counts); %we start counts1 in index 2 because index 1 corresponds to background
    
    % the 3D coordinates of each pixel 
    for k=1:length(labelobj2) %starting in index 2 because 1 corresponds to the background
        [row{k} col{k} xyz{k}]=Obj3DCoord(labelobj2,k,L2,imgs2(:,:,i),cam_params.Kdepth);            
    end
        
    
    %clearing unnecessary variables
    clear labelobj2;
    clear counts2;
   
    %getting the 3D coords of each object without overlapping and background
    for j=1:length(xyz)
                goodxyz2=[goodxyz2 ConnComponents(xyz{j},knn,edgethresh,region_counts,mst_grid)]; %adding cells
    end
    
    for k=1:length(goodxyz2)
            %points from cam2 in the cam1 reference frame
            goodxyz21{k}=goodxyz2{k}*R+ones(length(goodxyz2{k}),1)*T(1,:);
            %goodxyz21{k}=(inv(cam1toW.R)*(cam2toW.R*goodxyz2{k}'+cam2toW.T*ones(1,length(goodxyz2{k}(:,1))-cam1toW.T*ones(1,length(goodxyz2{k}(:,1))))';
           % goodxyz21{k}=(pinv([cam1toW.R cam1toW.T])*[cam2toW.R cam2toW.R]*goodxyz2{k}')';
    end
    
    %hsv image
    im1=imread(imgseq1(i).rgb);
    
    [hImage1,sImage1,vImage1]=rgb2hsv(im1);
    %red when h=0 and h=1 
    r_ind=[];
    c_ind=[];
    row=[];
    col=[];
    [r_ind,c_ind]=find(hImage1>0.95);
    hImage1(sub2ind(size(hImage1),r_ind,c_ind))=0;
        
    %im1 rgb without background
    im1_wb = imabsdiff(rgb2gray(im1),imgray1)>4; %returns 0 or 1
    [row, col] = find(im1_wb==0); %indexes of background
    
 
    %set background color to white
    for x=1:length(row)
        for g=1:3
            im1(row(x), col(x), g)=255;
        end
    end   
        
   counts1=[];
   [hsv_descr_new1,keypt_descr_new1,counts1] = GetDescriptors(goodxyz1,hImage1, sImage1, im1, cam_params.Krgb,  cam_params.R, cam_params.T,peak_thresh1, edge_thresh1);
 
    %getting the limits of the box for each object
    cam=2;
    %hsv image
    im2=imread(imgseq2(i).rgb);
   
    [hImage2,sImage2,vImage2]=rgb2hsv(im2);
    %red when h=0 and h=1 
    r_ind=[];
    c_ind=[];
    row=[];
    col=[];
    [r_ind,c_ind]=find(hImage2>0.95);
    hImage2(sub2ind(size(hImage2),r_ind,c_ind))=0;
    
     %im1 rgb without background
    im2_wb = imabsdiff(rgb2gray(im2),imgray2)>7; %returns 0 or 1
    [row, col] = find(im2_wb==0); %indexes of background   
 
    %set background color to white
    for x=1:length(row)
        for g=1:3
            im2(row(x), col(x), g)=255;
        end
    end
    
    counts2=[];
   [hsv_descr_new2,keypt_descr_new2,counts2] = GetDescriptors(goodxyz2,hImage2, sImage2, im2, cam_params.Krgb,  cam_params.R, cam_params.T,peak_thresh1, edge_thresh1);
  
   [hsv_final,keypt_final,points_counts,limits]=GetFinalDescriptors(hsv_descr_new1,keypt_descr_new1,hsv_descr_new2,keypt_descr_new2,goodxyz1, goodxyz21);
   
   [final_limits, ntracks,new_frame]= MatchingTracksFinal(i,hsv_final_old,hsv_final,keypt_final_old,keypt_final,points_counts,points_counts_old,final_limits,limits,match_thresh,alfa,Beta,gama, ntracks,costOfNonAssignment,old_frame);

%    [final_limits, ntracks,limits_corr_new{i},assignments1{i},assignments2{i},total_cost1{i},total_cost2{i}]= MatchingTracks(i,counts_new1,counts_old1,counts_new2,counts_old2,hsv_descr_old1,hsv_descr_new1,hsv_descr_old2,hsv_descr_new2,keypt_descr_old1,keypt_descr_new1,keypt_descr_old2,keypt_descr_new2, limits12,final_limits,limits{i,1},limits{i,2},limits_corr,corr_new{i},corr_old,match_thresh,alfa,Beta,gama, ntracks,costOfNonAssignment)

    
    old_frame=[];
    old_frame=new_frame;
    new_frame=[];
    hsv_final_old=[];
    keypt_final_old=[];    
    hsv_final_old=hsv_final;
    keypt_final_old=keypt_final;
    limits=[];
    hsv_final=[];
    keypt_final=[];
    points_counts_old=[];
    points_counts_old=points_counts;
    points_counts=[];
    
    
    %clearing variables
    xyz=[];
    goodxyz1=[];
    goodxyz2=[];
    goodxyz21=[];
    row=[];
    col=[];    
    
    end
    
    objects=[];

    for obj=1:ntracks
        cnt_frame=1;
        objects(obj).frames_tracked=[];
        objects(obj).X=[];
        objects(obj).Y=[];
        objects(obj).Z=[];
        for img=1:d
            if length(find(final_limits{obj}(:,img)~=0))>0
                objects(obj).X(cnt_frame,:)=[final_limits{obj}(1,img) final_limits{obj}(1,img) final_limits{obj}(1,img) final_limits{obj}(1,img) final_limits{obj}(2,img) final_limits{obj}(2,img) final_limits{obj}(2,img) final_limits{obj}(2,img)];  
                objects(obj).Y(cnt_frame,:)=[final_limits{obj}(3,img) final_limits{obj}(3,img) final_limits{obj}(4,img) final_limits{obj}(4,img) final_limits{obj}(3,img) final_limits{obj}(3,img) final_limits{obj}(4,img) final_limits{obj}(4,img)];
                objects(obj).Z(cnt_frame,:)=[final_limits{obj}(5,img) final_limits{obj}(6,img) final_limits{obj}(5,img) final_limits{obj}(6,img) final_limits{obj}(5,img) final_limits{obj}(6,img) final_limits{obj}(5,img) final_limits{obj}(6,img)];
                objects(obj).frames_tracked=[objects(obj).frames_tracked img];
                cnt_frame=cnt_frame+1;
            end
        end       
    end

    cam1toW.R=eye(3);
    cam1toW.T=zeros(3,1);
    cam2toW.R=R';
    cam2toW.T=T(1,:)';

end
