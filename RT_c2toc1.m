function [R T] = RT_c2toc1(peak_thresh,edge_thresh, match_thresh, ransac_thresh, Niter_ransac,imgray1,imgray2, meddep1, meddep2,DepthK,RGBK,R_d_to_rgb,T_d_to_rgb)
  
    [F1,d1]=vl_sift(single(imgray1),'PeakThresh', peak_thresh,'edgethresh', edge_thresh);
    [F2,d2]=vl_sift(single(imgray2),'PeakThresh', peak_thresh,'edgethresh', edge_thresh);
   
    %finding matches 
    [matches, scores]= vl_ubcmatch(d1,d2,match_thresh);
        
    %find 2D coordinates in the RGB cam with F1 and F2
    uv1rgb(:,1) = F1(1,matches(1,:)) ;
    uv2rgb(:,1)= F2(1,matches(2,:)) ;
    uv1rgb(:,2)= F1(2,matches(1,:)) ;
    uv2rgb(:,2) = F2(2,matches(2,:)) ;

    xyz1 = get_xyzasus(meddep1 (:),[480 640],1:640*480,DepthK,1,0);
    xyz2 = get_xyzasus(meddep2 (:),[480 640],1:640*480,DepthK,1,0);

    P3d_1=[xyz1(:,1)'; xyz1(:,2)'; xyz1(:,3)'; ones(1,length(xyz1(:,1)))]; 
    P2d_1=RGBK*[R_d_to_rgb T_d_to_rgb]*P3d_1;    
    uv_fromd1(:,1)=(P2d_1(1,:)./P2d_1(3,:))';
    uv_fromd1(:,2)=(P2d_1(2,:)./P2d_1(3,:))';

    P3d_2=[xyz2(:,1)'; xyz2(:,2)'; xyz2(:,3)'; ones(1,length(xyz2(:,1)))]; 
    P2d_2=RGBK*[R_d_to_rgb T_d_to_rgb]*P3d_2;    
    uv_fromd2(:,1)=(P2d_2(1,:)./P2d_2(3,:))';
    uv_fromd2(:,2)=(P2d_2(2,:)./P2d_2(3,:))';

    bestuv1=zeros(length(uv1rgb(:,1)),1);
    bestuv2=zeros(length(uv2rgb(:,1)),1);

    for i=1:length(uv1rgb(:,1))
        [~,bestuv1(i)]=min(sqrt((uv_fromd1(:,1)-uv1rgb(i,1)).^2+(uv_fromd1(:,2)-uv1rgb(i,2)).^2));
        [~,bestuv2(i)]=min(sqrt((uv_fromd2(:,1)-uv2rgb(i,1)).^2+(uv_fromd2(:,2)-uv2rgb(i,2)).^2));

    end

    xyz_keypts1=xyz1(bestuv1,:);
    xyz_keypts2=xyz2(bestuv2,:);

    %find best inliers with RANSAC
    best_inliers=[];
    inliers=[];
    aux=fix(rand(4*Niter_ransac,1)*length(xyz_keypts1))+1;
    for k=1:Niter_ransac-4
        xyz1_sample=xyz_keypts1(aux(4*k:4*k+3),:);
        xyz2_sample=xyz_keypts2(aux(4*k:4*k+3),:);

       %for each group:
        %compute R and T from cam2 to cam 1 using procrustes 
        [d,xx,tr_trial]=procrustes(xyz1_sample,xyz2_sample,'scaling',false,'reflection',false); %computes transformation without scale and reflection

        %count the nr of inliers
        dist=xyz_keypts1-(xyz_keypts2*tr_trial.T+ones(length(xyz_keypts2),1)*tr_trial.c(1,:)); %distance between all keypoints
        inliers=find(sqrt(sum(dist.^2,2))<ransac_thresh);
        
        %keep tranformation and inliers if there are more inliers than the
        %maximum find until now
        if length(inliers)>length(best_inliers)
            best_inliers=inliers;
        end 
        inliers=[];
    end
    %recompute R and T using the model with more inliers
    [d,xx,tr]=procrustes(xyz_keypts1(best_inliers,:),xyz_keypts2(best_inliers,:),'scaling',false,'reflection',false);
    R = tr.T;
    T = tr.c;
end

