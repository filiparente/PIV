function tr = GetProcrustes(im1, im2,dep1,dep2,DepthK, RGBK, R_d_to_rgb,T_d_to_rgb)
    %para DepthK mandar Depth_Cam.K
    %para RGBK mandar RGB_cam.K
    %usa as funcoes xyzsus e get_rgbd do lab1 do prof
    
    
    %dep2(find(dep2(:)>4000))=0;
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', DepthK,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', DepthK,1,0);
    %REGISTER RGB TO DEPTH
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGBK);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGBK);
    figure(1);imagesc(rgbd1 );
    figure(2);imagesc(rgbd2 );
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
    figure(3);clf; showPointCloud(pc1);
    figure(4);clf; showPointCloud(pc2);
    %GET CORRESPONDING POINTS
    np=6;
    figure(1);x1=zeros(np,1);y1=x1;x2=y1;y2=x1;
    for i=1:np,
        figure(1);
        [xa ya]=ginput(1);text(xa,ya,int2str(i));
        xa=fix(xa);ya=fix(ya);
        x1(i)=xa;y1(i)=ya;
        aux1=xyz1(sub2ind([480 640],ya,xa),:);
        figure(3);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
        hold off;
        figure(2);
        [xa ya]=ginput(1);text(xa,ya,int2str(i));
        xa=fix(xa);ya=fix(ya);
        x2(i)=xa;y2(i)=ya;
        aux1=xyz2(sub2ind([480 640],fix(ya),fix(xa)),:);
        figure(4);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
        hold off;drawnow;
    end

    ind1=sub2ind(size(dep2),y1,x1);
    ind2=sub2ind(size(dep2),y2,x2);

    P1=xyz1(ind1,:);
    P2=xyz2(ind2,:);
    inds=find((P1(:,3).*P2(:,3))>0);
    P1=P1(inds,:);P2=P2(inds,:);
    [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
    % aceder à transf: 
    % Rotacao: tr.T   Translacao: tr.c
end
