function uv_fromd = uvrgb_fromd(xyz,K,R,T)

    P3d=[xyz(:,1)'; xyz(:,2)'; xyz(:,3)'; ones(1,length(xyz(:,1)))]; 
    P2d=K*[R T]*P3d;    
    uv_fromd(:,1)=(P2d(1,:)./P2d(3,:))';
    uv_fromd(:,2)=(P2d(2,:)./P2d(3,:))';

end