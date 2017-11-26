function [r c xyzCam] = Obj3DCoord(labelobj,k,L,img,K)
    
[r,c]=find(L==labelobj(k)); %getting the coordinates in the image of each object pixel

xyzCam=zeros(length(r),3);
%passar para 3D
for l=1:length(r)
    xyzCam(l,3) =img(r(l),c(l))*0.001; % Convert to meters
end

xyzCam(:,1)= (xyzCam(:,3)/K(1,1)) .*(c-K(1,3)) ;
xyzCam(:,2) = (xyzCam(:,3)/K(2,2)) .*(r-K(2,3)) ;

end