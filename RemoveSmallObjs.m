function bw  = RemoveSmallObjs(img,fg,zmax)

    % if it is too far from the camera (Z>zmax) then it's not an object
    [yz,xz] = find(img>zmax); 
    for j=1:length(xz)
        fg(yz(j),xz(j))=0; 
    end
    
    %removing small objects
    SE = strel('disk',5);
    bw_aux= imerode(fg,SE);
    bw= imdilate(bw_aux,SE);

end