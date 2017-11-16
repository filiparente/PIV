function newimg = ReplaceZeros(img)

[gr, gc, gv] = find(img);
F = scatteredInterpolant(gr, gc, gv,'nearest');
[br, bc] = find(~img);
replacements = F(br, bc);
img( sub2ind(size(img), br, bc) ) = replacements;
newimg=img;

end