function  [hsv_final,keypt_final,points_counts,final_limits] = GetFinalDescriptors(hsv_descr_new1,keypt_descr_new1,hsv_descr_new2,keypt_descr_new2,goodxyz1, goodxyz12)

obj=1;
hsv_final={};
keypt_final={};
bool=0;
cnt=0;
xyz={};
final_limits={};
intersec_ind={};
points_counts=[];

for k=1:length(goodxyz1)
    cnt=0;
    %p1=pointCloud(goodxyz1);
    xyz{k}=[];
    xyz{k}=goodxyz1{k};
    for j=1:length(goodxyz12)
        [bool]=IsIntercept(goodxyz1{k},goodxyz12{j});
        if(bool==1 & cnt==0)
            hsv_final{k}=hsv_descr_new1{k}+hsv_descr_new2{j};
            keypt_final{k}=[keypt_descr_new1{k} keypt_descr_new2{j}];
            bool=0;
            cnt=cnt+1;
            xyz{k}=[xyz{k}; goodxyz12{j}];
            intersec_ind{k}=j;
           % p12=pointCloud(goodxyz12);
           % paux=pcmerge(p1,p12,0.001);
            %goodxyz;
        elseif bool==1 & cnt~=0
            hsv_final{k}=hsv_final{k}+hsv_descr_new2{j};
            keypt_final{k}=[keypt_final{k} keypt_descr_new2{j}];
            bool=0;
            cnt=cnt+1;
            xyz{k}=[xyz{k}; goodxyz12{j}];
            intersec_ind{k}=[intersec_ind{k} j];
           % p12=pointCloud(goodxyz12);
           % paux=pcmerge(paux,p12,0.001);
            %goodxyz;
        end
    end
    if(cnt==0)
        hsv_final{k}=hsv_descr_new1{k};
        keypt_final{k}=keypt_descr_new1{k};
        intersec_ind{k}=0;
        %paux=p1;
    end
    
end

nobj=length(hsv_final);

rep_cnt=0;
rep_ind=[];
for k=1:length(goodxyz12)
    rep_cnt=0;
    rep_ind=[];
    for j=1:length(xyz)
        if ismember(k,intersec_ind{j})
            rep_cnt=rep_cnt+1;
            rep_ind=[rep_ind j];
        end
    end
    if rep_cnt>1
        for rep=2:length(rep_ind)
         hsv_final{rep_ind(1)}=hsv_final{rep_ind(1)}+hsv_final{rep_ind(rep)};
         keypt_final{rep_ind(1)}=[keypt_final{rep_ind(1)} keypt_final{rep_ind(rep)}];
         xyz{rep_ind(1)}=[xyz{rep_ind(1)}; xyz{rep_ind(rep)}];
         hsv_final{rep_ind(rep)}(1,1)=9999;
         keypt_final{rep_ind(rep)}(1,1)=9999;
         xyz{rep_ind(rep)}(1,1)=9999;
         intersec_ind{rep_ind(1)}=[intersec_ind{rep_ind(1)} intersec_ind{rep_ind(rep)}]; 
         intersec_ind{rep_ind(rep)}=0;
        end
    end
end

for k=1:length(goodxyz12)
    cnt=0;
    for j=1:length(goodxyz1)
        [bool]=IsIntercept(goodxyz1{j},goodxyz12{k});
        if(bool==1 & cnt==0)
            cnt=cnt+1;
        end
    end
        if(cnt==0)
        nobj=nobj+1;
        hsv_final{nobj}=hsv_descr_new2{k};
        keypt_final{nobj}=keypt_descr_new2{k};
        xyz{nobj}=goodxyz12{k};
        end
end


% 
% obj=0;
% for k=1:length(goodxyz12)
%     cnt=0;
%     for j=1:length(goodxyz1)
%         [bool]=IsIntercept(goodxyz1{j},goodxyz12{k});
%         if(bool==1 & cnt==0)
%             cnt=cnt+1;
%             obj=j;
%             bool=0;
%             %goodxyz;
%         elseif bool==1 & cnt~=0
%             hsv_final{obj}=hsv_final{obj}+hsv_final{j};
%             keypt_final{obj}=[keypt_final{obj} keypt_final{j}];
%             xyz{obj}=[xyz{obj}; xyz{j}];
%             hsv_final{j}=[];
%             keypt_final{j}=[];
%             xyz{j}=[];
%             bool=0;
%             cnt=cnt+1;
%             %goodxyz;
%         end
%     end
%     if(cnt==0)
%         nobj=nobj+1;
%         hsv_final{nobj}=hsv_descr_new2{k};
%         keypt_final{nobj}=keypt_descr_new2{k};
%         xyz{nobj}=goodxyz12{k};
%     end
% end

%  keypt_final(cellfun('isempty',keypt_final)) = []; %apagar cells vazias
%  hsv_final(cellfun('isempty',hsv_final)) = []; %apagar cells vazias
%  xyz(cellfun('isempty',xyz)) = []; %apagar cells vazias
for m=1:length(hsv_final)
    if hsv_final{m}(1,1)==9999;
        hsv_final(m)=[];
        keypt_final(m)=[];
        xyz(m)=[];
    end    
end

 for i=1:length(xyz)
     final_limits{i}=getLimits(xyz{i});
%      paux=pointCloud(xyz{i});
%      paux_ds=pcdownsample(paux,'gridAverage',0.05);
% %      points_counts(i)=paux_ds.Count;
    % [~,points_counts(i)]=convhull(xyz{i});
    points_counts(i)=i;
     
 end
    
end

