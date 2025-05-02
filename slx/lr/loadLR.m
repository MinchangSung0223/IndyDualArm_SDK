function [M,Slist,Blist,Mlist,Glist]=loadLR(json_name)
    filedata = fileread(json_name);
    jsondata = jsondecode(filedata);
    M = jsondata.M;
    Glist  = jsondata.Glist;
    Slist = jsondata.Slist;
    Blist = jsondata.Blist;
    Mlist = jsondata.Mlist;
    Gi_cell={}
    Mi_cell={}
    
    for  i= 1:1:length(Slist)
            Gi = reshape(Glist(i,:,:),6,6);
            Gi_cell{end+1} = Gi;
    end
    for i = 1:1:length(Slist)+1
        Mi = reshape(Mlist(i,:,:),4,4);
        Mi_cell{end+1} = Mi;
    end
    Glist=[];
    Mlist = [];
    for i = 1:1:length(Slist)
        Glist = cat(3, Glist, Gi_cell{i});
    end
    for i = 1:1:length(Slist)+1
        Mlist = cat(3, Mlist,  Mi_cell{i});
    end
    
end