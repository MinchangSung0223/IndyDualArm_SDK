function adV = ad(V)
omgmat = VecToso3(V(4: 6));
adV = [omgmat, VecToso3(V(1: 3)); ...
       zeros(3), omgmat];
end
