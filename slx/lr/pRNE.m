function taulist = pRNE(thetalist, dthetalist, ddthetalist,thetalist_ref, dthetalist_ref, ddthetalist_ref, ...
                                   g, Ftip, Mlist, Glist, Slist)
n = size(thetalist, 1);
Mi = eye(4);
Ai = zeros(6, n);
AdTi = zeros(6, 6, n + 1);
Vi = zeros(6, n + 1);
Vi_ref = zeros(6, n + 1);
Vdi = zeros(6, n + 1);
Vdi_ref = zeros(6, n + 1);
Vdi(1: 3, 1) = -g;
Vdi_ref(1: 3, 1) = -g;
AdTi(:, :, n + 1) = Adjoint(TransInv(Mlist(:, :, n + 1)));
Fi = Ftip;
taulist = zeros(n, 1);
for i=1: n   
    Mi = Mi * Mlist(:, :, i);
    Ai(:, i) = Adjoint(TransInv(Mi)) * Slist(:, i);

    AdTi(:, :, i) = Adjoint(MatrixExp6(VecTose3(Ai(:, i) ...
                    * -thetalist(i))) * TransInv(Mlist(:, :, i)));

    Vi(:, i + 1) = AdTi(:, :, i) * Vi(:, i) + Ai(:, i) * dthetalist(i);
    Vdi(:, i + 1) = AdTi(:, :, i) * Vdi(:, i) ...
                    + Ai(:, i) * ddthetalist(i) ...
                    + ad(Vi(:, i + 1)) * Ai(:, i) * dthetalist(i);   
    Vdi_ref(:, i + 1) = AdTi(:, :, i)*Vdi_ref(:, i) + Ai(:, i) * ddthetalist_ref(i) ...
                    + ad(Vi(:, i + 1)) * Ai(:, i) * dthetalist_ref(i);
    Vi_ref(:, i + 1) = AdTi(:, :, i) * Vi_ref(:, i) + Ai(:, i) * dthetalist_ref(i);    
end
Fi_list={Fi};
for i = n: -1: 1
    % Fi = AdTi(:, :, i + 1)' * Fi + Glist(:, :, i) * Vdi(:, i + 1) ...
    %      - ad(Vi(:, i + 1))' * (Glist(:, :, i) * Vi(:, i + 1));
    Fi = AdTi(:, :, i + 1)' * Fi + Glist(:, :, i) * Vdi_ref(:, i + 1) ...
         +(Glist(:, :, i)*ad(Vi_ref(:, i + 1))-ad(Vi(:, i + 1))' * (Glist(:, :, i))) * Vi_ref(:, i + 1);
    taulist(i) = Fi' * Ai(:, i);
    Fi_list{end+1} = Fi;
end

end





