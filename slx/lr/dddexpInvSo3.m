function ret = dddexpInvSo3(xi, xidot,xiddot)
    dexpInvxi= dexpInvSo3(xi);
    ret = -2*dexpInvxi*ddexpSo3(xi,xidot)*ddexpInvSo3(xi,xidot)-dexpInvxi*dddexpSo3(xi,xidot,xiddot)*dexpInvxi;
end