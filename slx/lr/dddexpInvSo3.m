function ret = dddexpInvSo3(xi, xidot,xiddot)
    dexpInvxi= dlog3(xi);
    ret = -2*dexpInvxi*ddlog3(xi,xidot)*ddlog3(xi,xidot)-dexpInvxi*dddexpSo3(xi,xidot,xiddot)*dexpInvxi;
end