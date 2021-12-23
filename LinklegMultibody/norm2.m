% Calculate the L2 norm of a vector

function L = norm2(v)
    if v'*v == 0
        L = 0;
    else
        L = sqrt(v'*v);
    end
end