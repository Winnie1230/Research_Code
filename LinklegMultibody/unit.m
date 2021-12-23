% Calculate the unit vector of a vector

function V = unit(v)
    if v'*v == 0
        V = [0 0 0]';
    else
        V = v/sqrt(v'*v);
    end
end