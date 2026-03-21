function out = outer_nest_function(x,y)
    
    [useless_var, useful_var] = inner_nest_function(x,y);

    even_more_useless_var = useless_var + 50;

    out = x + y + useful_var;
end