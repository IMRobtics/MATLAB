function output_args = alpha_correct(input_args)

a = input_args(1);
a_1 = input_args(2);

if( abs(a - a_1) > (pi) )
    if(a > pi)
        a_out = -1 * abs( abs(a) - abs(a_1) );
        %a_out =  a_1;
    else
        a_out = abs( abs(a) - abs(a_1) );
        %a_out = a_1;
    end
else
    a_out = a;
end
%a_out = a;

output_args = a_out;

end