function output_args = gain(input_args)

Xerr = input_args(1);
Yerr = input_args(2);
out1 = input_args(3);
out2 = input_args(4);

if( (abs(Xerr) < 0.01) && (abs(Yerr) < 0.01) )
   out1 = 0;
   out2 = 0;
else
    out1 = input_args(3);
    out2 = input_args(4);
    %output_args(3) = 0;
end

output_args(1) = out1;
output_args(2) = out2;
% Circle            :   v = 0.5, w = 2.0
% Sine Wave         :   v = 0.2, w = 2.5
% Arbitrary Curve   :   v = 0.1, w = 2.5
