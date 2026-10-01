function output = qc_bumpsEndpoint(input)

%minimize problem functional
output.objective = input.phase(1).integral(1); 

end