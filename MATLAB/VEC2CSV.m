function VEC2CSV(filename, vec, dec_prec)

if nargin < 3
    dec_prec = 6;
end

fileID = fopen(filename,'w');
for i = 1 : length(vec)-1
fprintf(fileID,['%.' num2str(dec_prec) 'f, '], vec(i));
end
fprintf(fileID,['%.' num2str(dec_prec) 'f\n']',vec(end));
fclose(fileID);

