function VEC2CSV(filename, vec)

fileID = fopen(filename,'w');
for i = 1 : length(vec)-1
fprintf(fileID,'%s,\n', typeConv_float_to_hex(vec(i)));
end
fprintf(fileID,'%s\n',  typeConv_float_to_hex(vec(end)));
fclose(fileID);

