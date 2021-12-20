function out = MAT2CMSIS(in)

in = in';
out = in(:)';

fprintf("%1.1ff, ",out(1:end-1));
fprintf("%1.1ff\n",out(end));
