function hex_str = typeConv_float_to_hex(f_val)

f = single(f_val);
byte_array = typecast(f, 'uint32');
hex_str = dec2hex(byte_array, 8);
hex_str = hex_str(:)';
hex_str = ['0x' hex_str];
