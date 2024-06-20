import struct

def read_binary_file(file_path):
    with open(file_path, 'rb') as file:
        data = file.read()
    return list(data)

def write_c_header_file(data, output_path):
    with open(output_path, 'w') as file:
        file.write('#ifndef DATA_ARRAY_H\n')
        file.write('#define DATA_ARRAY_H\n\n')
        file.write('unsigned char data_array[] = {\n')
        
        for i, byte in enumerate(data):
            if i % 12 == 0:
                file.write('\n    ')
            file.write(f'{byte}, ')
        
        file.write('\n};\n\n')
        file.write('#endif // DATA_ARRAY_H\n')

if __name__ == "__main__":
    input_file_path = 'sample.raw'
    output_file_path = 'sample.h'
    
    data = read_binary_file(input_file_path)
    write_c_header_file(data, output_file_path)
    print(f"C header file '{output_file_path}' created successfully.")
