import struct
import argparse

def read_binary_file(file_path):
    with open(file_path, 'rb') as file:
        data = file.read()
    return list(data)

def write_c_header_file(data, output_path):
    array_size = len(data)
    with open(output_path, 'w') as file:
        file.write('#ifndef DATA_ARRAY_H\n')
        file.write('#define DATA_ARRAY_H\n\n')
        file.write(f'unsigned char data_array[{array_size}] = {{')
        
        for i, byte in enumerate(data):
            if i % 12 == 0:
                file.write('\n    ')
            file.write(f'{byte}, ')
        
        file.write('\n};\n\n')
        file.write('#endif // DATA_ARRAY_H\n')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert a binary file to a C header file.')
    parser.add_argument('input_file', help='The path to the input binary file.')
    parser.add_argument('output_file', help='The path to the output C header file.')
    
    args = parser.parse_args()
    
    input_file_path = args.input_file
    output_file_path = args.output_file
    
    data = read_binary_file(input_file_path)
    if data:
        write_c_header_file(data, output_file_path)