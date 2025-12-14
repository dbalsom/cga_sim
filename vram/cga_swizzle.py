import sys
import os

def swizzle_cga_memory(input_filename, output_filename):
    """
    Converts a linear 16KB CGA memory dump (CPU View) into a 
    physically banked format (Hardware/CRTC View).
    
    Logic:
    - CPU View: Char, Attr, Char, Attr...
    - Hardware View: 
        Bank 0: Characters (Low 8k)
        Bank 1: Attributes (Low 8k)  <-- Offset by 4096 (A12 High)
        Bank 2: Characters (High 8k)
        Bank 3: Attributes (High 8k) <-- Offset by 4096 (A12 High)
    """
    
    EXPECTED_SIZE = 16384 # 16KB
    
    try:
        with open(input_filename, 'rb') as f:
            data = f.read()
    except FileNotFoundError:
        print(f"Error: File '{input_filename}' not found.")
        return

    file_size = len(data)
    print(f"Read {file_size} bytes from {input_filename}...")

    if file_size != EXPECTED_SIZE:
        print(f"Warning: Expected 16384 bytes, got {file_size}. "
              "Padding or truncating may occur.")
        # Pad with zeros if too short
        if file_size < EXPECTED_SIZE:
            data = data + b'\x00' * (EXPECTED_SIZE - file_size)
        # Truncate if too long
        data = data[:EXPECTED_SIZE]

    # Create a buffer for the scrambled output
    output_buffer = bytearray(EXPECTED_SIZE)

    # Process the data
    # We iterate through the input LINEARLY
    for linear_addr in range(EXPECTED_SIZE):
        byte = data[linear_addr]

        # --- The Address Bit Swizzle ---
        
        # 1. Is this an Odd byte (Attribute) or Even byte (Character)?
        #    This becomes the Physical A12 bit (adding 4096).
        is_odd = linear_addr & 1
        
        # 2. Are we in the upper 8KB of the dump? (A13)
        #    This stays as Physical A13 (adding 8192).
        is_upper_8k = linear_addr & 0x2000
        
        # 3. The Offset within the bank.
        #    We take bits 1-12 of the linear address and shift them down.
        #    (linear_addr % 8192) // 2
        offset = (linear_addr & 0x1FFF) >> 1

        # Calculate Physical Address
        # Base is 0, 4096, 8192, or 12288
        base_addr = 0
        if is_upper_8k:
            base_addr += 8192  # Bank 2 or 3
        if is_odd:
            base_addr += 4096  # Bank 1 or 3 (The Attribute Offset)

        final_addr = base_addr + offset
        
        output_buffer[final_addr] = byte

    # Write output
    try:
        with open(output_filename, 'wb') as f:
            f.write(output_buffer)
        print(f"Success! Swizzled data written to: {output_filename}")
        print("Layout:")
        print("  0x0000 - 0x0FFF: Page 0/1 Characters")
        print("  0x1000 - 0x1FFF: Page 0/1 Attributes")
        print("  0x2000 - 0x2FFF: Page 2/3 Characters")
        print("  0x3000 - 0x3FFF: Page 2/3 Attributes")
    except IOError as e:
        print(f"Error writing output file: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python cga_swizzle.py <input_dump.bin> [output_filename]")
    else:
        in_file = sys.argv[1]
        if len(sys.argv) > 2:
            out_file = sys.argv[2]
        else:
            # Auto-generate output filename
            name, ext = os.path.splitext(in_file)
            out_file = f"{name}_banked{ext}"
        
        swizzle_cga_memory(in_file, out_file)