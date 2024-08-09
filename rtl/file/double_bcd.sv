module double_bcd(
    input wire [6:0] binary,     // Binary input (0 to 99) - 7 bits to represent 99
    output reg [7:0] bcd    // BCD output - 2 digits, each 4 bits
);
    reg [3:0] tens;
    reg [3:0] units;
    integer i;

    always_comb begin
        // Initialize the BCD digits
        tens = 4'b0000;
        units = 4'b0000;

        // Double Dabble algorithm
        for (i = 6; i >= 0; i = i - 1) begin
            // Add 3 to columns >= 5
            if (tens >= 5) begin
                tens = tens + 3;
            end
            if (units >= 5) begin
                units = units + 3;
            end

            // Shift left one bit
            tens = tens << 1;
            tens[0] = units[3];
            units = units << 1;
            units[0] = binary[i];
        end

        // Combine the two BCD digits into the output
        bcd = {tens, units};
    end
endmodule