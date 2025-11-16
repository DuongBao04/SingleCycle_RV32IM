`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2025 02:51:29 PM
// Design Name: 
// Module Name: div
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "defines.vh"
module div (
    input  [31:0] A,
    input  [31:0] B,
    input  [4:0] ALUCtrl,
    output reg [31:0] result
);

    reg [31:0] dividend, divisor;
    reg [31:0] quotient, remainder;
    reg signA, signB;
    reg signedOp;

    integer i;

    always @(*) begin
        signedOp = (ALUCtrl == `ALU_DIV) || (ALUCtrl == `ALU_REM);
        if (signedOp) begin
            signA = A[31];
            signB = B[31];
            dividend = signA ? -A : A;
            divisor  = signB ? -B : B;
        end else begin
            signA = 0;
            signB = 0;
            dividend = A;
            divisor  = B;
        end

        if (divisor > 0) begin
            quotient  = 0;
            remainder = 0;

            for (i = 31; i >= 0; i = i - 1) begin
                // shift in next bit
                remainder = { remainder[30:0], dividend[i] };

                if (remainder >= divisor) begin
                    remainder = remainder - divisor;
                    quotient[i] = 1;
                end
            end

            case (ALUCtrl)
                `ALU_DIV:  // DIV (signed)
                    result = (signA ^ signB) ? -quotient : quotient;

                `ALU_DIVU:  // DIVU
                    result = quotient;

                `ALU_REM:  // REM (signed)
                    result = signA ? -remainder : remainder;

                `ALU_REMU:  // REMU
                    result = remainder;

                default:
                    result = 0;
            endcase
        end
    end
endmodule

