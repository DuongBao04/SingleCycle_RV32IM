`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2025 02:16:52 PM
// Design Name: 
// Module Name: mul
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
module mul (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [4:0] ALUCtrl,
    output reg  [31:0] result
);

    reg signed   [63:0] A_s;  
    reg signed   [63:0] B_s;  
    reg          [63:0] A_u;
    reg          [63:0] B_u;
    reg signed   [63:0] R_ss;
    reg signed   [63:0] R_su;
    reg          [63:0] R_uu;

    integer i;

    always @(*) begin
        // SIGN-EXTEND
        A_s = {{32{A[31]}}, A};
        B_s = {{32{B[31]}}, B};

        // UNSIGNED × UNSIGNED
        A_u = {{32{1'b0}}, A};
        B_u = {{32{1'b0}}, B};

        R_ss = 0;
        R_su = 0;
        R_uu = 0;

        // ---------- SIGNED × SIGNED (MUL, MULH) ----------
        for (i = 0; i < 32; i = i + 1) begin
            if (B_s[i] == 1'b1)
                R_ss = R_ss + (A_s <<< i);
        end

        // ---------- SIGNED × UNSIGNED (MULHSU) ----------
        for (i = 0; i < 32; i = i + 1) begin
            if (B_u[i] == 1'b1)
                R_su = R_su + (A_s <<< i);
        end

        // ---------- UNSIGNED × UNSIGNED (MULHU) ----------
        for (i = 0; i < 32; i = i + 1) begin
            if (B_u[i] == 1'b1)
                R_uu = R_uu + (A_u << i);
        end

        // ---------- SELECT OUTPUT ----------
        case (ALUCtrl)
            `ALU_MUL:   result = R_ss[31:0];   // MUL (signed × signed low)
            `ALU_MULH:  result = R_ss[63:32];  // MULH (signed × signed high)
            `ALU_MULSU: result = R_su[63:32];  // MULHSU (signed × unsigned high)
            `ALU_MULU:  result = R_uu[63:32];  // MULHU (unsigned × unsigned high)
        endcase
    end
endmodule
